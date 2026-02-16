/*
 * おさんぽメーター / おさんぽでんしゃ - M5StickC Plus2
 * BLE v1: 8バイト (進捗uint32 + 距離uint32)
 * BLE v2: コマンドプレフィックス方式
 *   0x00: 進捗更新 (16B)
 *   0x01: 駅名登録
 *   0x02: 駅クリア
 * v1/v2自動判別・後方互換
 */

#include <M5StickCPlus2.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-123456789def"

// 駅データ
#define MAX_STATIONS 10
#define MAX_NAME_BYTES 20  // UTF-8駅名(6文字*3+1)

struct StationInfo {
  char name[MAX_NAME_BYTES];
  uint8_t progressPct;  // 0-100: ルート上の位置(%)
};

StationInfo stations[MAX_STATIONS];
uint8_t stationCount = 0;
bool v2Mode = false;  // v2プロトコルが検出されたか

// 状態管理
enum State {
  STATE_WAITING,
  STATE_CONNECTED,
  STATE_PROGRESS,
  STATE_STATION_ARRIVED,
  STATE_GOAL
};

enum DisplayMode {
  DISPLAY_PROGRESS,
  DISPLAY_NEXT_STATION
};

State currentState = STATE_WAITING;
DisplayMode displayMode = DISPLAY_PROGRESS;
bool deviceConnected = false;
bool prevConnected = false;

uint32_t currentProgress = 0;
uint32_t prevProgress = 0;
int lastMilestone = -1;
uint32_t distToNextStation = 0;
uint32_t distToFinalDest = 0;
uint8_t currentNextStation = 0;
uint8_t prevNextStation = 0;
uint8_t totalStations = 0;

unsigned long goalStartTime = 0;
unsigned long lastBlinkTime = 0;
bool goalBlinkOn = true;
unsigned long lastBatteryUpdate = 0;
unsigned long stationArrivedTime = 0;

// v1互換用
uint32_t remainingDistance = 0;
bool showDistance = false;

// --- メロディ定義 ---

// 駅到着チャイム (JR風 4音)
const int stationChime[][2] = {
  {1047, 200}, {1319, 200}, {1568, 200}, {2093, 400}
};
const int stationChimeLen = 4;

// 愛の挨拶 (Salut d'Amour) Op.12 終結部
// E major: 最後の穏やかな下降フレーズ → E(長音)で終止
const int salutDamourEnding[][2] = {
  {988, 300},   // B5
  {880, 200},   // A5
  {831, 200},   // G#5
  {659, 300},   // E5
  {0, 80},
  {740, 200},   // F#5
  {659, 600},   // E5 (長音で終止)
};
const int salutEndLen = 7;

// v1用: マイルストーンメロディ (3音)
const int milestoneMelody[][2] = {
  {784, 100}, {988, 100}, {1319, 200}
};
const int milestoneLen = 3;

// ゴールメロディ (豪華ファンファーレ)
const int goalMelody[][2] = {
  {1047, 200}, {1319, 200}, {1568, 200}, {2093, 400},
  {0, 150},
  {988, 120}, {880, 120}, {831, 120}, {880, 120},
  {1319, 300}, {0, 100},
  {1175, 120}, {1047, 120}, {988, 120}, {1047, 120},
  {1568, 500}
};
const int goalLen = 16;

// --- BLEコールバック ---
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

class ProgressCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    uint8_t* data = pCharacteristic->getData();
    size_t len = pCharacteristic->getLength();
    if (len < 1) return;

    uint8_t cmd = data[0];

    // v2プロトコル判定: 先頭が0x00〜0x02ならv2
    if (cmd <= 0x02 && len >= 1) {

      if (cmd == 0x00 && len >= 15) {
        // v2: 進捗更新
        v2Mode = true;
        uint32_t prog = data[1] | (data[2]<<8) | (data[3]<<16) | (data[4]<<24);
        uint32_t dNext = data[5] | (data[6]<<8) | (data[7]<<16) | (data[8]<<24);
        uint32_t dFinal = data[9] | (data[10]<<8) | (data[11]<<16) | (data[12]<<24);
        uint8_t nextIdx = data[13];
        uint8_t totalSt = data[14];

        if (prog > 100000) prog = 100000;
        prevProgress = currentProgress;
        currentProgress = prog;
        distToNextStation = dNext;
        distToFinalDest = dFinal;
        prevNextStation = currentNextStation;
        currentNextStation = nextIdx;
        totalStations = totalSt;
        remainingDistance = dFinal;  // v1互換

      } else if (cmd == 0x01 && len >= 4) {
        // v2: 駅名登録
        // フォーマット: [0x01, idx, nameLen, name..., progressPct]
        v2Mode = true;
        uint8_t idx = data[1];
        uint8_t nameLen = data[2];
        if (idx < MAX_STATIONS && nameLen < MAX_NAME_BYTES) {
          memcpy(stations[idx].name, &data[3], nameLen);
          stations[idx].name[nameLen] = '\0';
          // 駅名の後にprogressPct(0-100)が付加されている場合
          if (len >= (size_t)(3 + nameLen + 1)) {
            stations[idx].progressPct = data[3 + nameLen];
          } else {
            stations[idx].progressPct = 0;
          }
          if (idx >= stationCount) stationCount = idx + 1;
        }

      } else if (cmd == 0x02) {
        // v2: 駅クリア
        v2Mode = true;
        stationCount = 0;
        currentNextStation = 0;
        for (int i = 0; i < MAX_STATIONS; i++) {
          memset(stations[i].name, 0, MAX_NAME_BYTES);
          stations[i].progressPct = 0;
        }
      }

    } else if (len >= 4) {
      // v1互換: 先頭バイトが大きい値 = uint32の一部
      v2Mode = false;
      uint32_t val = data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);
      if (val > 100000) val = 100000;
      prevProgress = currentProgress;
      currentProgress = val;
      if (len >= 8) {
        remainingDistance = data[4] | (data[5]<<8) | (data[6]<<16) | (data[7]<<24);
        distToFinalDest = remainingDistance;
      }
    }
  }
};

// --- 描画関数 ---
void drawWaiting() {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("せつぞく", M5.Lcd.width()/2, M5.Lcd.height()/2 - 20);
  M5.Lcd.drawString("まち...", M5.Lcd.width()/2, M5.Lcd.height()/2 + 20);
  drawBattery();
}

void drawBattery() {
  int batt = M5.Power.getBatteryLevel();
  char buf[16];
  snprintf(buf, sizeof(buf), "BAT:%d%%", batt);
  uint16_t col = batt > 50 ? TFT_GREEN : batt > 20 ? TFT_YELLOW : TFT_RED;
  M5.Lcd.setFont(&fonts::Font2);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(BR_DATUM);
  M5.Lcd.setTextColor(col, TFT_BLACK);
  M5.Lcd.drawString(buf, M5.Lcd.width()-5, M5.Lcd.height()-5);
}

void drawProgress(uint32_t progress) {
  M5.Lcd.fillScreen(TFT_BLACK);
  int W = M5.Lcd.width(), H = M5.Lcd.height();

  if (v2Mode && totalStations > 0) {
    // === v2: 1画面統合レイアウト ===
    // 上段: X.XXX% (大きく)
    float pct = progress / 1000.0;
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f%%", pct);
    M5.Lcd.setFont(&fonts::Font4);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.drawString(buf, W/2, 30);

    // 中段: 次は XX駅
    M5.Lcd.setFont(&fonts::lgfxJapanGothicP_24);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(0x8410, TFT_BLACK);  // グレー
    M5.Lcd.drawString("次は", W/2 - 55, H/2 + 22);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (currentNextStation < stationCount) {
      M5.Lcd.drawString(stations[currentNextStation].name, W/2 + 35, H/2 + 22);
    } else {
      M5.Lcd.drawString("---", W/2 + 35, H/2 + 22);
    }

    // 下段: 電車メーター
    drawTrainProgressBar(progress);
  } else {
    // === v1: 従来レイアウト ===
    float pct = progress / 1000.0;
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", pct);
    M5.Lcd.setFont(&fonts::Font4);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.drawString(buf, W/2, H/2 - 15);
    M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
    M5.Lcd.setTextSize(1);
    M5.Lcd.drawString("%", W/2, H/2 + 35);

    int barY = H - 20;
    int barW = W - 20;
    int filled = (int)(barW * progress / 100000.0);
    M5.Lcd.drawRect(9, barY-1, barW+2, 12, TFT_WHITE);
    if (filled > 0) M5.Lcd.fillRect(10, barY, filled, 10, TFT_CYAN);
  }
}

void drawTrainProgressBar(uint32_t progress) {
  int trackY = M5.Lcd.height() - 12;
  int trackL = 10, trackR = M5.Lcd.width() - 10;
  int trackW = trackR - trackL;

  // 枕木
  for (int x = trackL; x <= trackR; x += 7) {
    M5.Lcd.drawLine(x, trackY-3, x, trackY+3, 0x4208); // dark gray
  }
  // レール
  M5.Lcd.drawLine(trackL, trackY-2, trackR, trackY-2, 0x8410);
  M5.Lcd.drawLine(trackL, trackY+2, trackR, trackY+2, 0x8410);

  // 駅マーカー（実際の位置に配置）
  for (int i = 0; i < totalStations; i++) {
    int x;
    if (stations[i].progressPct > 0) {
      // v2.htmlから送られた実際の位置を使用
      x = trackL + (trackW * stations[i].progressPct) / 100;
    } else {
      // フォールバック: 等間隔
      x = trackL + (trackW * (i+1)) / (totalStations+1);
    }
    bool isLast = (i == totalStations - 1);
    if (i < currentNextStation) {
      M5.Lcd.fillCircle(x, trackY, 3, TFT_GREEN);
    } else if (isLast) {
      M5.Lcd.fillCircle(x, trackY, 5, TFT_RED);
    } else {
      M5.Lcd.drawCircle(x, trackY, 3, TFT_WHITE);
    }
  }

  // 電車アイコン（大きめ）
  int trainX = trackL + (trackW * progress) / 100000;
  M5.Lcd.fillRect(trainX-8, trackY-12, 16, 9, TFT_RED);
  M5.Lcd.fillRect(trainX-6, trackY-11, 4, 4, TFT_YELLOW);
  M5.Lcd.fillRect(trainX+2, trackY-11, 4, 4, TFT_YELLOW);
  M5.Lcd.drawLine(trainX-8, trackY-4, trainX+7, trackY-4, TFT_WHITE);
}

void drawNextStation() {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);

  // 「つぎは」
  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.drawString("次は", M5.Lcd.width()/2, 20);

  // 駅名
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  if (currentNextStation < stationCount) {
    M5.Lcd.drawString(stations[currentNextStation].name, M5.Lcd.width()/2, M5.Lcd.height()/2 + 5);
  } else {
    M5.Lcd.drawString("---", M5.Lcd.width()/2, M5.Lcd.height()/2 + 5);
  }

  // 残り距離
  char buf[32];
  if (distToNextStation >= 1000) {
    snprintf(buf, sizeof(buf), "のこり %.1fkm", distToNextStation/1000.0);
  } else {
    snprintf(buf, sizeof(buf), "のこり %lum", (unsigned long)distToNextStation);
  }
  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.drawString(buf, M5.Lcd.width()/2, M5.Lcd.height() - 20);
}

void drawDistance() {
  // v1互換: 残り距離表示
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.drawString("のこり", M5.Lcd.width()/2, 25);

  M5.Lcd.setFont(&fonts::Font4);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  char buf[32];
  if (remainingDistance >= 1000) {
    M5.Lcd.setTextSize(2);
    snprintf(buf, sizeof(buf), "%.1f", remainingDistance/1000.0);
    M5.Lcd.drawString(buf, M5.Lcd.width()/2, M5.Lcd.height()/2 + 5);
    M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
    M5.Lcd.setTextSize(1);
    M5.Lcd.drawString("km", M5.Lcd.width()/2, M5.Lcd.height()-20);
  } else {
    M5.Lcd.setTextSize(2);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)remainingDistance);
    M5.Lcd.drawString(buf, M5.Lcd.width()/2, M5.Lcd.height()/2 + 5);
    M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
    M5.Lcd.setTextSize(1);
    M5.Lcd.drawString("m", M5.Lcd.width()/2, M5.Lcd.height()-20);
  }
}

void drawStationArrived() {
  M5.Lcd.fillScreen(0x0018); // 濃紺
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_YELLOW, 0x0018);
  M5.Lcd.drawString("とうちゃく!", M5.Lcd.width()/2, 30);

  M5.Lcd.setTextColor(TFT_WHITE, 0x0018);
  if (prevNextStation < stationCount) {
    M5.Lcd.drawString(stations[prevNextStation].name, M5.Lcd.width()/2, M5.Lcd.height()/2 + 10);
  }
}

void drawGoal() {
  if (goalBlinkOn) {
    M5.Lcd.fillScreen(TFT_YELLOW);
    M5.Lcd.setTextColor(TFT_RED, TFT_YELLOW);
  } else {
    M5.Lcd.fillScreen(TFT_RED);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_RED);
  }
  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("到着", M5.Lcd.width()/2, M5.Lcd.height()/2 - 20);
  M5.Lcd.drawString("いってらっしゃい", M5.Lcd.width()/2, M5.Lcd.height()/2 + 20);
}

// --- 音楽再生 ---
void playMelody(const int melody[][2], int len) {
  for (int i = 0; i < len; i++) {
    if (melody[i][0] > 0) {
      M5.Speaker.tone(melody[i][0], melody[i][1]);
    }
    delay(melody[i][1] + 25);
  }
  M5.Speaker.stop();
}

void playStationArrivalMelody(uint8_t stationIdx) {
  // チャイム → 愛の挨拶 終結部
  playMelody(stationChime, stationChimeLen);
  delay(200);
  playMelody(salutDamourEnding, salutEndLen);
}

void playGoalMelody() {
  playMelody(goalMelody, goalLen);
}

void playMilestoneMelody() {
  playMelody(milestoneMelody, milestoneLen);
}

// --- BLE初期化 ---
void setupBLE() {
  BLEDevice::init("OsampoMeter");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic* pChar = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pChar->setCallbacks(new ProgressCallbacks());
  pChar->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  pAdv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

// --- Arduino ---
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Lcd.setRotation(1);
  M5.Lcd.setBrightness(80);
  M5.Speaker.begin();
  M5.Speaker.setVolume(128);
  setupBLE();
  drawWaiting();
}

void loop() {
  M5.update();

  // 接続状態の変化
  if (deviceConnected && !prevConnected) {
    prevConnected = true;
    currentProgress = 0; prevProgress = 0;
    lastMilestone = -1;
    currentNextStation = 0; prevNextStation = 0;
    displayMode = DISPLAY_PROGRESS;
    showDistance = false;
    v2Mode = false;
    currentState = STATE_CONNECTED;
    drawProgress(0);
  }

  if (!deviceConnected && prevConnected) {
    prevConnected = false;
    currentState = STATE_WAITING;
    currentProgress = 0; lastMilestone = -1;
    stationCount = 0; v2Mode = false;
    drawWaiting();
    delay(500);
    BLEDevice::startAdvertising();
  }

  if (!deviceConnected) {
    unsigned long now = millis();
    if (now - lastBatteryUpdate > 30000) {
      lastBatteryUpdate = now;
      drawWaiting();
    }
    delay(100);
    return;
  }

  // ボタンA: v1のみ表示モード切替（v2は1画面統合なので不要）
  if (M5.BtnA.wasPressed() && currentState == STATE_PROGRESS && !v2Mode) {
    showDistance = !showDistance;
    if (showDistance) drawDistance();
    else drawProgress(currentProgress);
  }

  // 駅到着演出
  if (currentState == STATE_STATION_ARRIVED) {
    if (millis() - stationArrivedTime > 3000) {
      currentState = STATE_PROGRESS;
      drawProgress(currentProgress);
    }
    delay(50);
    return;
  }

  // ゴール演出
  if (currentState == STATE_GOAL) {
    unsigned long now = millis();
    if (now - lastBlinkTime > 400) {
      goalBlinkOn = !goalBlinkOn;
      drawGoal();
      lastBlinkTime = now;
    }
    if (now - goalStartTime > 5000) {
      currentState = STATE_CONNECTED;
      drawProgress(currentProgress);
    }
    delay(50);
    return;
  }

  // 進捗更新
  if (currentProgress != prevProgress || currentState == STATE_CONNECTED) {

    // v2: 駅通過検知（ゴール時は駅到着をスキップしてゴール演出へ）
    if (v2Mode && currentNextStation > prevNextStation && prevNextStation < stationCount) {
      if (currentProgress < 100000) {
        // 中間駅に到着
        currentState = STATE_STATION_ARRIVED;
        stationArrivedTime = millis();
        drawStationArrived();
        playStationArrivalMelody(prevNextStation);
        prevNextStation = currentNextStation;
        prevProgress = currentProgress;
        return;
      }
      // ゴール時はprevNextStationだけ更新して、下のゴール処理へ進む
      prevNextStation = currentNextStation;
    }

    // 画面更新
    bool skipDraw = false;
    if (!v2Mode && showDistance) skipDraw = true;
    if (!skipDraw) drawProgress(currentProgress);

    currentState = STATE_PROGRESS;

    // マイルストーン / ゴールチェック
    int curMile = currentProgress / 10000;
    if (curMile > lastMilestone && lastMilestone >= 0) {
      if (currentProgress >= 100000) {
        currentState = STATE_GOAL;
        goalStartTime = millis();
        lastBlinkTime = millis();
        goalBlinkOn = true;
        drawGoal();
        playGoalMelody();
      } else if (!v2Mode) {
        // v1: 10%ごとにメロディ
        playMilestoneMelody();
      }
    }
    lastMilestone = curMile;
    prevProgress = currentProgress;
  }

  delay(50);
}
