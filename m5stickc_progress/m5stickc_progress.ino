/*
 * おさんぽメーター - M5StickC Plus2
 * BLEで進捗(0〜100000 = 0.000%〜100.000%)を受信し、画面に大きく表示する
 * 10%ごとにブザー音、100%でゴール演出
 */

#include <M5StickCPlus2.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-123456789def"

// 状態管理
enum State {
  STATE_WAITING,    // BLE接続待ち
  STATE_CONNECTED,  // 接続済み・待機
  STATE_PROGRESS,   // 進捗表示中
  STATE_GOAL        // ゴール演出中
};

State currentState = STATE_WAITING;
bool deviceConnected = false;
bool prevConnected = false;

uint32_t currentProgress = 0;    // 0〜100000 (0.000%〜100.000%)
uint32_t prevProgress = 0;
int lastMilestone = -1;          // 最後に音を鳴らした10%刻みの値
unsigned long goalStartTime = 0;
unsigned long lastBlinkTime = 0;
bool goalBlinkOn = true;
unsigned long lastBatteryUpdate = 0;  // バッテリー表示更新タイマー

// マイルストーン演出用メロディ (周波数, 長さms) - 短い3音ファンファーレ
const int milestoneMelody[][2] = {
  {784, 100}, {988, 100}, {1319, 200}   // G5→B5→E6 (明るい上昇音)
};
const int milestoneLength = 3;

// ゴール演出用メロディ (周波数, 長さms) - 豪華な6音ファンファーレ
const int goalMelody[][2] = {
  {523, 150}, {659, 150}, {784, 150}, {1047, 300},
  {784, 150}, {1047, 400}
};
const int goalLength = 6;

// --- BLEコールバック ---
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class ProgressCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    uint8_t* data = pCharacteristic->getData();
    size_t len = pCharacteristic->getLength();
    if (len >= 4) {
      // uint32_t little-endian: 0〜100000
      uint32_t val = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
      if (val > 100000) val = 100000;
      prevProgress = currentProgress;
      currentProgress = val;
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
  M5.Lcd.drawString("せつぞく", M5.Lcd.width() / 2, M5.Lcd.height() / 2 - 20);
  M5.Lcd.drawString("まち...", M5.Lcd.width() / 2, M5.Lcd.height() / 2 + 20);

  // バッテリー残量表示
  drawBattery();
}

void drawBattery() {
  int battLevel = M5.Power.getBatteryLevel();  // 0〜100
  char batBuf[16];
  snprintf(batBuf, sizeof(batBuf), "BAT:%d%%", battLevel);

  // バッテリーアイコンの色（残量に応じて変化）
  uint16_t batColor;
  if (battLevel > 50) {
    batColor = TFT_GREEN;
  } else if (battLevel > 20) {
    batColor = TFT_YELLOW;
  } else {
    batColor = TFT_RED;
  }

  M5.Lcd.setFont(&fonts::Font2);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(BR_DATUM);  // 右下基準
  M5.Lcd.setTextColor(batColor, TFT_BLACK);
  M5.Lcd.drawString(batBuf, M5.Lcd.width() - 5, M5.Lcd.height() - 5);
}

void drawProgress(uint32_t progress) {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);

  // メイン数字: XX.XXX%
  float pct = progress / 1000.0;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.3f", pct);

  // 大きな数字
  M5.Lcd.setFont(&fonts::Font4);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString(buf, M5.Lcd.width() / 2, M5.Lcd.height() / 2 - 10);

  // %記号
  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("%", M5.Lcd.width() / 2, M5.Lcd.height() / 2 + 40);

  // プログレスバー
  int barY = M5.Lcd.height() - 20;
  int barW = M5.Lcd.width() - 20;
  int filled = (int)(barW * progress / 100000.0);
  M5.Lcd.drawRect(9, barY - 1, barW + 2, 12, TFT_WHITE);
  if (filled > 0) {
    M5.Lcd.fillRect(10, barY, filled, 10, TFT_CYAN);
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

  M5.Lcd.drawString("★ クリア！★", M5.Lcd.width() / 2, M5.Lcd.height() / 2 - 30);

  M5.Lcd.setFont(&fonts::Font7);
  M5.Lcd.drawString("100", M5.Lcd.width() / 2, M5.Lcd.height() / 2 + 20);

  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.drawString("%", M5.Lcd.width() / 2, M5.Lcd.height() / 2 + 55);
}

// --- 音・振動 ---
void playMilestoneMelody() {
  for (int i = 0; i < milestoneLength; i++) {
    M5.Speaker.tone(milestoneMelody[i][0], milestoneMelody[i][1]);
    delay(milestoneMelody[i][1] + 30);
  }
  M5.Speaker.stop();
}

void playGoalMelody() {
  for (int i = 0; i < goalLength; i++) {
    M5.Speaker.tone(goalMelody[i][0], goalMelody[i][1]);
    delay(goalMelody[i][1] + 30);
  }
  M5.Speaker.stop();
}

// --- BLE初期化 ---
void setupBLE() {
  BLEDevice::init("OsampoMeter");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic* pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCharacteristic->setCallbacks(new ProgressCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

// --- Arduino ---
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Lcd.setRotation(1);  // 横向き
  M5.Lcd.setBrightness(80);

  M5.Speaker.begin();
  M5.Speaker.setVolume(128);

  setupBLE();
  drawWaiting();
}

void loop() {
  M5.update();

  // 接続状態の変化を検知
  if (deviceConnected && !prevConnected) {
    // 新規接続
    prevConnected = true;
    currentProgress = 0;
    prevProgress = 0;
    lastMilestone = -1;
    currentState = STATE_CONNECTED;
    drawProgress(0);
  }

  if (!deviceConnected && prevConnected) {
    // 切断
    prevConnected = false;
    currentState = STATE_WAITING;
    currentProgress = 0;
    lastMilestone = -1;
    drawWaiting();
    // 再アドバタイズ
    delay(500);
    BLEDevice::startAdvertising();
  }

  if (!deviceConnected) {
    // 待機中は30秒ごとにバッテリー表示を更新
    unsigned long now = millis();
    if (now - lastBatteryUpdate > 30000) {
      lastBatteryUpdate = now;
      drawWaiting();  // バッテリー残量込みで再描画
    }
    delay(100);
    return;
  }

  // ゴール演出中
  if (currentState == STATE_GOAL) {
    unsigned long now = millis();
    // 点滅
    if (now - lastBlinkTime > 400) {
      goalBlinkOn = !goalBlinkOn;
      drawGoal();
      lastBlinkTime = now;
    }
    // 5秒でゴール演出終了 → 待機
    if (now - goalStartTime > 5000) {
      currentState = STATE_CONNECTED;
      drawProgress(currentProgress);
    }
    delay(50);
    return;
  }

  // 進捗更新チェック
  if (currentProgress != prevProgress || currentState == STATE_CONNECTED) {
    // 画面更新
    drawProgress(currentProgress);
    currentState = STATE_PROGRESS;

    // 10%刻みのマイルストーンチェック (10000 = 10.000%)
    int currentMilestone = currentProgress / 10000;
    if (currentMilestone > lastMilestone && lastMilestone >= 0) {
      if (currentProgress >= 100000) {
        // ゴール！
        currentState = STATE_GOAL;
        goalStartTime = millis();
        lastBlinkTime = millis();
        goalBlinkOn = true;
        drawGoal();
        playGoalMelody();
      } else {
        // 10%マイルストーン
        playMilestoneMelody();
      }
    }
    lastMilestone = currentMilestone;
    prevProgress = currentProgress;
  }

  delay(50);
}
