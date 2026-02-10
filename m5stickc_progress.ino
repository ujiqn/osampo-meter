/*
 * おさんぽメーター - M5StickC Plus2
 * BLEで進捗(0〜1000 = 0.0%〜100.0%)を受信し、画面に大きく表示する
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

uint16_t currentProgress = 0;    // 0〜1000 (0.0%〜100.0%)
uint16_t prevProgress = 0;
int lastMilestone = -1;          // 最後に音を鳴らした10%刻みの値
unsigned long goalStartTime = 0;
unsigned long lastBlinkTime = 0;
bool goalBlinkOn = true;

// ゴール演出用メロディ (周波数, 長さms)
const int melodyNotes[][2] = {
  {523, 150}, {659, 150}, {784, 150}, {1047, 300},
  {784, 150}, {1047, 400}
};
const int melodyLength = 6;

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
    if (len >= 2) {
      // uint16_t little-endian: 0〜1000
      uint16_t val = data[0] | (data[1] << 8);
      if (val > 1000) val = 1000;
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
}

void drawProgress(uint16_t progress) {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);

  // メイン数字: XX.X%
  float pct = progress / 10.0;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", pct);

  // 大きな数字
  M5.Lcd.setFont(&fonts::Font7);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString(buf, M5.Lcd.width() / 2, M5.Lcd.height() / 2 - 10);

  // %記号
  M5.Lcd.setFont(&fonts::lgfxJapanGothicP_28);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("%", M5.Lcd.width() / 2, M5.Lcd.height() / 2 + 40);

  // プログレスバー
  int barY = M5.Lcd.height() - 20;
  int barW = M5.Lcd.width() - 20;
  int filled = (int)(barW * progress / 1000.0);
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
void playMilestoneBeep() {
  M5.Speaker.tone(880, 100);
  delay(120);
  M5.Speaker.stop();
}

void playGoalMelody() {
  for (int i = 0; i < melodyLength; i++) {
    M5.Speaker.tone(melodyNotes[i][0], melodyNotes[i][1]);
    delay(melodyNotes[i][1] + 30);
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

    // 10%刻みのマイルストーンチェック (100 = 10.0%)
    int currentMilestone = currentProgress / 100;
    if (currentMilestone > lastMilestone && lastMilestone >= 0) {
      if (currentProgress >= 1000) {
        // ゴール！
        currentState = STATE_GOAL;
        goalStartTime = millis();
        lastBlinkTime = millis();
        goalBlinkOn = true;
        drawGoal();
        playGoalMelody();
      } else {
        // 10%マイルストーン
        playMilestoneBeep();
      }
    }
    lastMilestone = currentMilestone;
    prevProgress = currentProgress;
  }

  delay(50);
}
