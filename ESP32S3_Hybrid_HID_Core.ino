/**
 * Bo mạch chọn: ESP32-S3 Dev Module
 * Cấu hình bắt buộc: Tools > USB Mode > USB-OTG (TinyUSB)
 * Thư viện yêu cầu cài đặt: NimBLEDevice
 *
 * ============================================================
 * GIAO THỨC BYTE (Android ↔ ESP32)
 * ============================================================
 * Android → ESP32:
 *   0xF1 + role     : Handshake định danh (0x01=WiFi, 0x02=BLE)
 *   0xF3 + len + pw : Mật khẩu thiết bị
 *   0xF6 + wifi     : Nạp cấu hình Wi-Fi mới
 *   0xF7            : Ping (kiểm tra kết nối)
 *   0xFE            : Yêu cầu chuyển sang Wi-Fi
 *   0xFF            : Yêu cầu chuyển sang BLE
 *
 * ESP32 → Android:
 *   0xF2            : Yêu cầu mật khẩu thiết bị
 *   0xF4            : Xác thực/lưu thành công
 *   0xF5            : Sai mật khẩu
 *   0xF8            : Pong (phản hồi ping)
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <NimBLEDevice.h>

// ---- Cấu phần USB HID phần cứng lõi ----
#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBHIDMouse.h"
#include "USBHIDConsumerControl.h"

// ==========================================
// 1. CẤU HÌNH HỆ THỐNG & TRẠNG THÁI HYBRID
// ==========================================
enum DeviceMode { MODE_BLE = 0, MODE_WIFI = 1 };
DeviceMode currentMode = MODE_BLE;
Preferences preferences;

// Chuỗi chứa thông tin Wi-Fi động đọc từ bộ nhớ Flash
String wifi_ssid_dynamic = "";
String wifi_pass_dynamic = "";

// Cổng lắng nghe TCP (Đón TcpScanner từ Android)
const uint16_t TCP_PORT = 12345;
WiFiServer tcpServer(TCP_PORT);
WiFiClient activeTcpClient;

// Mật khẩu thiết bị (Ví dụ: "123456")
const String SYSTEM_PASSWORD_PLAIN = "123456";

struct ConnectionState {
  volatile bool isConnected = false;
  volatile bool isIdentified = false;
  volatile bool isVerified = false;
  uint8_t role = 0;
};
// Dùng volatile để đảm bảo đồng bộ giữa 2 Core của ESP32
volatile ConnectionState activeConnection;

// ==========================================
// 1b. CỜ CHUYỂN MODE AN TOÀN GIỮA 2 CORE
// ==========================================
// ParserTask (Core 1) chỉ set cờ, KHÔNG gọi switchMode trực tiếp
// loop() (Core 0) đọc cờ và thực thi switchMode
volatile bool       gPendingSwitch       = false;
volatile DeviceMode gPendingMode         = MODE_BLE;
volatile bool       gPendingWifiResponse = false;
volatile bool       gPendingWifiIsBT     = false;

// ==========================================
// 1c. CỜ QUẢN LÝ SOCKET (CHỐNG SPAM RECONNECT)
// ==========================================
static unsigned long lastClientChangeTime = 0;
const unsigned long CLIENT_CHANGE_COOLDOWN_MS = 500;

// ==========================================
// 2. BỘ ĐỆM HÀNG ĐỢI FREE RTOS QUEUE
// ==========================================
struct Packet {
  uint8_t data[64];
  size_t length;
};
QueueHandle_t packetQueue = nullptr;
TaskHandle_t parserTaskHandle = NULL;

// ==========================================
// 3. BẢN QUY ƯỚC MÃ TRẠNG THÁI HỆ THỐNG (STATUS CODES)
// ==========================================
const uint8_t CMD_IDENTIFY_ROLE   = 0xF1;
const uint8_t REP_REQUIRE_AUTH    = 0xF2;
const uint8_t CMD_AUTH_PASSWORD   = 0xF3;
const uint8_t REP_AUTH_SUCCESS    = 0xF4;
const uint8_t REP_AUTH_FAILED     = 0xF5;
const uint8_t CMD_UPDATE_WIFI     = 0xF6;
const uint8_t CMD_PING            = 0xF7;
const uint8_t REP_PONG            = 0xF8;
const uint8_t CMD_SWITCH_TO_WIFI  = 0xFE;
const uint8_t CMD_SWITCH_TO_BLE   = 0xFF;

// ---- v1: Giao thức 3 bytes cũ ----
static const uint8_t CMD_KEY          = 0x01;
static const uint8_t CMD_MOUSE_MOVE   = 0x02;
static const uint8_t CMD_MOUSE_CLICK  = 0x03;
static const uint8_t CMD_MOUSE_SCROLL = 0x04;

// ---- v2: Giao thức TLV động [0xAA, 0x01] ----
static const uint8_t V2_MAGIC         = 0xAA;
static const uint8_t V2_VERSION       = 0x01;
static const uint8_t V2_SET_MODIFIERS = 0x01;
static const uint8_t V2_KEY_DOWN      = 0x02;
static const uint8_t V2_KEY_UP        = 0x03;
static const uint8_t V2_KEY_TAP       = 0x04;
static const uint8_t V2_MOUSE_MOVE    = 0x10;
static const uint8_t V2_MOUSE_SCROLL  = 0x11;
static const uint8_t V2_MOUSE_CLICK   = 0x12;
static const uint8_t V2_MOUSE_DOWN    = 0x13;
static const uint8_t V2_MOUSE_UP      = 0x14;

// Consumer Control qua V2 TLV
static const uint8_t V2_CONSUMER_TAP  = 0x20;
static const uint8_t V2_CONSUMER_DOWN = 0x21;
static const uint8_t V2_CONSUMER_UP   = 0x22;

// ==========================================
// 4. CẤU HÌNH UUID BLUETOOTH BLE VÀ NGOẠI VI
// ==========================================
static const char* kServiceUUID    = "2D2A0001-8A5A-4E76-A2E3-1E57D9A1B001";
static const char* kWriteCharUUID  = "2D2A0002-8A5A-4E76-A2E3-1E57D9A1B001";
static const char* kNotifyCharUUID = "2D2A0003-8A5A-4E76-A2E3-1E57D9A1B001";

NimBLECharacteristic* pNotifyChar = nullptr;
NimBLEServer* pServer = nullptr;

USBHIDKeyboard Keyboard;
USBHIDMouse Mouse;
USBHIDConsumerControl ConsumerControl;

static uint8_t gModifiersMask = 0x00;
static bool gKeysDown[256] = { false };

// Forward declarations
void switchMode(DeviceMode newMode);
void sendSystemByteResponse(uint8_t statusByte, bool isBluetooth);
void clearAllHardwareStates();
void setupWifiSTA();
void setupBle();

// ==========================================
// 5. BẢN ĐỒ MÃ HID CHUẨN
// ==========================================
const uint8_t MOD_NONE         = 0x00;
const uint8_t MOD_LEFT_CTRL    = 0x01;
const uint8_t MOD_LEFT_SHIFT   = 0x02;
const uint8_t MOD_LEFT_ALT     = 0x04;
const uint8_t MOD_LEFT_GUI     = 0x08;
const uint8_t MOD_RIGHT_CTRL   = 0x10;
const uint8_t MOD_RIGHT_SHIFT  = 0x20;
const uint8_t MOD_RIGHT_ALT    = 0x40;
const uint8_t MOD_RIGHT_GUI    = 0x80;

const uint8_t MOUSE_BUTTON_LEFT    = 0x01;
const uint8_t MOUSE_BUTTON_RIGHT   = 0x02;
const uint8_t MOUSE_BUTTON_MIDDLE  = 0x04;

const uint8_t KEY_NONE = 0x00;
const uint8_t KEY_A = 0x04; const uint8_t KEY_B = 0x05; const uint8_t KEY_C = 0x06;
const uint8_t KEY_D = 0x07; const uint8_t KEY_E = 0x08; const uint8_t KEY_F = 0x09;
const uint8_t KEY_G = 0x0A; const uint8_t KEY_H = 0x0B; const uint8_t KEY_I = 0x0C;
const uint8_t KEY_J = 0x0D; const uint8_t KEY_K = 0x0E; const uint8_t KEY_L = 0x0F;
const uint8_t KEY_M = 0x10; const uint8_t KEY_N = 0x11; const uint8_t KEY_O = 0x12;
const uint8_t KEY_P = 0x13; const uint8_t KEY_Q = 0x14; const uint8_t KEY_R = 0x15;
const uint8_t KEY_S = 0x16; const uint8_t KEY_T = 0x17; const uint8_t KEY_U = 0x18;
const uint8_t KEY_V = 0x19; const uint8_t KEY_W = 0x1A; const uint8_t KEY_X = 0x1B;
const uint8_t KEY_Y = 0x1C; const uint8_t KEY_Z = 0x1D;

const uint8_t KEY_1 = 0x1E; const uint8_t KEY_2 = 0x1F; const uint8_t KEY_3 = 0x20;
const uint8_t KEY_4 = 0x21; const uint8_t KEY_5 = 0x22; const uint8_t KEY_6 = 0x23;
const uint8_t KEY_7 = 0x24; const uint8_t KEY_8 = 0x25; const uint8_t KEY_9 = 0x26;
const uint8_t KEY_0 = 0x27;

const uint8_t KEY_ENTER = 0x28;      const uint8_t KEY_ESCAPE = 0x29;
const uint8_t KEY_BACKSPACE = 0x2A;  const uint8_t KEY_TAB = 0x2B;
const uint8_t KEY_SPACE = 0x2C;      const uint8_t KEY_MINUS = 0x2D;
const uint8_t KEY_EQUAL = 0x2E;      const uint8_t KEY_LEFT_BRACKET = 0x2F;
const uint8_t KEY_RIGHT_BRACKET = 0x30; const uint8_t KEY_BACKSLASH = 0x31;
const uint8_t KEY_SEMICOLON = 0x33;   const uint8_t KEY_QUOTE = 0x34;
const uint8_t KEY_GRAVE = 0x35;       const uint8_t KEY_COMMA = 0x36;
const uint8_t KEY_PERIOD = 0x37;      const uint8_t KEY_SLASH = 0x38;

const uint8_t KEY_F1 = 0x3A; const uint8_t KEY_F2 = 0x3B; const uint8_t KEY_F3 = 0x3C;
const uint8_t KEY_F4 = 0x3D; const uint8_t KEY_F5 = 0x3E; const uint8_t KEY_F6 = 0x3F;
const uint8_t KEY_F7 = 0x40; const uint8_t KEY_F8 = 0x41; const uint8_t KEY_F9 = 0x42;
const uint8_t KEY_F10 = 0x43; const uint8_t KEY_F11 = 0x44; const uint8_t KEY_F12 = 0x45;

// ==========================================
// 6. TẦNG DRIVER HID (ĐỒNG BỘ 100% VỚI DỰ ÁN 1)
// ==========================================

static void setModifiers(uint8_t newMask) {
  uint8_t diff = gModifiersMask ^ newMask;
  if (!diff) return;
  for (uint8_t i = 0; i < 8; i++) {
    if (diff & (1 << i)) {
      if (newMask & (1 << i)) Keyboard.pressRaw(0xE0 + i);
      else Keyboard.releaseRaw(0xE0 + i);
    }
  }
  gModifiersMask = newMask;
}

static void keyDown(uint8_t keycode) {
  if (keycode == 0x00 || gKeysDown[keycode]) return;
  Keyboard.pressRaw(keycode);
  gKeysDown[keycode] = true;
}

static void keyUp(uint8_t keycode) {
  if (keycode == 0x00 || !gKeysDown[keycode]) return;
  Keyboard.releaseRaw(keycode);
  gKeysDown[keycode] = false;
}

// KeyTap thuần túy (port nguyên từ dự án 1)
static void keyTap(uint8_t modifiersMask, uint8_t keycode) {
  if (keycode == 0x00) return;

  bool wasDown = gKeysDown[keycode];
  uint8_t savedMods = gModifiersMask;

  setModifiers(modifiersMask);

  if (!wasDown) {
    keyDown(keycode);
    delay(5);
    keyUp(keycode);
    delay(1);
  }

  setModifiers(savedMods);
}

// Wrapper chuột (port nguyên từ dự án 1)
static void sendMouseMove(int8_t dx, int8_t dy) {
  Mouse.move(dx, dy);
}

static void sendMouseClick(uint8_t button) {
  Mouse.click(button);
}

static void sendMouseButtonDown(uint8_t button) {
  Mouse.press(button);
}

static void sendMouseButtonUp(uint8_t button) {
  Mouse.release(button);
}

static void sendMouseScroll(int8_t dx, int8_t dy) {
  // Scroll: positive Y = scroll up, negative Y = scroll down
  Mouse.move(0, 0, dy, dx);
}

// ==========================================
// 6b. BỘ ĐỊNH TUYẾN PHÍM THÔNG MINH (SMART KEY TAP)
// ==========================================
static void handleSmartKeyTap(uint8_t modifiers, uint8_t keycode) {
  if (keycode == 0x00) return;

  // Lọc và ánh xạ dải phím Consumer Control Page 0x0C từ App Android
  switch (keycode) {
    case 0x30: ConsumerControl.press(SYSTEM_POWER_DOWN);   delay(10); ConsumerControl.release(); return;
    case 0x32: ConsumerControl.press(SYSTEM_SLEEP);        delay(10); ConsumerControl.release(); return;
    case 0x83: ConsumerControl.press(SYSTEM_WAKE_UP);      delay(10); ConsumerControl.release(); return;
    case 0xE2: ConsumerControl.press(AUDIO_MUTE);          delay(10); ConsumerControl.release(); return;
    case 0xE9: ConsumerControl.press(AUDIO_VOL_UP);        delay(10); ConsumerControl.release(); return;
    case 0xEA: ConsumerControl.press(AUDIO_VOL_DOWN);      delay(10); ConsumerControl.release(); return;
    case 0xCD: ConsumerControl.press(AUDIO_PLAY_PAUSE);    delay(10); ConsumerControl.release(); return;
    case 0xB5: ConsumerControl.press(AUDIO_NEXT_TRACK);    delay(10); ConsumerControl.release(); return;
    case 0xB6: ConsumerControl.press(AUDIO_PREV_TRACK);    delay(10); ConsumerControl.release(); return;
    case 0xB7: ConsumerControl.press(AUDIO_STOP);          delay(10); ConsumerControl.release(); return;
    case 0xB3: ConsumerControl.press(AUDIO_FAST_FORWARD);  delay(10); ConsumerControl.release(); return;
    case 0xB4: ConsumerControl.press(AUDIO_REWIND);        delay(10); ConsumerControl.release(); return;
    case 0x6F: ConsumerControl.press(SCREEN_BRIGHTNESS_UP);   delay(10); ConsumerControl.release(); return;
    case 0x70: ConsumerControl.press(SCREEN_BRIGHTNESS_DOWN); delay(10); ConsumerControl.release(); return;
    case 0x9E: ConsumerControl.press(MENU_PICK);           delay(10); ConsumerControl.release(); return;
    case 0x76: ConsumerControl.press(MENU_ESCAPE);         delay(10); ConsumerControl.release(); return;
  }

  // Fallback: gõ tổ hợp bàn phím ký tự chuẩn
  keyTap(modifiers, keycode);
}

// ==========================================
// 6c. XỬ LÝ CONSUMER CONTROL QUA MÃ 16-BIT (V2 TLV)
// ==========================================
static void handleConsumerAction(uint8_t cmd, uint16_t usageCode) {
  uint16_t mappedCode = 0;
  switch (usageCode) {
    case 0x0030: mappedCode = SYSTEM_POWER_DOWN;      break;
    case 0x0032: mappedCode = SYSTEM_SLEEP;           break;
    case 0x0083: mappedCode = SYSTEM_WAKE_UP;         break;
    case 0x00E2: mappedCode = AUDIO_MUTE;             break;
    case 0x00E9: mappedCode = AUDIO_VOL_UP;           break;
    case 0x00EA: mappedCode = AUDIO_VOL_DOWN;         break;
    case 0x00CD: mappedCode = AUDIO_PLAY_PAUSE;       break;
    case 0x00B5: mappedCode = AUDIO_NEXT_TRACK;       break;
    case 0x00B6: mappedCode = AUDIO_PREV_TRACK;       break;
    case 0x00B7: mappedCode = AUDIO_STOP;             break;
    case 0x00B3: mappedCode = AUDIO_FAST_FORWARD;     break;
    case 0x00B4: mappedCode = AUDIO_REWIND;           break;
    case 0x006F: mappedCode = SCREEN_BRIGHTNESS_UP;   break;
    case 0x0070: mappedCode = SCREEN_BRIGHTNESS_DOWN; break;
    case 0x009E: mappedCode = MENU_PICK;              break;
    case 0x0076: mappedCode = MENU_ESCAPE;            break;
    default: return;
  }

  switch (cmd) {
    case V2_CONSUMER_TAP:
      ConsumerControl.press(mappedCode);
      delay(5);
      ConsumerControl.release();
      break;
    case V2_CONSUMER_DOWN:
      ConsumerControl.press(mappedCode);
      break;
    case V2_CONSUMER_UP:
      ConsumerControl.release();
      break;
    default:
      break;
  }
}

// ==========================================
// 7. DỌN SẠCH TÀI NGUYÊN PHẦN CỨNG
// ==========================================
void clearAllHardwareStates() {
  Serial.println("[CLEANER] Thực thi dọn sạch tài nguyên bộ đệm...");
  Keyboard.releaseAll();
  Mouse.releaseAll();
  ConsumerControl.release();
  memset(gKeysDown, 0, sizeof(gKeysDown));
  gModifiersMask = 0x00;

  if (packetQueue != nullptr) {
    xQueueReset(packetQueue);
  }

  activeConnection.isConnected  = false;
  activeConnection.isIdentified = false;
  activeConnection.isVerified   = false;
  activeConnection.role         = 0;
}

bool verifyPlainPassword(const uint8_t* inputBytes, size_t length) {
  if (length != SYSTEM_PASSWORD_PLAIN.length()) return false;
  for (size_t i = 0; i < length; i++) {
    if (inputBytes[i] != SYSTEM_PASSWORD_PLAIN[i]) return false;
  }
  return true;
}

// ==========================================
// 8. TRÌNH PARSER TRỘN ĐỒNG BỘ ĐẦY ĐỦ CHỨC NĂNG
//    (Chỉ set cờ — KHÔNG gọi switchMode trực tiếp)
// ==========================================
void parseHidCommand(const uint8_t* data, size_t length, bool isBluetooth) {
  if (length == 0 || data == nullptr) return;

  // ============================================================
  // XỬ LÝ PING — Phản hồi PONG ngay lập tức
  // Đặt TRƯỚC handshake để ping hoạt động mọi lúc (kể cả chưa identify)
  // ============================================================
  if (data[0] == CMD_PING) {
    sendSystemByteResponse(REP_PONG, isBluetooth);
    Serial.println("[PING] Đã nhận CMD_PING, phản hồi REP_PONG");
    return;
  }

  // ============================================================
  // LỆNH HỆ THỐNG CƯỠNG BÁCH ĐỔI CHẾ ĐỘ PHẦN CỨNG TỪ XA
  // Chỉ set cờ, loop() sẽ thực thi switchMode trên Core 0
  // ============================================================
  if (data[0] == CMD_SWITCH_TO_WIFI) {
    gPendingMode = MODE_WIFI;
    gPendingSwitch = true;
    Serial.println("[PARSER] Yêu cầu chuyển sang WIFI (chờ loop xử lý)");
    return;
  }
  if (data[0] == CMD_SWITCH_TO_BLE) {
    gPendingMode = MODE_BLE;
    gPendingSwitch = true;
    Serial.println("[PARSER] Yêu cầu chuyển sang BLE (chờ loop xử lý)");
    return;
  }

  // ============================================================
  // XỬ LÝ LỆNH ĐỒNG BỘ CẤU HÌNH WI-FI ĐỘNG TỪ APP GỬI XUỐNG CỨU HỘ
  // Lưu cấu hình vào Flash, set cờ để loop() phản hồi + switch
  // ============================================================
  if (data[0] == CMD_UPDATE_WIFI && length >= 2) {
    size_t ssidLen   = data[1];
    size_t ssidStart = 2;
    size_t ssidEnd   = ssidStart + ssidLen;

    // Bảo vệ chống đọc tràn buffer
    if (ssidEnd + 1 > length) {
      Serial.println("[WIFI-CFG] Lỗi: ssidLen vượt quá buffer");
      return;
    }

    String newSSID = "";
    for (size_t i = 0; i < ssidLen; i++) {
      newSSID += (char)data[ssidStart + i];
    }

    size_t passLenIdx = ssidEnd;
    size_t passLen    = data[passLenIdx];
    size_t passStart  = passLenIdx + 1;
    size_t passEnd    = passStart + passLen;

    // Bảo vệ chống đọc tràn buffer
    if (passEnd > length) {
      Serial.println("[WIFI-CFG] Lỗi: passLen vượt quá buffer");
      return;
    }

    String newPASS = "";
    for (size_t i = 0; i < passLen; i++) {
      newPASS += (char)data[passStart + i];
    }

    // Ghi vào Flash ngay (an toàn với NVS)
    preferences.begin("hybrid-sys", false);
    preferences.putString("wf_ssid", newSSID);
    preferences.putString("wf_pass", newPASS);
    preferences.end();

    Serial.printf("[FLASH] Đã nhận Wi-Fi mới: SSID='%s', PASS='***'\n", newSSID.c_str());
    Serial.println("[FLASH] Chờ loop() xử lý chuyển mode...");

    // Set cờ để loop() gửi phản hồi + switch mode
    gPendingWifiIsBT     = isBluetooth;
    gPendingWifiResponse = true;
    gPendingMode         = MODE_WIFI;
    gPendingSwitch       = true;
    return;
  }

  // ============================================================
  // GIAI ĐOẠN 1: BẮT TAY ĐỊNH DANH (ROLE HANDSHAKE)
  // ============================================================
  if (!activeConnection.isIdentified) {
    if (data[0] == CMD_IDENTIFY_ROLE && length >= 2) {
      activeConnection.role = data[1];
      activeConnection.isIdentified = true;

      if (activeConnection.role == 0x02) { // BLE -> auto pass
        activeConnection.isVerified = true;
        sendSystemByteResponse(REP_AUTH_SUCCESS, true);
        Serial.println("[BLE] Xác thực lớp vật lý thành công. READY.");
      } else { // Wi-Fi -> đòi mật khẩu
        sendSystemByteResponse(REP_REQUIRE_AUTH, false);
        Serial.println("[WIFI] Mạng mở. Yêu cầu nhập mật khẩu bảo mật.");
      }
    }
    return;
  }

  // ============================================================
  // GIAI ĐOẠN 2: AUTH SANDBOX MODE
  // ============================================================
  if (!activeConnection.isVerified) {
    if (data[0] == CMD_AUTH_PASSWORD && length >= 2) {
      size_t passLen = data[1];
      if (length >= (2 + passLen)) {
        if (verifyPlainPassword(&data[2], passLen)) {
          activeConnection.isVerified = true;
          sendSystemByteResponse(REP_AUTH_SUCCESS, isBluetooth);
          Serial.println("[AUTH] Đúng mật khẩu. Đã mở khóa luồng HID.");
        } else {
          sendSystemByteResponse(REP_AUTH_FAILED, isBluetooth);
          Serial.println("[AUTH] Sai mật khẩu. Giữ nguyên Socket chờ nhập lại.");
        }
      }
    }
    return;
  }

  // ============================================================
  // GIAI ĐOẠN 3: THỰC THI LỆNH ĐIỀU KHIỂN CHUỘT PHÍM (READY MODE)
  // ============================================================

  // ---- GIAO THỨC V2 ĐỘNG: [0xAA, 0x01] + TLV ----
  if (length >= 2 && data[0] == V2_MAGIC && data[1] == V2_VERSION) {
    size_t idx = 2;
    while (idx + 1 < length) {
      uint8_t cmd = data[idx + 0];
      uint8_t len = data[idx + 1];
      idx += 2;
      if (idx + len > length) break;
      const uint8_t* payload = &data[idx];

      switch (cmd) {
        case V2_SET_MODIFIERS:
          if (len == 1) setModifiers(payload[0]);
          break;
        case V2_KEY_DOWN:
          if (len == 1) keyDown(payload[0]);
          break;
        case V2_KEY_UP:
          if (len == 1) keyUp(payload[0]);
          break;
        case V2_KEY_TAP:
          if (len == 2) handleSmartKeyTap(payload[0], payload[1]);
          break;
        case V2_MOUSE_MOVE:
          if (len == 2) sendMouseMove((int8_t)payload[0], (int8_t)payload[1]);
          break;
        case V2_MOUSE_SCROLL:
          if (len == 2) sendMouseScroll((int8_t)payload[0], (int8_t)payload[1]);
          break;
        case V2_MOUSE_CLICK:
          if (len == 1) sendMouseClick(payload[0]);
          break;
        case V2_MOUSE_DOWN:
          if (len == 1) sendMouseButtonDown(payload[0]);
          break;
        case V2_MOUSE_UP:
          if (len == 1) sendMouseButtonUp(payload[0]);
          break;
        case V2_CONSUMER_TAP:
        case V2_CONSUMER_DOWN:
        case V2_CONSUMER_UP:
          if (len == 2) {
            uint16_t usageCode = (uint16_t)((payload[0] << 8) | payload[1]);
            handleConsumerAction(cmd, usageCode);
          }
          break;
        default:
          break;
      }
      idx += len;
    }
    return;
  }

  // ---- GIAO THỨC V1 CỐ ĐỊNH: Gói tin 3 Bytes (Hỗ trợ Batching) ----
  for (size_t i = 0; i + 2 < length; i += 3) {
    uint8_t type  = data[i + 0];
    uint8_t byte1 = data[i + 1];
    uint8_t byte2 = data[i + 2];

    switch (type) {
      case CMD_KEY:
        handleSmartKeyTap(byte1, byte2);
        break;
      case CMD_MOUSE_MOVE:
        sendMouseMove((int8_t)byte1, (int8_t)byte2);
        break;
      case CMD_MOUSE_CLICK:
        sendMouseClick(byte1);
        break;
      case CMD_MOUSE_SCROLL:
        sendMouseScroll((int8_t)byte1, (int8_t)byte2);
        break;
      default:
        break;
    }
  }
}

// ==========================================
// 9. THAO TÁC HÀNG ĐỢI AN TOÀN (NON-BLOCKING QUEUE)
// ==========================================
void pushToQueue(const uint8_t* buffer, size_t size) {
  if (packetQueue == nullptr || size == 0) return;

  Packet p;
  size_t bytesToCopy = min(size, (size_t)64);
  memcpy(p.data, buffer, bytesToCopy);
  p.length = bytesToCopy;

  if (xQueueSend(packetQueue, &p, 0) != pdTRUE) {
    Packet dummy;
    xQueueReceive(packetQueue, &dummy, 0);
    xQueueSend(packetQueue, &p, 0);
  }
}

void parserTaskWorker(void* pvParameters) {
  Packet packet;
  while (true) {
    if (xQueueReceive(packetQueue, &packet, portMAX_DELAY) == pdTRUE) {
      parseHidCommand(packet.data, packet.length, (currentMode == MODE_BLE));
    }
  }
}

// ==========================================
// 10. PHẢN HỒI TRẠNG THÁI HỆ THỐNG VỀ APP
// ==========================================
void sendSystemByteResponse(uint8_t statusByte, bool isBluetooth) {
  if (isBluetooth) {
    if (pNotifyChar != nullptr && pServer != nullptr && pServer->getConnectedCount() > 0) {
      pNotifyChar->setValue(&statusByte, 1);
      pNotifyChar->notify();
    }
  } else {
    if (activeTcpClient && activeTcpClient.connected()) {
      activeTcpClient.write(statusByte);
      activeTcpClient.flush();
    }
  }
}

// ==========================================
// 11. QUẢN LÝ ĐÓNG / MỞ SÓNG ĐỘC QUYỀN TRÊN CHIP RF
// ==========================================
void setupWifiSTA() {
  Serial.println("[DRV] Đang đọc cấu hình Wi-Fi từ bộ nhớ Flash...");

  preferences.begin("hybrid-sys", true);
  wifi_ssid_dynamic = preferences.getString("wf_ssid", "");
  wifi_pass_dynamic = preferences.getString("wf_pass", "");
  preferences.end();

  if (wifi_ssid_dynamic == "") {
    Serial.println("[WIFI] Bộ nhớ trống! Tự động chuyển sang Bluetooth chờ nạp...");
    currentMode = MODE_BLE;
    preferences.begin("hybrid-sys", false);
    preferences.putInt("mode", (int)MODE_BLE);
    preferences.end();
    setupBle();
    return;
  }

  Serial.printf("[WIFI] Tiến hành kết nối tới Router: %s\n", wifi_ssid_dynamic.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("ESP32S3-HID");
  WiFi.begin(wifi_ssid_dynamic.c_str(), wifi_pass_dynamic.c_str());

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 8000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WIFI] Kết nối Router OK! IP: ");
    Serial.println(WiFi.localIP());
    tcpServer.begin();
    tcpServer.setNoDelay(true);
  } else {
    Serial.println("\n[WIFI] LỖI MẠNG: Tự động CỨU HỘ sang Bluetooth.");
    currentMode = MODE_BLE;
    preferences.begin("hybrid-sys", false);
    preferences.putInt("mode", (int)MODE_BLE);
    preferences.end();
    setupBle();
  }
}

void disableWifi() {
  // Xả buffer và đóng client trước
  if (activeTcpClient) {
    activeTcpClient.flush();
    while (activeTcpClient.available() > 0) activeTcpClient.read();
    activeTcpClient.stop();
    activeTcpClient = WiFiClient();
  }
  tcpServer.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  delay(100);
}

// ==========================================
// 12. CALLBACK BLE
// ==========================================
class BleCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    std::string v = pChar->getValue();
    if (v.size() == 0) return;
    pushToQueue((const uint8_t*)v.data(), v.size());
  }
};

class BleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    clearAllHardwareStates();
    activeConnection.isConnected = true;
    Serial.println("[BLE] Client Connected. Bộ đệm phím chuột đã SẠCH.");
  }

  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    clearAllHardwareStates();
    Serial.printf("[BLE] Client Disconnected, reason=%d\n", reason);
    NimBLEDevice::startAdvertising();
  }
};

void setupBle() {
  Serial.println("[DRV] Khởi động Bluetooth BLE...");
  NimBLEDevice::init("ESP32S3_HID_REMOTE");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new BleServerCallbacks());

  NimBLEService* svc = pServer->createService(kServiceUUID);
  NimBLECharacteristic* pWriteChar = svc->createCharacteristic(
    kWriteCharUUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pWriteChar->setCallbacks(new BleCallbacks());

  pNotifyChar = svc->createCharacteristic(kNotifyCharUUID, NIMBLE_PROPERTY::NOTIFY);

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(kServiceUUID);
  adv->setName("ESP32S3_HID_REMOTE");
  adv->enableScanResponse(true);
  adv->start();

  Serial.println("[BLE] Đang quảng bá tên thiết bị: ESP32S3_HID_REMOTE");
}

void disableBle() {
  if (pServer != nullptr) {
    NimBLEDevice::deinit(true);
    pServer = nullptr;
    pNotifyChar = nullptr;
    delay(100);
  }
}

// ==========================================
// 13. ĐẢO CHẾ ĐỘ PHẦN CỨNG HYBRID (CHỈ GỌI TỪ loop() - CORE 0)
// ==========================================
void switchMode(DeviceMode newMode) {
  if (currentMode == newMode) {
    Serial.printf("[SWITCH] Đã ở mode %s, bỏ qua.\n",
                  newMode == MODE_WIFI ? "WIFI" : "BLE");
    return;
  }
  currentMode = newMode;

  preferences.begin("hybrid-sys", false);
  preferences.putInt("mode", (int)currentMode);
  preferences.end();

  clearAllHardwareStates();

  if (newMode == MODE_WIFI) {
    disableBle();
    delay(200);
    setupWifiSTA();
  } else {
    disableWifi();
    delay(200);
    setupBle();
  }
}

// ==========================================
// 14. SETUP & LOOP CHÍNH (Core 0)
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("=== ESP32-S3 Hybrid HID Core starting ===");

  Keyboard.begin();
  Mouse.begin();
  ConsumerControl.begin();
  USB.begin();

  packetQueue = xQueueCreate(50, sizeof(Packet));

  // Tạo luồng Parser chạy song song trên Core 1
  xTaskCreatePinnedToCore(
    parserTaskWorker,
    "ParserTask",
    4096,
    NULL,
    5,
    &parserTaskHandle,
    1
  );

  preferences.begin("hybrid-sys", true);
  currentMode = (DeviceMode)preferences.getInt("mode", MODE_BLE);
  preferences.end();

  if (currentMode == MODE_WIFI) {
    setupWifiSTA();
  } else {
    setupBle();
  }
}

void loop() {
  // ============================================================
  // XỬ LÝ CHUYỂN MODE AN TOÀN TRÊN CORE 0
  // ParserTask (Core 1) chỉ set cờ — loop() thực thi switchMode
  // ============================================================
  if (gPendingSwitch) {
    gPendingSwitch = false;
    DeviceMode target = gPendingMode;

    // Nếu đang ở Wi-Fi và có client, đóng socket SẠCH trước khi switch
    if (currentMode == MODE_WIFI && activeTcpClient) {
      activeTcpClient.flush();
      while (activeTcpClient.available() > 0) activeTcpClient.read();
      activeTcpClient.stop();
      activeTcpClient = WiFiClient();
      Serial.println("[LOOP] Đã đóng sạch TCP client trước khi switch mode");
    }

    // Gửi phản hồi Wi-Fi config TRƯỚC khi switch (nếu có yêu cầu)
    if (gPendingWifiResponse) {
      gPendingWifiResponse = false;
      sendSystemByteResponse(REP_AUTH_SUCCESS, gPendingWifiIsBT);
      Serial.println("[LOOP] Đã gửi REP_AUTH_SUCCESS cho Wi-Fi config");
      delay(100);
    }

    Serial.printf("[CORE0] Thực thi chuyển mode: %s\n",
                  target == MODE_WIFI ? "WIFI" : "BLE");
    switchMode(target);
  }

  // ============================================================
  // XỬ LÝ WI-FI (chỉ khi đang ở MODE_WIFI)
  // ============================================================
  if (currentMode == MODE_WIFI) {

    // ---- KIỂM TRA CLIENT CŨ CÒN SỐNG KHÔNG ----
    if (activeTcpClient && !activeTcpClient.connected()) {
      Serial.println("[WIFI] Client cũ mất kết nối, dọn dẹp...");
      activeTcpClient.flush();
      while (activeTcpClient.available() > 0) activeTcpClient.read();
      activeTcpClient.stop();
      activeTcpClient = WiFiClient();
      clearAllHardwareStates();
      activeConnection.isConnected = false;
    }

    // ---- ĐÓN CLIENT MỚI (cooldown + kiểm tra hợp lệ) ----
    if (tcpServer.hasClient()) {
      unsigned long now = millis();

      if (now - lastClientChangeTime < CLIENT_CHANGE_COOLDOWN_MS) {
        // Chưa đủ cooldown → từ chối client mới
        WiFiClient rejected = tcpServer.available();
        if (rejected) rejected.stop();
        Serial.println("[WIFI] Từ chối client mới (cooldown chống spam).");
      } else {
        WiFiClient newClient = tcpServer.available();

        // Kiểm tra client mới hợp lệ
        if (!newClient || !newClient.connected()) {
          Serial.println("[WIFI] Client mới không hợp lệ, bỏ qua.");
          if (newClient) newClient.stop();
        } else {
          // Đá văng client cũ (nếu có) — XẢ BUFFER SẠCH trước
          if (activeTcpClient && activeTcpClient.connected()) {
            activeTcpClient.flush();
            while (activeTcpClient.available() > 0) activeTcpClient.read();
            activeTcpClient.stop();
            Serial.println("[WIFI] Đã xả sạch và đá văng Client cũ.");
          }

          // Nhận client mới
          activeTcpClient = newClient;
          activeTcpClient.setNoDelay(true);

          // Reset state hoàn toàn
          clearAllHardwareStates();
          activeConnection.isConnected = true;
          lastClientChangeTime = now;

          Serial.println("[WIFI] Đã nhận client mới. Bộ đệm SẠCH.");
        }
      }
    }

    // ---- ĐỌC DỮ LIỆU TỪ CLIENT ----
    if (activeTcpClient && activeTcpClient.connected()) {
      while (activeTcpClient.available() > 0) {
        uint8_t buffer[64];
        int len = activeTcpClient.read(buffer, min((int)activeTcpClient.available(), 64));
        if (len > 0) {
          pushToQueue(buffer, len);
        }
      }
    }
  }

  delay(1);
}


/**
┌─────────────────────────────────────────────────────────────┐
│                    ESP32-S3 KHỞI ĐỘNG                       │
│  1. Đọc Flash: mode (BLE/WiFi)                              │
│  2. Nếu BLE → setupBle()                                    │
│  3. Nếu WiFi → setupWifiSTA() → tcpServer.begin()           │
└─────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┴───────────────────┐
        ▼                                       ▼
┌───────────────────┐                  ┌───────────────────┐
│   MODE BLE        │                  │   MODE WIFI       │
│                   │                  │                   │
│ - Phát BLE name   │                  │ - TCP server      │
│ - Chờ Android     │                  │ - Chờ Android     │
│   connect GATT    │                  │   Socket connect  │
└───────────────────┘                  └───────────────────┘
        │                                       │
        │ onWrite(bytes)                        │ hasClient()
        ▼                                       ▼
┌─────────────────────────────────────────────────────────────┐
│                    pushToQueue()                            │
│  Copy vào Packet, đẩy vào FreeRTOS queue                    │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              ParserTaskWorker (Core 1)                      │
│  - Đọc từ queue                                             │
│  - Gọi parseHidCommand()                                    │
│  - Chỉ set cờ, KHÔNG gọi switchMode                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                parseHidCommand()                            │
│  1. CMD_PING (0xF7) → REP_PONG (0xF8)                       │
│  2. CMD_SWITCH_TO_WIFI/BLE → set cờ                         │
│  3. CMD_UPDATE_WIFI (0xF6) → Lưu Flash + set cờ             │
│  4. CMD_IDENTIFY_ROLE (0xF1) → Handshake                    │
│  5. CMD_AUTH_PASSWORD (0xF3) → Verify                       │
│  6. V2 TLV [0xAA, 0x01] → HID commands                      │
│  7. V1 3-byte → HID commands                                │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  loop() (Core 0)                            │
│  1. Kiểm tra gPendingSwitch → switchMode()                  │
│  2. Kiểm tra client cũ chết → dọn dẹp                       │
│  3. Đón client mới (cooldown 500ms)                         │
│  4. Đọc bytes → pushToQueue                                 │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              USB HID (Keyboard, Mouse, Consumer)            │
│  → Máy tính nhận lệnh như chuột/bàn phím thật               │
└─────────────────────────────────────────────────────────────┘
*/