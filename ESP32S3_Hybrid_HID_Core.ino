/**
 * ESP32-S3 Hybrid HID Core v4.0
 * 
 * MÔ HÌNH 3 MODE TÁCH BIỆT:
 *   MODE_BLE   (0) — Chỉ BLE GATT
 *   MODE_WIFI  (1) — Chỉ Wi-Fi STA + TCP
 *   MODE_SETUP (2) — Chỉ SoftAP + Web Server
 * 
 * NGUYÊN TẮC:
 *   - Chỉ 1 mode chạy tại một thời điểm
 *   - Không BLE + Wi-Fi song song
 *   - Chuyển mode qua BLE (khi ở BLE) hoặc TCP (khi ở WIFI)
 *   - Nút BOOT giữ 3s → vào MODE_SETUP (dự phòng)
 *   - Cứu hộ tự tắt sau khi lưu Wi-Fi
 * 
 * Bo mạch: ESP32-S3 Dev Module
 * USB Mode: USB-OTG (TinyUSB)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>

// ---- USB HID ----
#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBHIDMouse.h"
#include "USBHIDConsumerControl.h"

// ==========================================
// 0. NÚT BOOT DỰ PHÒNG
// ==========================================
#define BOOT_BUTTON_PIN 0
static unsigned long bootPressStart = 0;
static bool bootPressed = false;
const unsigned long BOOT_HOLD_MS = 3000;  // 3 giây

// ==========================================
// 1. CẤU HÌNH HỆ THỐNG
// ==========================================
enum DeviceMode { MODE_BLE = 0,
                  MODE_WIFI = 1,
                  MODE_SETUP = 2 };
DeviceMode currentMode = MODE_BLE;

Preferences preferences;

// ---- Config động ----
char wifi_ssid_dynamic[64] = "";
char wifi_pass_dynamic[64] = "";

char cfg_ap_ssid[32] = "ESP32S3_HID_Setup";
char cfg_ap_pass[64] = "";

char cfg_ble_name[32] = "ESP32S3_HID_REMOTE";
char cfg_ble_uuid_prefix[5] = "2D2A";

uint16_t cfg_tcp_port = 1989;
char cfg_sys_password[32] = "123456";

// ---- Runtime objects ----
WiFiServer* tcpServer = nullptr;
WiFiClient activeTcpClient;

struct ConnectionState {
  volatile bool isConnected = false;
  volatile bool isIdentified = false;
  volatile bool isVerified = false;
  uint8_t role = 0;
};
volatile ConnectionState activeConnection;

// ==========================================
// 1b. CỜ CHUYỂN MODE
// ==========================================
volatile bool gPendingSwitch = false;
volatile DeviceMode gPendingMode = MODE_BLE;

// ==========================================
// 1c. CỜ QUẢN LÝ SOCKET
// ==========================================
static unsigned long lastClientChangeTime = 0;
const unsigned long CLIENT_CHANGE_COOLDOWN_MS = 500;

// ==========================================
// 1d. WEB SERVER
// ==========================================
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
DNSServer dnsServer;
WebServer webServer(80);

static int wifiFailCount = 0;
const int MAX_WIFI_FAIL = 2;

// ==========================================
// 2. QUEUE
// ==========================================
struct Packet {
  uint8_t data[64];
  size_t length;
};
QueueHandle_t packetQueue = nullptr;
TaskHandle_t parserTaskHandle = NULL;

// ==========================================
// 3. MÃ GIAO THỨC
// ==========================================
const uint8_t CMD_IDENTIFY_ROLE    = 0xF1;
const uint8_t REP_REQUIRE_AUTH     = 0xF2;
const uint8_t CMD_AUTH_PASSWORD    = 0xF3;
const uint8_t REP_AUTH_SUCCESS     = 0xF4;
const uint8_t REP_AUTH_FAILED      = 0xF5;
const uint8_t CMD_UPDATE_WIFI      = 0xF6;
const uint8_t CMD_PING             = 0xF7;
const uint8_t REP_PONG             = 0xF8;
const uint8_t REP_SWITCH_OK        = 0xF9;
const uint8_t REP_WIFI_IP          = 0xFA;
const uint8_t REP_BLE_NAME         = 0xFB;
const uint8_t REP_TCP_PORT         = 0xFC;
const uint8_t CMD_GET_CONFIG       = 0xFD;
const uint8_t CMD_SWITCH_TO_WIFI   = 0xFE;
const uint8_t CMD_SWITCH_TO_BLE    = 0xFF;
const uint8_t CMD_SWITCH_TO_SETUP  = 0xE0;  // ← MỚI

// ---- v1: 3 bytes ----
static const uint8_t CMD_KEY          = 0x01;
static const uint8_t CMD_MOUSE_MOVE   = 0x02;
static const uint8_t CMD_MOUSE_CLICK  = 0x03;
static const uint8_t CMD_MOUSE_SCROLL = 0x04;

// ---- v2: TLV ----
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
static const uint8_t V2_CONSUMER_TAP  = 0x20;
static const uint8_t V2_CONSUMER_DOWN = 0x21;
static const uint8_t V2_CONSUMER_UP   = 0x22;

// ==========================================
// 4. UUID BLE ĐỘNG
// ==========================================
static char gServiceUUID[40];
static char gWriteCharUUID[40];
static char gNotifyCharUUID[40];

static void rebuildBleUUIDs() {
  const char* p = cfg_ble_uuid_prefix;
  if (strlen(p) != 4) p = "2D2A";
  snprintf(gServiceUUID,    sizeof(gServiceUUID),    "%s0001-8A5A-4E76-A2E3-1E57D9A1B001", p);
  snprintf(gWriteCharUUID,  sizeof(gWriteCharUUID),  "%s0002-8A5A-4E76-A2E3-1E57D9A1B001", p);
  snprintf(gNotifyCharUUID, sizeof(gNotifyCharUUID), "%s0003-8A5A-4E76-A2E3-1E57D9A1B001", p);

  Serial.printf("[BLE-UUID] Service: %s\n", gServiceUUID);
  Serial.printf("[BLE-UUID] Write:   %s\n", gWriteCharUUID);
  Serial.printf("[BLE-UUID] Notify:  %s\n", gNotifyCharUUID);
}

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
void setupSoftAP();
void disableSoftAP();
void disableWifi();
void disableBle();
void loadConfigFromNVS();
void saveConfigToNVS();
void saveCurrentModeToNVS();
void startTcpServer();
void stopTcpServer();
void checkBootButton();

// ==========================================
// 5. BẢN ĐỒ MÃ HID
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

const uint8_t MS_LEFT   = 0x01;
const uint8_t MS_RIGHT  = 0x02;
const uint8_t MS_MIDDLE = 0x04;

const uint8_t KB_NONE       = 0x00;
const uint8_t KB_A          = 0x04;
const uint8_t KB_B          = 0x05;
const uint8_t KB_C          = 0x06;
const uint8_t KB_D          = 0x07;
const uint8_t KB_E          = 0x08;
const uint8_t KB_F          = 0x09;
const uint8_t KB_G          = 0x0A;
const uint8_t KB_H          = 0x0B;
const uint8_t KB_I          = 0x0C;
const uint8_t KB_J          = 0x0D;
const uint8_t KB_K          = 0x0E;
const uint8_t KB_L          = 0x0F;
const uint8_t KB_M          = 0x10;
const uint8_t KB_N          = 0x11;
const uint8_t KB_O          = 0x12;
const uint8_t KB_P          = 0x13;
const uint8_t KB_Q          = 0x14;
const uint8_t KB_R          = 0x15;
const uint8_t KB_S          = 0x16;
const uint8_t KB_T          = 0x17;
const uint8_t KB_U          = 0x18;
const uint8_t KB_V          = 0x19;
const uint8_t KB_W          = 0x1A;
const uint8_t KB_X          = 0x1B;
const uint8_t KB_Y          = 0x1C;
const uint8_t KB_Z          = 0x1D;

const uint8_t KB_1 = 0x1E;
const uint8_t KB_2 = 0x1F;
const uint8_t KB_3 = 0x20;
const uint8_t KB_4 = 0x21;
const uint8_t KB_5 = 0x22;
const uint8_t KB_6 = 0x23;
const uint8_t KB_7 = 0x24;
const uint8_t KB_8 = 0x25;
const uint8_t KB_9 = 0x26;
const uint8_t KB_0 = 0x27;

const uint8_t KB_ENTER     = 0x28;
const uint8_t KB_ESCAPE    = 0x29;
const uint8_t KB_BACKSPACE = 0x2A;
const uint8_t KB_TAB       = 0x2B;
const uint8_t KB_SPACE     = 0x2C;
const uint8_t KB_MINUS     = 0x2D;
const uint8_t KB_EQUAL     = 0x2E;
const uint8_t KB_LBRACKET  = 0x2F;
const uint8_t KB_RBRACKET  = 0x30;
const uint8_t KB_BACKSLASH = 0x31;
const uint8_t KB_SEMICOLON = 0x33;
const uint8_t KB_QUOTE     = 0x34;
const uint8_t KB_GRAVE     = 0x35;
const uint8_t KB_COMMA     = 0x36;
const uint8_t KB_PERIOD    = 0x37;
const uint8_t KB_SLASH     = 0x38;

const uint8_t KB_F1  = 0x3A;
const uint8_t KB_F2  = 0x3B;
const uint8_t KB_F3  = 0x3C;
const uint8_t KB_F4  = 0x3D;
const uint8_t KB_F5  = 0x3E;
const uint8_t KB_F6  = 0x3F;
const uint8_t KB_F7  = 0x40;
const uint8_t KB_F8  = 0x41;
const uint8_t KB_F9  = 0x42;
const uint8_t KB_F10 = 0x43;
const uint8_t KB_F11 = 0x44;
const uint8_t KB_F12 = 0x45;

// ==========================================
// 6. TẦNG DRIVER HID
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

static void sendMouseMove(int8_t dx, int8_t dy)  { Mouse.move(dx, dy); }
static void sendMouseClick(uint8_t button)       { Mouse.click(button); }
static void sendMouseButtonDown(uint8_t button)  { Mouse.press(button); }
static void sendMouseButtonUp(uint8_t button)    { Mouse.release(button); }
static void sendMouseScroll(int8_t dx, int8_t dy){ Mouse.move(0, 0, dy, dx); }

// ==========================================
// 6b. SMART KEY TAP — Mảng lookup
// ==========================================
static const uint8_t CONSUMER_KEYS[] = {
  0x30, 0x32, 0x83, 0xE2, 0xE9, 0xEA, 0xCD, 0xB5,
  0xB6, 0xB7, 0xB3, 0xB4, 0x6F, 0x70, 0x9E, 0x76
};
static const size_t CONSUMER_KEYS_COUNT = sizeof(CONSUMER_KEYS) / sizeof(CONSUMER_KEYS[0]);

static bool isConsumerKey(uint8_t k) {
  for (size_t i = 0; i < CONSUMER_KEYS_COUNT; i++) {
    if (CONSUMER_KEYS[i] == k) return true;
  }
  return false;
}

static void handleSmartKeyTap(uint8_t modifiers, uint8_t keycode) {
  if (keycode == 0x00) return;
  if (isConsumerKey(keycode)) {
    ConsumerControl.press(keycode);
    delay(10);
    ConsumerControl.release();
    return;
  }
  keyTap(modifiers, keycode);
}

// ==========================================
// 6c. CONSUMER ACTION
// ==========================================
static void handleConsumerAction(uint8_t cmd, uint16_t usageCode) {
  uint8_t mappedCode = 0;
  switch (usageCode) {
    case 0x0030: mappedCode = 0x30; break;
    case 0x0032: mappedCode = 0x32; break;
    case 0x0083: mappedCode = 0x83; break;
    case 0x00E2: mappedCode = 0xE2; break;
    case 0x00E9: mappedCode = 0xE9; break;
    case 0x00EA: mappedCode = 0xEA; break;
    case 0x00CD: mappedCode = 0xCD; break;
    case 0x00B5: mappedCode = 0xB5; break;
    case 0x00B6: mappedCode = 0xB6; break;
    case 0x00B7: mappedCode = 0xB7; break;
    case 0x00B3: mappedCode = 0xB3; break;
    case 0x00B4: mappedCode = 0xB4; break;
    case 0x006F: mappedCode = 0x6F; break;
    case 0x0070: mappedCode = 0x70; break;
    case 0x009E: mappedCode = 0x9E; break;
    case 0x0076: mappedCode = 0x76; break;
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
    default: break;
  }
}

// ==========================================
// 7. DỌN SẠCH TÀI NGUYÊN
// ==========================================
void clearAllHardwareStates() {
  Serial.println("[CLEANER] Dọn sạch HID + buffer...");

  Keyboard.releaseAll();
  memset(gKeysDown, 0, sizeof(gKeysDown));
  gModifiersMask = 0x00;

  Mouse.release(MS_LEFT);
  Mouse.release(MS_RIGHT);
  Mouse.release(MS_MIDDLE);

  ConsumerControl.release();

  if (packetQueue != nullptr) {
    xQueueReset(packetQueue);
  }

  if (activeTcpClient && activeTcpClient.connected()) {
    while (activeTcpClient.available() > 0) activeTcpClient.read();
  }

  activeConnection.isConnected  = false;
  activeConnection.isIdentified = false;
  activeConnection.isVerified   = false;
  activeConnection.role         = 0;

  delay(20);
  Serial.println("[CLEANER] ✅ Đã dọn sạch.");
}

bool verifyPlainPassword(const uint8_t* inputBytes, size_t length) {
  size_t storedLen = strlen(cfg_sys_password);
  if (length != storedLen) return false;
  return memcmp(inputBytes, cfg_sys_password, length) == 0;
}

// ==========================================
// 8. PARSER
// ==========================================
void sendIpOverCurrentChannel(bool isBluetooth);

void parseHidCommand(const uint8_t* data, size_t length, bool isBluetooth) {
  if (length == 0 || data == nullptr) return;

  // ---- PING ----
  if (data[0] == CMD_PING) {
    sendSystemByteResponse(REP_PONG, isBluetooth);
    Serial.println("[PING] Đã nhận CMD_PING, phản hồi REP_PONG");
    return;
  }

  // ---- GET CONFIG ----
  if (data[0] == CMD_GET_CONFIG) {
    Serial.println("[CFG] Client yêu cầu cấu hình");
    {
      uint8_t buf[64];
      size_t nameLen = strlen(cfg_ble_name);
      if (nameLen > 60) nameLen = 60;
      buf[0] = REP_BLE_NAME;
      buf[1] = (uint8_t)nameLen;
      memcpy(&buf[2], cfg_ble_name, nameLen);
      if (isBluetooth && pNotifyChar && pServer && pServer->getConnectedCount() > 0) {
        pNotifyChar->setValue(buf, 2 + nameLen);
        pNotifyChar->notify();
      } else if (!isBluetooth && activeTcpClient && activeTcpClient.connected()) {
        activeTcpClient.write(buf, 2 + nameLen);
        activeTcpClient.flush();
      }
    }
    {
      uint8_t buf[3];
      buf[0] = REP_TCP_PORT;
      buf[1] = (uint8_t)(cfg_tcp_port >> 8);
      buf[2] = (uint8_t)(cfg_tcp_port & 0xFF);
      if (isBluetooth && pNotifyChar && pServer && pServer->getConnectedCount() > 0) {
        pNotifyChar->setValue(buf, 3);
        pNotifyChar->notify();
      } else if (!isBluetooth && activeTcpClient && activeTcpClient.connected()) {
        activeTcpClient.write(buf, 3);
        activeTcpClient.flush();
      }
    }
    sendIpOverCurrentChannel(isBluetooth);
    return;
  }

  // ---- SWITCH MODE ----
  if (data[0] == CMD_SWITCH_TO_WIFI) {
    Serial.println("[PARSER] Yêu cầu chuyển sang WIFI");
    sendSystemByteResponse(REP_SWITCH_OK, isBluetooth);
    delay(100);
    gPendingMode = MODE_WIFI;
    gPendingSwitch = true;
    return;
  }
  if (data[0] == CMD_SWITCH_TO_BLE) {
    Serial.println("[PARSER] Yêu cầu chuyển sang BLE");
    sendSystemByteResponse(REP_SWITCH_OK, isBluetooth);
    delay(100);
    gPendingMode = MODE_BLE;
    gPendingSwitch = true;
    return;
  }
  if (data[0] == CMD_SWITCH_TO_SETUP) {
    Serial.println("[PARSER] Yêu cầu chuyển sang SETUP");
    sendSystemByteResponse(REP_SWITCH_OK, isBluetooth);
    delay(100);
    gPendingMode = MODE_SETUP;
    gPendingSwitch = true;
    return;
  }

  // ---- UPDATE WIFI (legacy) ----
  if (data[0] == CMD_UPDATE_WIFI && length >= 2) {
    size_t ssidLen   = data[1];
    size_t ssidStart = 2;
    size_t ssidEnd   = ssidStart + ssidLen;
    if (ssidEnd + 1 > length) {
      Serial.println("[WIFI-CFG] Lỗi ssidLen");
      return;
    }

    size_t passLenIdx = ssidEnd;
    size_t passLen    = data[passLenIdx];
    size_t passStart  = passLenIdx + 1;
    size_t passEnd    = passStart + passLen;
    if (passEnd > length) {
      Serial.println("[WIFI-CFG] Lỗi passLen");
      return;
    }

    char newSSID[64] = { 0 };
    char newPASS[64] = { 0 };
    size_t copySSID = (ssidLen < 63) ? ssidLen : 63;
    size_t copyPASS = (passLen < 63) ? passLen : 63;
    memcpy(newSSID, &data[ssidStart], copySSID);
    memcpy(newPASS, &data[passStart], copyPASS);
    newSSID[copySSID] = '\0';
    newPASS[copyPASS] = '\0';

    preferences.begin("hybrid-sys", false);
    preferences.putString("wf_ssid", newSSID);
    preferences.putString("wf_pass", newPASS);
    preferences.end();

    strncpy(wifi_ssid_dynamic, newSSID, sizeof(wifi_ssid_dynamic) - 1);
    wifi_ssid_dynamic[sizeof(wifi_ssid_dynamic) - 1] = '\0';
    strncpy(wifi_pass_dynamic, newPASS, sizeof(wifi_pass_dynamic) - 1);
    wifi_pass_dynamic[sizeof(wifi_pass_dynamic) - 1] = '\0';

    Serial.printf("[FLASH] Wi-Fi mới: SSID='%s'\n", newSSID);

    gPendingMode = MODE_WIFI;
    gPendingSwitch = true;
    return;
  }

  // ---- HANDSHAKE ----
  if (!activeConnection.isIdentified) {
    if (data[0] == CMD_IDENTIFY_ROLE && length >= 2) {
      activeConnection.role = data[1];
      activeConnection.isIdentified = true;

      if (activeConnection.role == 0x02) {
        activeConnection.isVerified = true;
        sendSystemByteResponse(REP_AUTH_SUCCESS, true);
        Serial.println("[BLE] Xác thực lớp vật lý thành công. READY.");
      } else {
        sendSystemByteResponse(REP_REQUIRE_AUTH, false);
        Serial.println("[WIFI] Yêu cầu nhập mật khẩu.");
      }
    }
    return;
  }

  // ---- AUTH ----
  if (!activeConnection.isVerified) {
    if (data[0] == CMD_AUTH_PASSWORD && length >= 2) {
      size_t passLen = data[1];
      if (length >= (2 + passLen)) {
        if (verifyPlainPassword(&data[2], passLen)) {
          activeConnection.isVerified = true;
          sendSystemByteResponse(REP_AUTH_SUCCESS, isBluetooth);
          Serial.println("[AUTH] Đúng mật khẩu.");
        } else {
          sendSystemByteResponse(REP_AUTH_FAILED, isBluetooth);
          Serial.println("[AUTH] Sai mật khẩu.");
        }
      }
    }
    return;
  }

  // ---- V2 TLV ----
  if (length >= 2 && data[0] == V2_MAGIC && data[1] == V2_VERSION) {
    size_t idx = 2;
    while (idx + 1 < length) {
      uint8_t cmd = data[idx + 0];
      uint8_t len = data[idx + 1];
      idx += 2;
      if (idx + len > length) break;
      const uint8_t* payload = &data[idx];

      switch (cmd) {
        case V2_SET_MODIFIERS: if (len == 1) setModifiers(payload[0]); break;
        case V2_KEY_DOWN:      if (len == 1) keyDown(payload[0]); break;
        case V2_KEY_UP:        if (len == 1) keyUp(payload[0]); break;
        case V2_KEY_TAP:       if (len == 2) handleSmartKeyTap(payload[0], payload[1]); break;
        case V2_MOUSE_MOVE:    if (len == 2) sendMouseMove((int8_t)payload[0], (int8_t)payload[1]); break;
        case V2_MOUSE_SCROLL:  if (len == 2) sendMouseScroll((int8_t)payload[0], (int8_t)payload[1]); break;
        case V2_MOUSE_CLICK:   if (len == 1) sendMouseClick(payload[0]); break;
        case V2_MOUSE_DOWN:    if (len == 1) sendMouseButtonDown(payload[0]); break;
        case V2_MOUSE_UP:      if (len == 1) sendMouseButtonUp(payload[0]); break;
        case V2_CONSUMER_TAP:
        case V2_CONSUMER_DOWN:
        case V2_CONSUMER_UP:
          if (len == 2) {
            uint16_t usageCode = (uint16_t)((payload[0] << 8) | payload[1]);
            handleConsumerAction(cmd, usageCode);
          }
          break;
        default: break;
      }
      idx += len;
    }
    return;
  }

  // ---- V1 3-byte ----
  for (size_t i = 0; i + 2 < length; i += 3) {
    uint8_t type  = data[i + 0];
    uint8_t byte1 = data[i + 1];
    uint8_t byte2 = data[i + 2];

    switch (type) {
      case CMD_KEY:          handleSmartKeyTap(byte1, byte2); break;
      case CMD_MOUSE_MOVE:   sendMouseMove((int8_t)byte1, (int8_t)byte2); break;
      case CMD_MOUSE_CLICK:  sendMouseClick(byte1); break;
      case CMD_MOUSE_SCROLL: sendMouseScroll((int8_t)byte1, (int8_t)byte2); break;
      default: break;
    }
  }
}

// ==========================================
// 9. QUEUE
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
// 10. PHẢN HỒI
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

void sendIpOverCurrentChannel(bool isBluetooth) {
  IPAddress ip;
  if (currentMode == MODE_WIFI && WiFi.status() == WL_CONNECTED) {
    ip = WiFi.localIP();
  } else if (currentMode == MODE_SETUP) {
    ip = WiFi.softAPIP();
  } else {
    return;
  }
  uint8_t buf[5];
  buf[0] = REP_WIFI_IP;
  buf[1] = ip[0];
  buf[2] = ip[1];
  buf[3] = ip[2];
  buf[4] = ip[3];
  if (isBluetooth) {
    if (pNotifyChar && pServer && pServer->getConnectedCount() > 0) {
      pNotifyChar->setValue(buf, 5);
      pNotifyChar->notify();
    }
  } else {
    if (activeTcpClient && activeTcpClient.connected()) {
      activeTcpClient.write(buf, 5);
      activeTcpClient.flush();
    }
  }
}

// ==========================================
// 11. NVS CONFIG
// ==========================================
void loadConfigFromNVS() {
  preferences.begin("hybrid-sys", true);

  String tmp;
  tmp = preferences.getString("wf_ssid", "");
  strncpy(wifi_ssid_dynamic, tmp.c_str(), sizeof(wifi_ssid_dynamic) - 1);
  wifi_ssid_dynamic[sizeof(wifi_ssid_dynamic) - 1] = '\0';

  tmp = preferences.getString("wf_pass", "");
  strncpy(wifi_pass_dynamic, tmp.c_str(), sizeof(wifi_pass_dynamic) - 1);
  wifi_pass_dynamic[sizeof(wifi_pass_dynamic) - 1] = '\0';

  tmp = preferences.getString("ap_ssid", "ESP32S3_HID_Setup");
  strncpy(cfg_ap_ssid, tmp.c_str(), sizeof(cfg_ap_ssid) - 1);
  cfg_ap_ssid[sizeof(cfg_ap_ssid) - 1] = '\0';

  tmp = preferences.getString("ap_pass", "");
  strncpy(cfg_ap_pass, tmp.c_str(), sizeof(cfg_ap_pass) - 1);
  cfg_ap_pass[sizeof(cfg_ap_pass) - 1] = '\0';

  tmp = preferences.getString("ble_name", "ESP32S3_HID_REMOTE");
  strncpy(cfg_ble_name, tmp.c_str(), sizeof(cfg_ble_name) - 1);
  cfg_ble_name[sizeof(cfg_ble_name) - 1] = '\0';

  tmp = preferences.getString("ble_uuid_pre", "2D2A");
  strncpy(cfg_ble_uuid_prefix, tmp.c_str(), sizeof(cfg_ble_uuid_prefix) - 1);
  cfg_ble_uuid_prefix[sizeof(cfg_ble_uuid_prefix) - 1] = '\0';

  tmp = preferences.getString("sys_password", "123456");
  strncpy(cfg_sys_password, tmp.c_str(), sizeof(cfg_sys_password) - 1);
  cfg_sys_password[sizeof(cfg_sys_password) - 1] = '\0';

  cfg_tcp_port = preferences.getUShort("tcp_port", 1989);

  // ⚠️ MẶC ĐỊNH MODE_BLE KHI NVS TRỐNG
  currentMode = (DeviceMode)preferences.getInt("last_mode", MODE_BLE);
  preferences.end();

  Serial.println("[NVS] Đã load cấu hình:");
  Serial.printf("  - Wi-Fi SSID: %s\n", wifi_ssid_dynamic);
  Serial.printf("  - AP SSID: %s\n", cfg_ap_ssid);
  Serial.printf("  - BLE name: %s\n", cfg_ble_name);
  Serial.printf("  - BLE UUID prefix: %s\n", cfg_ble_uuid_prefix);
  Serial.printf("  - TCP port: %u\n", cfg_tcp_port);
  Serial.printf("  - Sys password: %s\n", cfg_sys_password);
  Serial.printf("  - Last mode: %d\n", (int)currentMode);

  rebuildBleUUIDs();
}

void saveConfigToNVS() {
  preferences.begin("hybrid-sys", false);
  preferences.putString("wf_ssid", wifi_ssid_dynamic);
  preferences.putString("wf_pass", wifi_pass_dynamic);
  preferences.putString("ap_ssid", cfg_ap_ssid);
  preferences.putString("ap_pass", cfg_ap_pass);
  preferences.putString("ble_name", cfg_ble_name);
  preferences.putString("ble_uuid_pre", cfg_ble_uuid_prefix);
  preferences.putUShort("tcp_port", cfg_tcp_port);
  preferences.putString("sys_password", cfg_sys_password);
  preferences.end();
  Serial.println("[NVS] Đã lưu cấu hình.");
}

void saveCurrentModeToNVS() {
  preferences.begin("hybrid-sys", false);
  preferences.putInt("last_mode", (int)currentMode);
  preferences.end();
  Serial.printf("[NVS] Đã lưu mode: %d\n", (int)currentMode);
}

void factoryResetNVS() {
  preferences.begin("hybrid-sys", false);
  preferences.clear();
  preferences.end();
  Serial.println("[NVS] FACTORY RESET — đã xóa toàn bộ cấu hình.");
}

// ==========================================
// 12. QUẢN LÝ TCP SERVER
// ==========================================
void stopTcpServer() {
  if (activeTcpClient) {
    activeTcpClient.flush();
    while (activeTcpClient.available() > 0) activeTcpClient.read();
    activeTcpClient.stop();
    activeTcpClient = WiFiClient();
  }
  if (tcpServer != nullptr) {
    tcpServer->end();
    tcpServer->stop();
    delete tcpServer;
    tcpServer = nullptr;
    Serial.println("[TCP] Đã đóng TCP server.");
  }
}

void startTcpServer() {
  stopTcpServer();
  delay(50);

  tcpServer = new WiFiServer(cfg_tcp_port);
  if (tcpServer == nullptr) {
    Serial.println("[TCP] ❌ Không đủ RAM!");
    return;
  }
  tcpServer->begin();
  tcpServer->setNoDelay(true);
  Serial.printf("[TCP] ✅ TCP server port %u\n", cfg_tcp_port);
}

// ==========================================
// 13. WI-FI STA
// ==========================================
void setupWifiSTA() {
  Serial.println("[DRV] Đang đọc cấu hình Wi-Fi...");
  if (strlen(wifi_ssid_dynamic) == 0) {
    Serial.println("[WIFI] Bộ nhớ trống! Chuyển sang SETUP...");
    gPendingMode = MODE_SETUP;
    gPendingSwitch = true;
    return;
  }

  Serial.printf("[WIFI] Kết nối tới: %s\n", wifi_ssid_dynamic);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("ESP32S3-HID");
  WiFi.begin(wifi_ssid_dynamic, wifi_pass_dynamic);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 8000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiFailCount = 0;
    Serial.print("\n[WIFI] OK! IP: ");
    Serial.println(WiFi.localIP());

    startTcpServer();
    saveCurrentModeToNVS();
  } else {
    wifiFailCount++;
    Serial.printf("\n[WIFI] LỖI MẠNG (%d/%d): CỨU HỘ...\n",
                  wifiFailCount, MAX_WIFI_FAIL);
    if (wifiFailCount >= MAX_WIFI_FAIL) {
      gPendingMode = MODE_SETUP;
      gPendingSwitch = true;
    } else {
      delay(2000);
      setupWifiSTA();
    }
  }
}

void disableWifi() {
  stopTcpServer();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  delay(100);
  Serial.println("[WIFI] Đã tắt Wi-Fi.");
}

// ==========================================
// 14. BLE
// ==========================================
class BleCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    std::string v = pChar->getValue();
    if (v.size() == 0) return;
    if (v.size() > 64) {
      Serial.printf("[BLE] ⚠️ Gói quá lớn (%u bytes), cắt xuống 64.\n", (unsigned)v.size());
      v.resize(64);
    }
    pushToQueue((const uint8_t*)v.data(), v.size());
  }
};

class BleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    Serial.println("[BLE] 🔗 Client Connected");
    clearAllHardwareStates();
    delay(30);
    activeConnection.isConnected = true;
    activeConnection.isIdentified = false;
    activeConnection.isVerified = false;
    activeConnection.role = 0;
  }

  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    Serial.printf("[BLE] 🔌 Disconnected (reason=%d)\n", reason);
    clearAllHardwareStates();
    delay(50);
    if (currentMode == MODE_BLE) {
      NimBLEDevice::startAdvertising();
    }
  }
};

void setupBle() {
  Serial.println("[DRV] Khởi động BLE...");

  NimBLEDevice::init(cfg_ble_name);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setDeviceName(cfg_ble_name);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new BleServerCallbacks());

  NimBLEService* svc = pServer->createService(gServiceUUID);

  NimBLECharacteristic* pWriteChar = svc->createCharacteristic(
    gWriteCharUUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pWriteChar->setCallbacks(new BleCallbacks());

  pNotifyChar = svc->createCharacteristic(
    gNotifyCharUUID,
    NIMBLE_PROPERTY::NOTIFY);

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(cfg_ble_name);
  adv->addServiceUUID(gServiceUUID);
  adv->enableScanResponse(true);
  adv->setConnectableMode(BLE_GAP_CONN_MODE_UND);
  adv->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);

  bool started = adv->start();
  if (started) {
    Serial.printf("[BLE] ✅ Advertising: %s\n", cfg_ble_name);
    Serial.printf("[BLE] Service UUID: %s\n", gServiceUUID);
  } else {
    Serial.println("[BLE] ❌ Advertising FAILED");
    NimBLEDevice::startAdvertising();
  }
}

void disableBle() {
  if (pServer != nullptr) {
    Serial.println("[BLE] Đang tắt BLE...");
    NimBLEDevice::deinit(true);
    pServer = nullptr;
    pNotifyChar = nullptr;
    delay(200);  // ⚠️ Đợi radio giải phóng
    Serial.println("[BLE] ✅ Đã tắt BLE.");
  }
}

// ==========================================
// 15. WEB SERVER (MODE_SETUP)
// ==========================================
String getWiFiScanOptions() {
  Serial.println("[WEB] Quét Wi-Fi...");
  int n = WiFi.scanNetworks();
  String options = "";
  options.reserve(4096); 
  if (n <= 0) {
    options = "<option value=''>Không tìm thấy Wi-Fi</option>";
  } else {
    for (int i = 0; i < n; i++) {
      String ssid = WiFi.SSID(i);
      int rssi = WiFi.RSSI(i);
      String enc = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "🔓" : "🔒";
      options += "<option value='" + ssid + "'>" + enc + " " + ssid + " (" + String(rssi) + " dBm)</option>";
    }
  }
  WiFi.scanDelete();
  return options;
}

String buildDashboardFromFS() {
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    Serial.println("[FS] ❌ Không tìm thấy /index.html");
    return "<meta charset='UTF-8'><h1>Lỗi: chưa upload filesystem</h1>"
           "<p>Chạy Tools → ESP32 Sketch Data Upload</p>";
  }

  String html = f.readString();
  f.close();

  String modeStr = "SETUP";
  if (currentMode == MODE_BLE) modeStr = "BLUETOOTH";
  else if (currentMode == MODE_WIFI) modeStr = "WI-FI STA";

  String ipStr = "-";
  if (currentMode == MODE_WIFI && WiFi.status() == WL_CONNECTED)
    ipStr = WiFi.localIP().toString();
  else if (currentMode == MODE_SETUP)
    ipStr = WiFi.softAPIP().toString();

  html.replace("%MODE%", modeStr);
  html.replace("%IP%", ipStr);
  html.replace("%BLE_NAME%", String(cfg_ble_name));
  html.replace("%TCP_PORT%", String(cfg_tcp_port));
  html.replace("%WIFI_SSID%", strlen(wifi_ssid_dynamic) > 0 ? String(wifi_ssid_dynamic) : String("(chưa cấu hình)"));
  html.replace("%WIFI_OPTIONS%", getWiFiScanOptions());
  html.replace("%AP_SSID%", String(cfg_ap_ssid));
  html.replace("%AP_PASS%", String(cfg_ap_pass));
  html.replace("%UUID_PREFIX%", String(cfg_ble_uuid_prefix));
  html.replace("%SYS_PASSWORD%", String(cfg_sys_password));

  return html;
}

void handleRoot() {
  Serial.println("[WEB] Client truy cập /");
  webServer.send(200, "text/html; charset=utf-8", buildDashboardFromFS());
}

void handleSaveWifi() {
  String ssid = webServer.arg("ssid");
  String ssidManual = webServer.arg("ssid_manual");
  String password = webServer.arg("password");
  if (ssidManual.length() > 0) ssid = ssidManual;

  if (ssid.length() == 0) {
    webServer.send(400, "text/plain", "SSID không được để trống");
    return;
  }

  strncpy(wifi_ssid_dynamic, ssid.c_str(), sizeof(wifi_ssid_dynamic) - 1);
  wifi_ssid_dynamic[sizeof(wifi_ssid_dynamic) - 1] = '\0';
  strncpy(wifi_pass_dynamic, password.c_str(), sizeof(wifi_pass_dynamic) - 1);
  wifi_pass_dynamic[sizeof(wifi_pass_dynamic) - 1] = '\0';

  preferences.begin("hybrid-sys", false);
  preferences.putString("wf_ssid", wifi_ssid_dynamic);
  preferences.putString("wf_pass", wifi_pass_dynamic);
  preferences.end();

  Serial.printf("[WEB] Lưu Wi-Fi: SSID='%s'\n", wifi_ssid_dynamic);
  webServer.send(200, "text/html; charset=utf-8",
                 "<meta charset='UTF-8'><body style='background:#1a1a1a;color:#0f0;font-family:sans-serif;"
                 "text-align:center;padding:50px'><h1>✅ Đã lưu Wi-Fi!</h1>"
                 "<p>Board sẽ chuyển sang chế độ Wi-Fi...</p></body>");

  delay(1500);
  gPendingMode = MODE_WIFI;
  gPendingSwitch = true;
}

void handleSaveAp() {
  String ap_ssid_str = webServer.arg("ap_ssid");
  String ap_pass_str = webServer.arg("ap_pass");

  if (ap_ssid_str.length() > 0) {
    strncpy(cfg_ap_ssid, ap_ssid_str.c_str(), sizeof(cfg_ap_ssid) - 1);
    cfg_ap_ssid[sizeof(cfg_ap_ssid) - 1] = '\0';
  }
  strncpy(cfg_ap_pass, ap_pass_str.c_str(), sizeof(cfg_ap_pass) - 1);
  cfg_ap_pass[sizeof(cfg_ap_pass) - 1] = '\0';

  preferences.begin("hybrid-sys", false);
  preferences.putString("ap_ssid", cfg_ap_ssid);
  preferences.putString("ap_pass", cfg_ap_pass);
  preferences.end();

  webServer.send(200, "text/html; charset=utf-8",
                 "<meta charset='UTF-8'><body style='background:#1a1a1a;color:#0f0;font-family:sans-serif;"
                 "text-align:center;padding:50px'><h1>✅ Đã lưu AP config!</h1>"
                 "<p>SoftAP sẽ khởi động lại sau 2 giây...</p></body>");

  delay(2000);
  Serial.println("[AP] Restart SoftAP...");
  disableSoftAP();
  delay(500);
  setupSoftAP();
}

void handleSaveBle() {
  String ble_name_str = webServer.arg("ble_name");
  String uuid_prefix_str = webServer.arg("ble_uuid_prefix");

  if (ble_name_str.length() > 0) {
    strncpy(cfg_ble_name, ble_name_str.c_str(), sizeof(cfg_ble_name) - 1);
    cfg_ble_name[sizeof(cfg_ble_name) - 1] = '\0';
  }
  if (uuid_prefix_str.length() == 4) {
    strncpy(cfg_ble_uuid_prefix, uuid_prefix_str.c_str(), 4);
    cfg_ble_uuid_prefix[4] = '\0';
  }

  preferences.begin("hybrid-sys", false);
  preferences.putString("ble_name", cfg_ble_name);
  preferences.putString("ble_uuid_pre", cfg_ble_uuid_prefix);
  preferences.end();

  rebuildBleUUIDs();

  webServer.send(200, "text/html; charset=utf-8",
                 "<meta charset='UTF-8'><body style='background:#1a1a1a;color:#0f0;font-family:sans-serif;"
                 "text-align:center;padding:50px'><h1>✅ Đã lưu BLE config!</h1>"
                 "<p>Cần chuyển sang BLE mode để áp dụng.</p>"
                 "<script>setTimeout(()=>location.href='/',3000)</script>");
}

void handleSaveTcp() {
  String tcp_port_str = webServer.arg("tcp_port");
  String sys_pass_str = webServer.arg("sys_password");

  if (tcp_port_str.length() > 0) {
    int port = tcp_port_str.toInt();
    if (port >= 1024 && port <= 65535) cfg_tcp_port = (uint16_t)port;
  }
  if (sys_pass_str.length() > 0) {
    strncpy(cfg_sys_password, sys_pass_str.c_str(), sizeof(cfg_sys_password) - 1);
    cfg_sys_password[sizeof(cfg_sys_password) - 1] = '\0';
  }

  preferences.begin("hybrid-sys", false);
  preferences.putUShort("tcp_port", cfg_tcp_port);
  preferences.putString("sys_password", cfg_sys_password);
  preferences.end();

  webServer.send(200, "text/html; charset=utf-8",
                 "<meta charset='UTF-8'><body style='background:#1a1a1a;color:#0f0;font-family:sans-serif;"
                 "text-align:center;padding:50px'><h1>✅ Đã lưu TCP config!</h1>"
                 "<script>setTimeout(()=>location.href='/',3000)</script>");
}

void handleApiStatus() {
  String json = "{";
  json += "\"mode\":" + String((int)currentMode) + ",";
  json += "\"modeName\":\"" + String(currentMode == MODE_BLE ? "BLE" : currentMode == MODE_WIFI ? "WIFI" : "SETUP") + "\",";
  json += "\"ip\":\"" + (currentMode == MODE_WIFI && WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : currentMode == MODE_SETUP ? WiFi.softAPIP().toString() : "") + "\",";
  json += "\"bleName\":\"" + String(cfg_ble_name) + "\",";
  json += "\"tcpPort\":" + String(cfg_tcp_port) + ",";
  json += "\"wifiSsid\":\"" + String(wifi_ssid_dynamic) + "\"";
  json += "}";
  webServer.send(200, "application/json", json);
}

void handleApiSwitch() {
  String mode = webServer.arg("mode");
  DeviceMode target = MODE_SETUP;
  if (mode == "ble") target = MODE_BLE;
  else if (mode == "wifi") target = MODE_WIFI;
  else if (mode == "setup") target = MODE_SETUP;

  Serial.printf("[WEB-API] Chuyển mode: %s → %d\n", mode.c_str(), (int)target);

  webServer.send(200, "text/plain", "OK");
  delay(300);

  gPendingMode = target;
  gPendingSwitch = true;
}

void handleApiFactoryReset() {
  Serial.println("[WEB-API] FACTORY RESET...");
  webServer.send(200, "text/plain", "OK");
  delay(500);
  factoryResetNVS();
  delay(500);
  ESP.restart();
}

void handleNotFound() {
  webServer.send(404, "text/plain", "404 Not Found");
}

void setupSoftAP() {
  Serial.println("╔═══════════════════════════════════╗");
  Serial.println("║  MODE_SETUP — WEB CONFIG HUB      ║");
  Serial.println("╚═══════════════════════════════════╝");

  if (!LittleFS.begin(true)) {
    Serial.println("[FS] ❌ LittleFS mount failed!");
  } else {
    Serial.println("[FS] ✅ LittleFS mounted");
  }

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(cfg_ap_ssid, cfg_ap_pass);

  Serial.printf("[AP] SSID: %s\n", cfg_ap_ssid);
  Serial.printf("[AP] IP: %s\n", WiFi.softAPIP().toString().c_str());

  dnsServer.start(DNS_PORT, "*", apIP);

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/save/wifi", HTTP_POST, handleSaveWifi);
  webServer.on("/save/ap", HTTP_POST, handleSaveAp);
  webServer.on("/save/ble", HTTP_POST, handleSaveBle);
  webServer.on("/save/tcp", HTTP_POST, handleSaveTcp);
  webServer.on("/api/status", HTTP_GET, handleApiStatus);
  webServer.on("/api/switch", HTTP_POST, handleApiSwitch);
  webServer.on("/api/factory-reset", HTTP_POST, handleApiFactoryReset);
  webServer.onNotFound(handleNotFound);

  webServer.begin();

  Serial.println("[WEB] Web server sẵn sàng");
  saveCurrentModeToNVS();
}

void disableSoftAP() {
  webServer.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  Serial.println("[AP] Đã tắt SoftAP.");
}

// ==========================================
// 16. NÚT BOOT DỰ PHÒNG
// ==========================================
void checkBootButton() {
  bool currentState = (digitalRead(BOOT_BUTTON_PIN) == LOW);

  if (currentState && !bootPressed) {
    bootPressed = true;
    bootPressStart = millis();
  } else if (currentState && bootPressed) {
    if (millis() - bootPressStart >= BOOT_HOLD_MS) {
      Serial.println("[BOOT] Giữ 3s — chuyển sang SETUP!");
      bootPressed = false;

      if (currentMode != MODE_SETUP) {
        gPendingMode = MODE_SETUP;
        gPendingSwitch = true;
      }
    }
  } else {
    bootPressed = false;
  }
}

// ==========================================
// 17. SWITCH MODE — MƯỢT MÀ, TẮT TẤT CẢ TRƯỚC
// ==========================================
void switchMode(DeviceMode newMode) {
  if (currentMode == newMode) {
    Serial.printf("[SWITCH] Đã ở mode %d, bỏ qua.\n", newMode);
    return;
  }

  Serial.printf("\n[SWITCH] ========== %d → %d ==========\n",
                (int)currentMode, (int)newMode);

  // ---- Bước 1: Dọn sạch HID ----
  Serial.println("[SWITCH] Bước 1: Dọn sạch HID + buffer...");
  clearAllHardwareStates();
  delay(50);

  // ---- Bước 2: TẮT TẤT CẢ mode cũ ----
  Serial.println("[SWITCH] Bước 2: Tắt tất cả mode cũ...");
  DeviceMode oldMode = currentMode;
  currentMode = newMode;

  // Tắt BLE nếu đang bật
  if (pServer != nullptr) {
    disableBle();
  }

  // Tắt Wi-Fi STA nếu đang ở MODE_WIFI
  if (oldMode == MODE_WIFI) {
    disableWifi();
  }

  // Tắt SoftAP nếu đang ở MODE_SETUP
  if (oldMode == MODE_SETUP) {
    disableSoftAP();
  }

  // ⚠️ Đợi radio giải phóng hoàn toàn
  delay(300);

  // ---- Bước 3: Bật CHỈ mode mới ----
  Serial.println("[SWITCH] Bước 3: Khởi động mode mới...");

  switch (newMode) {
    case MODE_BLE:
      Serial.println("[SWITCH] → MODE_BLE");
      setupBle();
      break;

    case MODE_WIFI:
      Serial.println("[SWITCH] → MODE_WIFI");
      setupWifiSTA();
      break;

    case MODE_SETUP:
      Serial.println("[SWITCH] → MODE_SETUP");
      setupSoftAP();
      break;
  }

  saveCurrentModeToNVS();
  Serial.printf("[SWITCH] ✅ Hoàn tất: mode = %d\n\n", (int)currentMode);
}

// ==========================================
// 18. SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("=== ESP32-S3 Hybrid HID Core v4.0 ===");
  Serial.println("=== 3 MODE TÁCH BIỆT ===");

  // ---- Nút BOOT ----
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  // ---- USB HID ----
  Keyboard.begin();
  Mouse.begin();
  ConsumerControl.begin();
  USB.begin();

  // ---- Queue + Parser Task ----
  packetQueue = xQueueCreate(50, sizeof(Packet));
  if (packetQueue == nullptr) {
    Serial.println("[FATAL] Không tạo được Queue!");
  }
  xTaskCreatePinnedToCore(
    parserTaskWorker, "ParserTask", 4096, NULL, 5, &parserTaskHandle, 1);

  // ---- Load config ----
  loadConfigFromNVS();

  // ---- Khởi động mode đã lưu ----
  Serial.printf("[BOOT] Khởi động mode: %d\n", (int)currentMode);
  switch (currentMode) {
    case MODE_WIFI:  setupWifiSTA(); break;
    case MODE_BLE:   setupBle();     break;
    case MODE_SETUP:
    default:
      currentMode = MODE_SETUP;
      setupSoftAP();
      break;
  }

  Serial.println("[BOOT] ✅ Hệ thống sẵn sàng.");
}

void loop() {
  // ============================================================
  // 0. KIỂM TRA NÚT BOOT
  // ============================================================
  checkBootButton();

  // ============================================================
  // 1. XỬ LÝ CHUYỂN MODE
  // ============================================================
  if (gPendingSwitch) {
    gPendingSwitch = false;
    DeviceMode target = gPendingMode;
    Serial.printf("[LOOP] Chuyển mode: %d\n", target);
    switchMode(target);
    return;
  }

  // ============================================================
  // 2. XỬ LÝ WI-FI (CHỈ KHI Ở MODE_WIFI)
  // ============================================================
  if (currentMode == MODE_WIFI) {
    // ---- Client cũ mất kết nối ----
    if (activeTcpClient && !activeTcpClient.connected()) {
      Serial.println("[WIFI] Client mất kết nối, dọn dẹp...");
      activeTcpClient.flush();
      while (activeTcpClient.available() > 0) activeTcpClient.read();
      activeTcpClient.stop();
      activeTcpClient = WiFiClient();
      clearAllHardwareStates();
      activeConnection.isConnected = false;
    }

    // ---- Client mới ----
    if (tcpServer != nullptr && tcpServer->hasClient()) {
      unsigned long now = millis();
      if (now - lastClientChangeTime < CLIENT_CHANGE_COOLDOWN_MS) {
        WiFiClient rejected = tcpServer->available();
        if (rejected) rejected.stop();
        Serial.println("[WIFI] Từ chối client mới (cooldown).");
      } else {
        WiFiClient newClient = tcpServer->available();
        if (!newClient || !newClient.connected()) {
          if (newClient) newClient.stop();
        } else {
          if (activeTcpClient && activeTcpClient.connected()) {
            activeTcpClient.flush();
            while (activeTcpClient.available() > 0) activeTcpClient.read();
            activeTcpClient.stop();
            Serial.println("[WIFI] Đá văng client cũ.");
          }
          clearAllHardwareStates();
          delay(30);

          activeTcpClient = newClient;
          activeTcpClient.setNoDelay(true);
          activeConnection.isConnected = true;
          lastClientChangeTime = now;
          Serial.println("[WIFI] ✅ Client mới.");
        }
      }
    }

    // ---- Đọc dữ liệu TCP ----
    if (activeTcpClient && activeTcpClient.connected()) {
      int avail = activeTcpClient.available();
      if (avail > 0) {
        uint8_t buffer[64];
        int toRead = min(avail, 64);
        int len = activeTcpClient.read(buffer, toRead);
        if (len > 0) {
          pushToQueue(buffer, len);
        }
      }
    }
  }

  // ============================================================
  // 3. WEB SERVER (CHỈ KHI Ở MODE_SETUP)
  // ============================================================
  if (currentMode == MODE_SETUP) {
    dnsServer.processNextRequest();
    webServer.handleClient();
  }

  delay(1);
}
