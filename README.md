# ESP32-S3 Hybrid HID Core v4.1

> **Bàn phím + Chuột USB HID không dây** điều khiển qua **Bluetooth LE** hoặc **Wi-Fi TCP** từ điện thoại Android.

[![ESP32-S3](https://img.shields.io/badge/ESP32--S3-DevKitC--1-blue)]()
[![Arduino](https://img.shields.io/badge/Arduino-IDE%202.x-00979D)]()
[![Version](https://img.shields.io/badge/version-4.1-green)]()

---

## 📋 Mục lục

- [Tổng quan](#-tổng-quan)
- [Kiến trúc hệ thống](#-kiến-trúc-hệ-thống)
- [Mô hình 3 MODE](#-mô-hình-3-mode)
- [Sơ đồ luồng dữ liệu](#-sơ-đồ-luồng-dữ-liệu)
- [Cấu trúc thư mục](#-cấu-trúc-thư-mục)
- [Cài đặt](#-cài-đặt)
- [Sử dụng](#-sử-dụng)
- [Giao thức](#-giao-thức)
- [Troubleshooting](#-troubleshooting)
- [Thông số kỹ thuật](#-thông-số-kỹ-thuật)

---

## 🎯 Tổng quan

**ESP32-S3 Hybrid HID Core** biến board ESP32-S3 thành một **thiết bị HID USB** (bàn phím + chuột) có thể điều khiển từ xa qua điện thoại Android.

### Tính năng chính

| Tính năng | Mô tả |
|---|---|
| **USB HID** | Bàn phím + Chuột + Consumer Control (media keys) |
| **BLE GATT** | Kết nối không dây qua Bluetooth LE |
| **Wi-Fi TCP** | Kết nối qua mạng LAN, độ trễ thấp |
| **Web Config** | Cấu hình qua trình duyệt (không cần app) |
| **3 MODE tách biệt** | BLE, Wi-Fi STA, Setup — chỉ 1 mode chạy tại 1 thời điểm |
| **Nút BOOT dự phòng** | Giữ 3s → vào chế độ Setup |
| **Power Control** | Tắt nguồn / Sleep / Wake qua keyboard shortcut |

---

## 🏗 Kiến trúc hệ thống

### Nguyên tắc vàng

> **Chỉ 1 mode chạy tại 1 thời điểm. Không bao giờ chạy song song.**

```
┌──────────────────────────────────────────────────────────────────────┐
│                          ĐIỆN THOẠI ANDROID                         │
│                                                                      │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐         │
│  │  App BLE       │  │  App TCP       │  │  Web Browser   │         │
│  │  (GATT Client) │  │  (Socket)      │  │  (Chrome/Safari)│         │
│  └────────┬───────┘  └────────┬───────┘  └────────┬───────┘         │
└───────────┼────────────────────┼────────────────────┼────────────────┘
            │                    │                    │
            │ BLE                │ TCP                │ HTTP
            │                    │                    │
┌───────────▼────────────────────▼────────────────────▼────────────────┐
│                         ESP32-S3 DevKitC-1                           │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │                    HYBRID HID CORE                             │ │
│  │                                                                │ │
│  │  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐      │ │
│  │  │  MODE_BLE    │   │  MODE_WIFI   │   │  MODE_SETUP  │      │ │
│  │  │  (mode 0)    │   │  (mode 1)    │   │  (mode 2)    │      │ │
│  │  ├──────────────┤   ├──────────────┤   ├──────────────┤      │ │
│  │  │ BLE GATT     │   │ WiFi STA     │   │ SoftAP       │      │ │
│  │  │ - Write char │   │ + TCP Server │   │ + Web Server │      │ │
│  │  │ - Notify char│   │ - Port 1989  │   │ - 192.168.4.1│      │ │
│  │  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘      │ │
│  │         │                  │                  │              │ │
│  │         └──────────────────┼──────────────────┘              │ │
│  │                            │                                  │ │
│  │              ┌─────────────▼─────────────┐                   │ │
│  │              │   parseHidCommand()        │                   │ │
│  │              │   - V1: 3-byte frames      │                   │ │
│  │              │   - V2: TLV frames         │                   │ │
│  │              └─────────────┬─────────────┘                   │ │
│  │                            │                                  │ │
│  │         ┌──────────────────┼──────────────────┐              │ │
│  │         │                  │                  │              │ │
│  │    ┌────▼────┐      ┌──────▼──────┐    ┌─────▼─────┐        │ │
│  │    │Keyboard │      │   Mouse     │    │ Consumer  │        │ │
│  │    │ (HID)   │      │   (HID)     │    │ Control   │        │ │
│  │    └────┬────┘      └──────┬──────┘    └─────┬─────┘        │ │
│  │         │                  │                  │              │ │
│  │         └──────────────────┼──────────────────┘              │ │
│  │                            │                                  │ │
│  │                    ┌───────▼────────┐                        │ │
│  │                    │   USB-OTG      │                        │ │
│  │                    │  (TinyUSB)     │                        │ │
│  │                    └───────┬────────┘                        │ │
│  └────────────────────────────┼─────────────────────────────────┘ │
└───────────────────────────────┼─────────────────────────────────────┘
                                │
                                │ USB Cable
                                │
┌───────────────────────────────▼─────────────────────────────────────┐
│                         MÁY TÍNH (PC/Laptop)                        │
│                                                                     │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐        │
│  │  Bàn phím HID  │  │  Chuột HID     │  │ Consumer HID   │        │
│  │  (Keyboard)    │  │  (Mouse)       │  │ (Media Keys)   │        │
│  └────────────────┘  └────────────────┘  └────────────────┘        │
│                                                                     │
│  → Windows / macOS / Linux nhận như bàn phím + chuột thật          │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 🔄 Mô hình 3 MODE

### MODE_BLE (0) — Bluetooth

```
┌────────────────────────────────────────┐
│  MODE_BLE                              │
├────────────────────────────────────────┤
│  ✅ BLE GATT (Advertising)             │
│  ✅ USB HID                            │
│  ❌ Wi-Fi STA                          │
│  ❌ TCP Server                         │
│  ❌ SoftAP                             │
│  ❌ Web Server                         │
├────────────────────────────────────────┤
│  Dùng khi: Không cần mạng, điều khiển  │
│  từ xa qua Bluetooth                   │
└────────────────────────────────────────┘
```

### MODE_WIFI (1) — Wi-Fi STA + TCP

```
┌────────────────────────────────────────┐
│  MODE_WIFI                             │
├────────────────────────────────────────┤
│  ❌ BLE GATT                           │
│  ✅ USB HID                            │
│  ✅ Wi-Fi STA (kết nối router)         │
│  ✅ TCP Server (port 1989)             │
│  ❌ SoftAP                             │
│  ❌ Web Server                         │
├────────────────────────────────────────┤
│  Dùng khi: Điều khiển qua mạng LAN,   │
│  độ trễ thấp, ổn định hơn BLE          │
└────────────────────────────────────────┘
```

### MODE_SETUP (2) — Cấu hình qua Web

```
┌────────────────────────────────────────┐
│  MODE_SETUP                            │
├────────────────────────────────────────┤
│  ❌ BLE GATT                           │
│  ✅ USB HID (vẫn hoạt động)            │
│  ❌ Wi-Fi STA                          │
│  ❌ TCP Server                         │
│  ✅ SoftAP (ESP32S3_HID_Setup)         │
│  ✅ Web Server (192.168.4.1)           │
├────────────────────────────────────────┤
│  Dùng khi: Cấu hình Wi-Fi/BLE/TCP,    │
│  cứu hộ khi mất kết nối                │
└────────────────────────────────────────┘
```

### Sơ đồ chuyển đổi mode

```
                    ┌──────────────┐
                    │  MODE_SETUP  │
                    │   (mode 2)   │
                    └──────┬───────┘
                           │
         ┌─────────────────┼─────────────────┐
         │                 │                 │
    Bấm "Ngắt        Bấm "Chuyển      Bấm "Chuyển
    kết nối và       sang BLE"        sang WI-FI"
    thoát"           (tuỳ chọn)       (tuỳ chọn)
         │                 │                 │
         │ (tự chọn)       │                 │
         ▼                 ▼                 ▼
    ┌──────────┐      ┌──────────┐     ┌──────────┐
    │ Nếu có   │      │MODE_BLE  │     │MODE_WIFI │
    │ Wi-Fi?   │      │ (mode 0) │     │ (mode 1) │
    └────┬─────┘      └────┬─────┘     └────┬─────┘
         │                 │                 │
    ┌────┴────┐            │                 │
    │         │            │                 │
   Có       Không          │                 │
    │         │            │                 │
    ▼         ▼            │                 │
MODE_WIFI  MODE_BLE        │                 │
(mode 1)   (mode 0)        │                 │
                           │                 │
                           └─────────────────┘
                                    │
                        Bấm "Chuyển sang SETUP"
                        (tuỳ chọn) hoặc giữ BOOT 3s
                                    │
                                    ▼
                            ┌──────────────┐
                            │  MODE_SETUP  │
                            └──────────────┘
```

```
Lưu ý là khi vừa khởi động lên nếu lần trước đang ở chế độ wi-fi nhưng wi-fi mode bị tắt nó chưa tìm thấy thì nó sẽ quay trở về mode setup vậy thì chúng ta phải vào thẳng trực tiếp địa chỉ để kích hoạt lại chế độ tùy chọn mà mình muốn
```
---

## 📊 Sơ đồ luồng dữ liệu

### Luồng BLE (MODE_BLE)

```
┌─────────────┐
│ Android App │
│ (BLE Client)│
└──────┬──────┘
       │
       │ BLE GATT Write
       ▼
┌─────────────────────────┐
│ NimBLE Stack (ESP32-S3) │
│  onWrite() callback     │
└──────────┬──────────────┘
           │ pushToQueue()
           ▼
┌─────────────────────────┐
│ FreeRTOS Queue          │
│ (50 packets × 64 bytes) │
└──────────┬──────────────┘
           │ xQueueReceive()
           ▼
┌─────────────────────────┐
│ ParserTask (Core 1)     │
│  parseHidCommand()      │
└──────────┬──────────────┘
           │ Keyboard.pressRaw() / Mouse.move()
           ▼
┌─────────────────────────┐
│ USB HID (TinyUSB)       │
└──────────┬──────────────┘
           │ USB
           ▼
┌─────────────────────────┐
│ Máy tính (PC)           │
└─────────────────────────┘
```

### Luồng TCP (MODE_WIFI)

```
┌─────────────┐
│ Android App │
│ (TCP Client)│
└──────┬──────┘
       │ TCP Socket (Port 1989)
       ▼
┌─────────────────────────┐
│ WiFiServer (ESP32-S3)   │
│  hasClient()            │
└──────────┬──────────────┘
           │ activeTcpClient.read()
           ▼
┌─────────────────────────┐
│ loop() (Core 0)         │
│  parseHidCommand()      │  ← ZERO-COPY
└──────────┬──────────────┘
           │ Keyboard.pressRaw() / Mouse.move()
           ▼
┌─────────────────────────┐
│ USB HID (TinyUSB)       │
└──────────┬──────────────┘
           │ USB
           ▼
┌─────────────────────────┐
│ Máy tính (PC)           │
└─────────────────────────┘
```

### Luồng Web Config (MODE_SETUP)

```
┌─────────────────┐
│ Điện thoại      │
│ (Browser)       │
└────────┬────────┘
         │ Kết nối AP: ESP32S3_HID_Setup
         │ Mở http://192.168.4.1
         ▼
┌─────────────────────────┐
│ SoftAP (ESP32-S3)       │
│  IP: 192.168.4.1        │
└──────────┬──────────────┘
           │ HTTP Request
           ▼
┌─────────────────────────┐
│ WebServer (port 80)     │
│  handleRoot()           │
│  handleSaveWifi()       │
│  handleSaveBle()        │
│  handleSaveTcp()        │
│  handleApiExitSetup()   │
└──────────┬──────────────┘
           │ Lưu config vào NVS
           ▼
┌─────────────────────────┐
│ Preferences (NVS)       │
└─────────────────────────┘
```

---

## 📁 Cấu trúc thư mục

```
ESP32S3_Hybrid_HID_Core/
├── ESP32S3_Hybrid_HID_Core.ino    ← Firmware chính
├── data/
│   └── index.html                 ← Web config UI
└── README.md                      ← File này
```

### Cấu trúc firmware

```
ESP32S3_Hybrid_HID_Core.ino
├── 0.  Nút BOOT dự phòng
├── 1.  Cấu hình hệ thống
│   ├── 1b. Cờ chuyển mode
│   ├── 1c. Cờ quản lý socket
│   └── 1d. Web Server
├── 2.  Queue (FreeRTOS)
├── 3.  Mã giao thức (V1 + V2)
├── 4.  UUID BLE động
├── 5.  Bản đồ mã HID
├── 6.  Tầng driver HID
│   ├── 6b. Smart Key Tap
│   ├── 6c. Consumer Action
│   └── 6d. Power Control
├── 7.  Dọn sạch tài nguyên
├── 8.  Parser (parseHidCommand)
├── 9.  Queue (BLE)
├── 10. Phản hồi
├── 11. NVS Config
├── 12. Quản lý TCP Server
├── 13. Wi-Fi STA
├── 14. BLE
├── 15. Web Server (MODE_SETUP)
├── 16. Nút BOOT dự phòng
├── 17. Switch Mode
└── 18. Setup & Loop
```

---

## ⚙️ Cài đặt

### Phần cứng

| Linh kiện | Ghi chú |
|---|---|
| **ESP32-S3-DevKitC-1 N16R8** | 16MB Flash, 8MB PSRAM |
| **Cáp USB-C** | Có dây dữ liệu |
| **Máy tính** | Windows / macOS / Linux |

### Phần mềm

1. **Arduino IDE 2.x**
2. **ESP32 Board Package** (v3.x)
3. **NimBLE-Arduino** (v2.x)
4. **LittleFS Uploader** (earlephilhower)

### Cấu hình Arduino IDE

```
Tools → Board:              ESP32S3 Dev Module
Tools → USB Mode:           USB-OTG (TinyUSB)
Tools → USB CDC On Boot:    Enabled
Tools → Flash Size:         16MB (128Mb)
Tools → PSRAM:              OPI PSRAM
Tools → Partition Scheme:   Minimal SPIFFS (1.9MB APP)
Tools → Upload Speed:       921600
```

### Nạp firmware

1. Copy `ESP32S3_Hybrid_HID_Core.ino` vào Arduino IDE.
2. Nhấn **Verify** (Ctrl+R) — kiểm tra compile.
3. Nhấn **Upload** — nạp firmware.
4. **Ctrl+Shift+P** → `Upload LittleFS to Pico/ESP8266/ESP32` — nạp HTML.

---

## 🚀 Sử dụng

### Lần đầu sử dụng

```
1. Cấp nguồn board (cắm USB)
   → Board khởi động vào MODE_BLE (mặc định)

2. Mở app Android
   → Quét BLE → tìm "ESP32S3_HID_REMOTE"
   → Kết nối

3. Cấu hình Wi-Fi (nếu muốn dùng TCP)
   → Trong app: bấm "Cứu hộ" (gửi 0xE0)
   → Board chuyển MODE_SETUP
   → Kết nối AP "ESP32S3_HID_Setup"
   → Mở http://192.168.4.1
   → Nhập Wi-Fi + mật khẩu
   → Bấm "Lưu Wi-Fi"
   → Bấm "Ngắt kết nối và thoát"
   → Board chuyển MODE_WIFI

4. Cắm board vào máy tính
   → Máy tính nhận board như bàn phím + chuột
```

### Chuyển mode

| Từ mode | Sang mode | Cách |
|---|---|---|
| BLE | Setup | App gửi `0xE0` |
| BLE | Wi-Fi | App gửi `0xFE` |
| Wi-Fi | BLE | App gửi `0xFF` |
| Wi-Fi | Setup | App gửi `0xE0` |
| Setup | Wi-Fi | Web: "Ngắt kết nối và thoát" (nếu có Wi-Fi) |
| Setup | BLE | Web: "Ngắt kết nối và thoát" (nếu chưa có Wi-Fi) |
| Bất kỳ | Setup | Giữ nút BOOT 3s |

---

## 📡 Giao thức

### Frame V1 (3 bytes)

```
[CMD] [BYTE1] [BYTE2]

CMD_KEY (0x01):          BYTE1=modifiers, BYTE2=keycode
CMD_MOUSE_MOVE (0x02):   BYTE1=dx, BYTE2=dy
CMD_MOUSE_CLICK (0x03):  BYTE1=button
CMD_MOUSE_SCROLL (0x04): BYTE1=dx, BYTE2=dy
```

### Frame V2 (TLV — Type-Length-Value)

```
[0xAA] [0x01] [CMD] [LEN] [PAYLOAD...] [CMD] [LEN] [PAYLOAD...] ...

Keyboard:
  0x01 SET_MODIFIERS:  [mask]
  0x02 KEY_DOWN:       [keycode]
  0x03 KEY_UP:         [keycode]
  0x04 KEY_TAP:        [modifiers, keycode]

Mouse:
  0x10 MOUSE_MOVE:     [dx, dy]
  0x11 MOUSE_SCROLL:   [dx, dy]
  0x12 MOUSE_CLICK:    [button]
  0x13 MOUSE_DOWN:     [button]
  0x14 MOUSE_UP:       [button]

Consumer:
  0x20 CONSUMER_TAP:   [usage_hi, usage_lo]
  0x21 CONSUMER_DOWN:  [usage_hi, usage_lo]
  0x22 CONSUMER_UP:    [usage_hi, usage_lo]
```

### Command hệ thống

| Mã | Tên | Mô tả |
|---|---|---|
| `0xF1` | CMD_IDENTIFY_ROLE | `[role]` — 0x01=WiFi, 0x02=BLE |
| `0xF2` | REP_REQUIRE_AUTH | Yêu cầu mật khẩu |
| `0xF3` | CMD_AUTH_PASSWORD | `[len, password...]` |
| `0xF4` | REP_AUTH_SUCCESS | Xác thực thành công |
| `0xF5` | REP_AUTH_FAILED | Sai mật khẩu |
| `0xF6` | CMD_UPDATE_WIFI | `[ssidLen, ssid, passLen, pass]` |
| `0xF7` | CMD_PING | Kiểm tra kết nối |
| `0xF8` | REP_PONG | Phản hồi ping |
| `0xF9` | REP_SWITCH_OK | Xác nhận chuyển mode |
| `0xFA` | REP_WIFI_IP | `[4 bytes IP]` |
| `0xFB` | REP_BLE_NAME | `[len, name...]` |
| `0xFC` | REP_TCP_PORT | `[port_hi, port_lo]` |
| `0xFD` | CMD_GET_CONFIG | Yêu cầu config |
| `0xFE` | CMD_SWITCH_TO_WIFI | Chuyển sang Wi-Fi |
| `0xFF` | CMD_SWITCH_TO_BLE | Chuyển sang BLE |
| `0xE0` | CMD_SWITCH_TO_SETUP | Chuyển sang Setup |

---

## 🔧 Troubleshooting

### Board không nhận diện USB HID

| Nguyên nhân | Cách sửa |
|---|---|
| Cáp USB chỉ sạc | Dùng cáp có dây data |
| Cắm sai cổng USB | Cắm vào cổng **USB-OTG** (native) |
| USB Mode sai | Tools → USB Mode → USB-OTG (TinyUSB) |
| USB CDC On Boot | Tools → USB CDC On Boot → Enabled |

### Không kết nối được BLE

| Nguyên nhân | Cách sửa |
|---|---|
| Board ở MODE_WIFI | Chuyển sang MODE_BLE |
| BLE name sai | Kiểm tra `cfg_ble_name` trong NVS |
| Bluetooth tắt | Bật Bluetooth trên điện thoại |

### TCP kết nối thất bại

| Nguyên nhân | Cách sửa |
|---|---|
| Board chưa kết nối Wi-Fi | Kiểm tra Serial Monitor |
| Sai IP/Port | Kiểm tra IP trên Serial Monitor |
| Tường lửa | Tắt tường lửa trên router |

### Web config không mở được

| Nguyên nhân | Cách sửa |
|---|---|
| Chưa kết nối AP | Kết nối `ESP32S3_HID_Setup` |
| Chưa upload LittleFS | Ctrl+Shift+P → Upload LittleFS |
| Sai địa chỉ | Dùng `http://192.168.4.1` (không HTTPS) |

### Nút BOOT không hoạt động

| Nguyên nhân | Cách sửa |
|---|---|
| Giữ không đủ 3s | Giữ đúng **3 giây** |
| Nút bị hỏng | Kiểm tra nút BOOT trên board |

---

## 📊 Thông số kỹ thuật

| Thông số | Giá trị |
|---|---|
| **Chip** | ESP32-S3 (LX7 dual-core, 240MHz) |
| **Flash** | 16MB |
| **PSRAM** | 8MB |
| **RAM** | 512KB SRAM + 16KB RTC |
| **USB** | USB-OTG (TinyUSB) |
| **BLE** | Bluetooth 5 LE |
| **Wi-Fi** | 2.4GHz 802.11 b/g/n |
| **GPIO** | 45 chân |
| **Điện áp** | 3.0V - 3.6V |

---

## 📝 License

...
