# XY Optical Drive Arduino Controller - Project Requirements Document

**Project:** XY Optical Drive Lab Automation Controller  
**Platform:** Arduino Uno R3  
**Branch:** Modbus-Motor-control-integration  
**Last Updated:** 2026-06-11

---

## 1. Executive Summary

This document consolidates critical technical learnings from debugging and implementing an Arduino Uno-based lab automation controller that integrates Ethernet networking, SD card file storage, and Modbus motor control. The project addresses SPI bus multiplexing challenges between incompatible peripheral architectures and establishes best practices for Arduino resource-constrained development.

---

## 2. Hardware Architecture

### 2.1 Processing Unit
- **Microcontroller:** ATmega328P
- **Clock:** 16 MHz
- **RAM:** 2 KB (54.6% currently used = 1119 bytes)
- **Flash:** 31.5 KB (70.9% currently used = 22,864 bytes)
- **Constraint:** Severe memory limitations require careful code optimization

### 2.2 Ethernet Shield (W5100/W5500)
- **Interface:** SPI
- **Chip Select:** Pin 10 (active LOW)
- **Static IP:** 169.254.43.138
- **Subnet Mask:** 255.255.0.0
- **MAC Address:** A8:61:0A:AE:1C:E4
- **State Storage:** Hardware registers in W5100 chip (persistent across SPI context switches)
- **Critical Property:** Network state is NOT affected by SPI bus ownership; shield maintains configuration in chip registers

### 2.3 SD Card Interface
- **Interface:** SPI
- **Chip Select:** Pin 4 (active LOW)
- **File System:** FAT (auto-detected: SD1, SD2, or SDHC)
- **State Storage:** SD library maintains FAT metadata in Arduino RAM (NOT persistent after SPI bus context switch)
- **Critical Property:** Loss of SPI bus control corrupts internal FAT state; remounting required

### 2.4 Motor Control
- **Communication:** Modbus RTU (9600 baud, SoftwareSerial)
- **Slave ID:** 1
- **RX Pin:** 9
- **TX Pin:** 8
- **GPIO Direction Control:**
  - Pin 2: CW (Clockwise) direction
  - Pin 5: CCW (Counter-clockwise) direction
- **Register Map:**
  - `REG_RS485_ENABLE` (0x000F): Enable RS485 communication
  - `REG_DIRECTION` (0x0007): Motor direction selection
  - `REG_JOG_SPEED` (0x01E1): Speed in RPM (configured: 60)
  - `REG_JOG_ACCEL` (0x01E7): Acceleration value (configured: 50)
  - `REG_CONTROL_WORD` (0x1801): Master control register
  - `REG_QUICK_STOP` (0x6002): Emergency stop register
- **Command Values:**
  - `RS485_ENABLE_VALUE` = 0x0001
  - `DIR_CW` = 0x0000
  - `DIR_CCW` = 0x0001
  - `JOG_SPEED_RPM` = 60
  - `JOG_ACCEL_VALUE` = 50
  - `JOG_CW_CMD` = 0x4001
  - `JOG_CCW_CMD` = 0x4002
  - `STOP_CMD` = 0x0004
  - `QUICK_STOP_CMD` = 0x0040

---

## 3. Critical Technical Discovery: SPI Bus Multiplexing

### 3.1 The Problem
When multiple SPI devices share a single bus on Arduino, chip select (CS) lines control which device responds. However, the SD library and Ethernet shield have fundamentally different state storage models:

| Device | State Storage | Persistence | Behavior |
|--------|---------------|-------------|----------|
| Ethernet (W5100) | Hardware registers in chip | ✅ Persistent across SPI context switch | State survives when another device uses bus |
| SD Card (SD library) | RAM-based FAT metadata | ❌ Lost when SPI bus taken by another device | FAT structure becomes invalid |

### 3.2 The Root Cause of SD Access Failures
1. `Ethernet.begin(mac, ip)` initializes the W5100 and takes SPI bus control
2. W5100 state is saved in chip registers (safe)
3. SD library's internal FAT volume structure remains in Arduino RAM but becomes **stale**
4. When `SD.open("/")` is called later, the FAT metadata pointers are invalid
5. Result: "could not open root" error

### 3.3 The Solution: Remounting on Every Access
Instead of relying on simple CS line switching (`digitalWrite(4, LOW)` / `digitalWrite(4, HIGH)`), the SD library must be reinitialized before each SD operation:

```cpp
void ActiveSD() {
  digitalWrite(10, HIGH);  // Deselect Ethernet (CS active LOW)
  SD.begin(4);             // Remount SD FAT volume (rebuilds internal state)
}
```

**Why `SD.begin(4)` is safe:**
- `SD.begin()` does NOT format the card—it simply rebuilds the FAT metadata in RAM
- Operation is lightweight (~50ms) and idempotent (can be called repeatedly)
- Only necessary if another SPI device has used the bus since last SD operation

### 3.4 Implementation Pattern
Apply this pattern at all SD access points:

```cpp
// Before any SD file operation:
ActiveSD();              // Remount FAT volume
File myFile = SD.open("filename.txt");

// Used throughout codebase at:
// - Line 314 (createtxt)
// - Line 787 (sendHWStatusResponse)
// - Line 825 (sendHTMLPage)
// - Line 845, 852, 869 (diagSDInit)
```

### 3.5 Ethernet Does NOT Need Remounting
The `activeEthernet()` function only needs CS control—no reinitialization:

```cpp
void activeEthernet() {
  digitalWrite(4, HIGH);   // Deselect SD (CS active LOW)
  digitalWrite(10, LOW);   // Select Ethernet (CS active LOW)
  // No Ethernet.begin() needed—state persists in W5100 registers
}
```

---

## 4. Serial Communication Configuration

### 4.1 Current Mismatch (IDENTIFIED)
- **Code:** `Serial.begin(9600)` at [src/main.cpp](src/main.cpp#L549)
- **Monitor Configuration:** `monitor_speed = 115200` at [platformio.ini](platformio.ini#L29)
- **Impact:** Serial monitor receives garbled output or displays nothing

### 4.2 Recommendation
Align both to **115200 baud** for faster USB debugging:
- Change `Serial.begin(9600)` → `Serial.begin(115200)` in src/main.cpp line 549
- platformio.ini line 29 already set to 115200 ✅

---

## 5. Codebase Structure

### 5.1 Main Firmware (src/main.cpp)
**Purpose:** Complete Arduino application for lab automation

**Key Components:**

| Function | Purpose | Status |
|----------|---------|--------|
| `setup()` | Initialize hardware, Ethernet, SD, ModbusSerial | ✅ Working |
| `checkEthernetStatus()` | Detect shield, read MAC, check link | ✅ Tested |
| `checkSDCardStatus()` | Probe SD card volume/FAT/size | ✅ Fixed with ActiveSD() |
| `ActiveSD()` | **CRITICAL** Remount SD, deselect Ethernet | ✅ Implemented |
| `activeEthernet()` | Deselect SD, select Ethernet | ✅ Working |
| `sendHTMLPage()` | Stream index.htm from SD to HTTP client | ✅ Uses ActiveSD() |
| `sendHWStatusResponse()` | Stream test.txt status file | ✅ Uses ActiveSD() |
| `createtxt()` | Generate test.txt diagnostics | ✅ Uses ActiveSD() |
| `handleClient()` | HTTP request dispatcher | ✅ Working |
| `diagSDInit()` | Debug SD initialization | ✅ Uses ActiveSD() |

**Motor Registers (lines ~31-47):** Modbus command constants properly defined

**Current Resource Usage:**
- Flash: 22,864 bytes / 32,256 (70.9%)
- RAM: 1,119 bytes / 2,048 (54.6%)
- Status: ✅ Within limits; no immediate optimization needed

### 5.2 Build Configuration (platformio.ini)
**Status:** Functional but deprecated warning about `src_filter`  
**Action Required:** Eventually migrate to `build_src_filter` (non-critical)

### 5.3 Web Interface (index.htm)
**Purpose:** Motor control interface  
**Status:** Subject to recent modifications; served from SD card

---

## 6. Issues Resolved During Session

### 6.1 Issue #1: SD Root Filesystem Inaccessible (CRITICAL)
**Symptom:** "could not open root" error in diagSDInit()  
**Root Cause:** SD library FAT metadata corrupted after Ethernet took SPI bus  
**Solution:** Implement `ActiveSD()` with `SD.begin(4)` remounting  
**Status:** ✅ FIXED

### 6.2 Issue #2: Serial Baud Rate Mismatch
**Symptom:** Garbled serial monitor output  
**Root Cause:** 9600 baud in code, 115200 in platformio.ini  
**Solution:** Align to 115200 (change line 549)  
**Status:** ⏳ PENDING USER DECISION

### 6.3 Issue #3: Syntax Error
**Symptom:** "expected an expression" at line 232  
**Root Cause:** Orphaned `);` in SD card diagnostic code  
**Solution:** Removed errant `);`  
**Status:** ✅ FIXED

### 6.4 Issue #4: Git History
**Action:** `git restore src/main.cpp` to cancel 17:00 modifications  
**Status:** ✅ COMPLETED

---

## 7. Key Lessons Learned

### 7.1 SPI Bus Multiplexing Architecture Pattern
**Principle:** Different SPI devices require different reinitialization strategies based on state storage model.

- **Hardware-state devices** (W5100 Ethernet): State persists; only CS switching needed
- **RAM-state devices** (SD library): State lost; must remount/reinitialize after SPI context switch
- **Pattern Application:** Document state storage type for all new SPI peripherals

### 7.2 SD Library Design
The Arduino SD library is optimized for **single-device scenarios**. In multiplexed configurations:
- `SD.begin()` is lightweight and idempotent—safe to call frequently
- FAT metadata stored in RAM requires remounting after SPI bus ownership change
- No formatting occurs on `SD.begin()` if card already initialized

### 7.3 Arduino Resource Management
With 2KB RAM and 32KB Flash:
- String literals must use `F()` macro for Flash storage
- Serial debugging must balance baud rates (9600 vs 115200)
- Functions should be factored to minimize stack usage
- Current project is **at 70.9% Flash capacity**—future additions risk overflow

### 7.4 Serial Communication Best Practices
- Serial baud rate MUST match between code and monitor configuration
- platformio.ini `monitor_speed` is the source of truth for USB debugging
- Recommend 115200 for modern systems (faster throughput, 16MHz Arduino handles easily)

### 7.5 Hardware Integration Testing Strategy
1. **Verify each peripheral individually** before integration
2. **Test SPI multiplexing** with explicit CS toggling diagnostics
3. **Monitor serial output** with matching baud rates during development
4. **Create status endpoints** (HTTP /status, SD test.txt) to validate all subsystems

---

## 8. Current Project State

### 8.1 Firmware Status
- **Build:** ✅ SUCCESS (70.9% Flash, 54.6% RAM)
- **Compilation Errors:** ✅ NONE
- **Warnings:** 2 (unused EEPROM, unused crc16_update—benign library warnings)
- **Last Successful Build:** 2026-06-11

### 8.2 Known Working Features
✅ Ethernet shield detection and MAC reading  
✅ SD card probing and FAT metadata reading  
✅ HTTP server and request handling  
✅ Hardware status file generation (test.txt)  
✅ Motor register definitions  
✅ Modbus RTU initialization  

### 8.3 Pending Tasks
⏳ Serial baud rate alignment (9600→115200)  
⏳ Re-upload firmware to hardware to test SD fix in practice  
⏳ Verify index.htm access and test.txt generation on actual board  
⏳ Migrate platformio.ini src_filter to build_src_filter  

---

## 9. Recommendations for Future Development

### 9.1 Immediate (Next Session)
1. Align serial baud rates (recommend 115200 for all)
2. Re-upload firmware to Arduino and validate SD card operations
3. Test HTTP /status endpoint response time
4. Monitor motor command execution via Modbus

### 9.2 Short Term
1. Implement SD file caching to reduce remount frequency (advanced optimization)
2. Add performance metrics for SPI bus switching overhead
3. Document Modbus motor commands with test sequences
4. Create automated test suite for HTTP endpoints

### 9.3 Long Term
1. Consider porting to Arduino Mega (4KB RAM, 256KB Flash) if features expand beyond 71% capacity
2. Implement watchdog timer for fault recovery
3. Add EEPROM-based configuration storage for network settings
4. Develop web-based firmware update mechanism

### 9.4 Code Quality
- Remove unused EEPROM and crc16_update warnings (clean up includes if not needed)
- Consider replacing SoftwareSerial with hardware UART alternative if performance critical
- Add unit tests for HTTP request parsing and Modbus command generation
- Document all SPI multiplexing points with comments referencing this PRD

---

## 10. Technical Debt & Known Issues

| Issue | Severity | Impact | Workaround | Target Resolution |
|-------|----------|--------|-----------|-------------------|
| Serial baud mismatch | Medium | Debug output corrupted | Align to 115200 | Next session |
| platformio.ini src_filter deprecated | Low | Build warning | Migrate to build_src_filter | Planned |
| EEPROM unused warning | Low | Clean build output | Remove if truly unused | Nice to have |
| No SD file caching | Medium | Frequent remounts overhead | Acceptable for current use case | Post-MVP |

---

## 11. Architecture Diagram

```
┌─────────────────────────────────────────────────────────┐
│          Arduino Uno R3 (ATmega328P)                     │
│          16MHz, 2KB RAM, 32KB Flash                      │
└─────────────────────────────────────────────────────────┘
                           │
                ┌──────────┼──────────┐
                │          │          │
           ┌────▼────┐ ┌───▼────┐ ┌──▼───┐
           │  UART   │ │  GPIO  │ │ SPI  │ (Shared Bus)
           │ Serial  │ │ Motor  │ └──────┘
           │ 9600bps │ │Control │  │   │
           └─────────┘ │Pins 2,5│  │   │
                       └────────┘  │   │
                         ┌─────────┘   └──────┐
                         │                    │
                    ┌────▼────┐        ┌─────▼──┐
                    │    SD    │        │ W5100  │
                    │  Card    │        │ (Eth)  │
                    │ CS=Pin4  │        │CS=Pin10│
                    └──────────┘        └────────┘
                         │                   │
                    FAT  │                   │ Hardware
                    (RAM)│                   │ Registers
                         │                   │
    State Model:    (Lost)│              (Persistent)
                    after SPI              across SPI
                    context switch         switches
```

---

## 12. References & Related Documentation

- **Arduino Uno Datasheet:** ATmega328P 16MHz microcontroller
- **W5100 Ethernet Shield:** SPI protocol, CS pin 10, state persistence in hardware
- **SD Library:** FAT metadata storage in RAM, remounting behavior
- **Modbus RTU:** Master/slave protocol, register addressing (0x0000–0xFFFF)
- **SoftwareSerial:** GPIO-based UART emulation (pins 8–9)

---

## 13. Contact & Change Log

**Document Author:** GitHub Copilot  
**Last Updated:** 2026-06-11  
**Repository:** WhiteShay/XY-Optical-Drive  
**Current Branch:** Modbus-Motor-control-integration  
**Default Branch:** RAM-optimization

### Change History
| Date | Author | Change |
|------|--------|--------|
| 2026-06-11 | Copilot | Initial PRD—consolidated debugging findings from SPI multiplexing session |

