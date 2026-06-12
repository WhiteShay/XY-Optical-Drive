/*
 * XY_Optical_Drive_V1_0_3.ino
 * * DESCRIPTION:
 * Reliable Lab Automation Controller for Arduino Uno + Ethernet Shield + Stepper Motors NEMA 17.
 *  * * HARDWARE STACK:
 * 1. Arduino Uno R3 (Bottom)
 * 2. Ethernet Shield W5100/W5500 (Middle) - Uses SPI (ICSP Header + Pin 10), SD CS = Pin 4
 * 3. Integrated Stepper Motor Drivers connected via Modbus RS485 (TTL->RS485 module P7156-1).
 *
 * * ================= REST API REFERENCE =================
 * * GET /status   : JSON (uptime, cached feedback position, Modbus diagnostics, motor state)
 * * GET /cw, /ccw : latch the JOG in that direction. The firmware then refreshes
 * *                 the drive command every 30 ms until /stop is received.
 * * GET /stop     : immediate quick stop (0x6002 = 0x0040)
 * * GET /hwstatus : content of test.txt (hardware report written at boot)
 * * GET /         : serves index.htm from the SD card
 * * ======================================================
 *
 * * ================= COMMAND MODEL (V1.0.3) =================
 * * Simple latch: one /cw or /ccw starts the JOG, /stop ends it. No watchdog,
 * * no keep-alive (removed on request): if a /stop request is lost, the motor
 * * keeps running until any /stop arrives - use the Stop button as recovery.
 * * Communication priority (highest first):
 * *   1) Modbus JOG command to the motor (loop() services it FIRST, and the
 * *      file-serving loops interleave it between 64-byte chunks).
 * *   2) HTTP requests (page/status updates may be slowed down, never the JOG).
 * *   3) Feedback position read: ONLY while the motor is stopped, 1 Hz poll.
 * * ======================================================
 *
 * * HISTORY / UPDATES:
 * * 03/03/2026 ==> 10/04/2026 - V0.1.0 - Initial development: Eth+SD checks,
 * *   test.txt, web server, HTML on SD, ModbusMaster integration, JOG CW/CCW.
 * * 11/06/2026 - V1.0.1 - SD / SPI bus fix: removed ActiveSD()/activeEthernet(),
 * *   single SD.begin(4) at setup, low-level diagnostics before the FAT mount,
 * *   64-byte chunk serving, removed dead verifyAndClearTxtFile().
 * * 12/06/2026 - V1.0.2 - JOG command rework: keep-alive watchdog + Modbus
 * *   priority + position read at standstill only. Field test showed E2
 * *   (response timeout) errors: each one froze the sketch for 2 s
 * *   (ModbusMaster's hard-coded 2000 ms timeout), which starved the
 * *   keep-alives and made the watchdog stop the motor in a loop
 * *   ("JOG watchdog: keep-alive lost"), and delayed the jog start by 2-3 s.
 *
 * 12/06/2026 - V1.0.3 - Watchdog removed + Modbus link rework
 * *  - REMOVED the keep-alive watchdog (on request). /cw and /ccw simply
 * *    latch the JOG until /stop. CAUTION: a lost /stop now leaves the motor
 * *    running until the next /stop (Stop button = recovery).
 * *  - REPLACED the ModbusMaster library with a built-in minimal Modbus RTU
 * *    master (FC03 read / FC06 write). Reasons and gains:
 * *      . Response timeout 80 ms instead of the library's hard-coded
 * *        2000 ms: a lost response now costs 80 ms, not a 2 s freeze of the
 * *        whole controller (web server included).
 * *      . RX buffer purged before every request (stale/garbled bytes from a
 * *        previous failed exchange can no longer corrupt the next one).
 * *      . Enforced 3.5-character inter-frame silence (4 ms @ 9600 baud), as
 * *        required by Modbus RTU. The previous 20 ms refresh sent frames
 * *        back-to-back with ~0 bus idle time, which can make the drive drop
 * *        requests (the iCL-RS needs 0.6 ms to re-arm its receiver).
 * *      . Modbus exception frames (FC|0x80) decoded and reported.
 * *      . "modbus_timeouts" counter exposed in /status for diagnostics:
 * *        it should stay near 0; if it keeps climbing, check wiring,
 * *        termination (SW8), common GND, and Pr5.24 = 4 (8N1).
 * *  - JOG refresh period 20 -> 30 ms: still well under the 50 ms required
 * *    by the drive, and guarantees idle time between frames.
 * *  - Removed unused EEPROM.h include. ModbusMaster library no longer needed.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <SoftwareSerial.h>

// ================= NETWORK CONFIGURATION =================
//                    MAC Address:
// Specific to your shield. If you change shields, update this.
byte mac[6] = { 0xA8, 0x61, 0x0A, 0xAE, 0x1C, 0xE4 };

//                    IP Configuration:
// Currently set to my PC ethernet settings (PC Ip : 169.254.43.137, local).
// If connecting directly to PC without router, change to 192.168.1.177 or match your PC's Autoconfig IP.
IPAddress ip(169, 254, 43, 138);
// IPAddress gateway(169, 254, 43, 1);
IPAddress subnet(255, 255, 0 , 0);
// IPAddress myDns(8, 8, 8, 8); // Not strictly used for local control

// ================= WEB SERVER =================
EthernetServer server(80);
bool sdReady = false; // Set to true after the single successful SD.begin() during setup

// ================= MODBUS RTU MASTER (built-in, replaces ModbusMaster lib) =================
// SoftwareSerial for RS485 Modbus: RX = D8 (<- RXD(O) of the P7156-1), TX = D9 (-> TXD(I)).
// The P7156-1 module has automatic direction control: no DE/RE handling needed.
SoftwareSerial modbusSerial(8, 9);  // (rxPin, txPin)

static const uint8_t  MB_SLAVE_ID            = 1;
static const uint8_t  MB_OK                  = 0x00;
static const uint8_t  MB_ERR_TIMEOUT         = 0xE2;  // same code as ModbusMaster, logs stay comparable
static const uint16_t MB_RESPONSE_TIMEOUT_MS = 80;    // worst case @ 9600 baud is ~25 ms (manual 4.1)
static const uint16_t MB_INTERFRAME_US       = 4000;  // >= 3.5 characters @ 9600 baud (3.65 ms)

uint32_t mbLastActivityUs = 0;   // end of last TX/RX byte, for inter-frame silence
uint16_t mbTimeouts = 0;         // lost/garbled responses since boot (diagnostics)

uint16_t mbCRC16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else              { crc >>= 1; }
    }
  }
  return crc;  // sent low byte first on the wire
}

// One blocking request/response exchange. frame[] holds the PDU without CRC
// (txLen bytes, 2 spare bytes required for the CRC). Returns MB_OK, a Modbus
// exception code (1,2,3,8), or MB_ERR_TIMEOUT on no/garbled response.
// Worst case duration: TX (~txLen ms) + MB_RESPONSE_TIMEOUT_MS. No 2 s freeze.
uint8_t mbTransaction(uint8_t *frame, uint8_t txLen, uint8_t *resp, uint8_t expectedLen) {
  uint16_t crc = mbCRC16(frame, txLen);
  frame[txLen++] = (uint8_t)(crc & 0xFF);
  frame[txLen++] = (uint8_t)(crc >> 8);

  // Purge stale RX bytes (leftovers of a previous failed exchange) so they
  // cannot be mistaken for the beginning of this response.
  while (modbusSerial.available()) modbusSerial.read();

  // Modbus RTU inter-frame silence: 3.5 characters of bus idle before TX.
  // The iCL-RS also needs 0.6 ms to re-arm its receiver after responding.
  while ((uint32_t)(micros() - mbLastActivityUs) < (uint32_t)MB_INTERFRAME_US) { }

  modbusSerial.write(frame, txLen);   // blocking, ~1.04 ms per byte @ 9600
  mbLastActivityUs = micros();

  uint8_t got = 0;
  uint32_t start = millis();
  while (got < expectedLen) {
    if (modbusSerial.available()) {
      resp[got++] = (uint8_t)modbusSerial.read();
      mbLastActivityUs = micros();
      if (got == 2 && (resp[1] & 0x80)) {
        expectedLen = 5;  // Modbus exception frame: id, fc|0x80, code, crc(2)
      }
    } else if (millis() - start > MB_RESPONSE_TIMEOUT_MS) {
      mbTimeouts++;
      return MB_ERR_TIMEOUT;
    }
  }

  uint16_t rcrc = mbCRC16(resp, got - 2);
  if (resp[0] != MB_SLAVE_ID ||
      resp[got - 2] != (uint8_t)(rcrc & 0xFF) ||
      resp[got - 1] != (uint8_t)(rcrc >> 8)) {
    mbTimeouts++;
    return MB_ERR_TIMEOUT;
  }
  if (resp[1] & 0x80) return resp[2];  // drive reported a Modbus exception
  return MB_OK;
}

// FC06 - Preset Single Register. Response echoes the request (8 bytes).
uint8_t mbWriteSingleRegister(uint16_t reg, uint16_t value) {
  uint8_t f[8];
  uint8_t r[8];
  f[0] = MB_SLAVE_ID;
  f[1] = 0x06;
  f[2] = (uint8_t)(reg >> 8);
  f[3] = (uint8_t)(reg & 0xFF);
  f[4] = (uint8_t)(value >> 8);
  f[5] = (uint8_t)(value & 0xFF);
  return mbTransaction(f, 6, r, 8);
}

// FC03 - Read Holding Registers (max 4 here). out[] receives 16-bit values.
uint8_t mbReadHoldingRegisters(uint16_t reg, uint8_t count, uint16_t *out) {
  uint8_t f[8];
  uint8_t r[5 + 2 * 4];
  f[0] = MB_SLAVE_ID;
  f[1] = 0x03;
  f[2] = (uint8_t)(reg >> 8);
  f[3] = (uint8_t)(reg & 0xFF);
  f[4] = 0;
  f[5] = count;
  uint8_t expected = 5 + 2 * count;  // id, fc, byte count, data, crc(2)
  uint8_t rc = mbTransaction(f, 6, r, expected);
  if (rc != MB_OK) return rc;
  if (r[1] != 0x03 || r[2] != 2 * count) {
    mbTimeouts++;
    return MB_ERR_TIMEOUT;
  }
  for (uint8_t i = 0; i < count; i++) {
    out[i] = ((uint16_t)r[3 + 2 * i] << 8) | r[4 + 2 * i];
  }
  return MB_OK;
}

// ================= MODBUS MOTOR COMMAND REGISTERS =================
// Register map from the motor drive manual.
static const uint16_t REG_RS485_ENABLE = 0x000F;  // Pr0.07: 1 = control by RS485
static const uint16_t REG_DIRECTION    = 0x0007;  // 0 = CW, 1 = CCW
static const uint16_t REG_JOG_SPEED    = 0x01E1;  // Pr6.00: JOG velocity
static const uint16_t REG_JOG_ACCEL    = 0x01E7;  // Pr6.03: JOG accel/decel
static const uint16_t REG_CONTROL_WORD = 0x1801;  // JOG command/control word
static const uint16_t REG_QUICK_STOP   = 0x6002;  // JOG quick stop register
static const uint16_t REG_FEEDBACK_POS_H = 0x1014;  // Feedback position (high 16 bits), pulses
static const uint16_t REG_FEEDBACK_POS_L = 0x1015;  // Feedback position (low 16 bits), pulses

// Common values written to the command registers.
static const uint16_t RS485_ENABLE_VALUE = 0x0001;
static const uint16_t DIR_CW             = 0x0000;
static const uint16_t DIR_CCW            = 0x0001;
static const uint16_t JOG_SPEED_RPM      = 100;
static const uint16_t JOG_ACCEL_VALUE    = 50;
static const uint16_t JOG_CW_CMD         = 0x4001;
static const uint16_t JOG_CCW_CMD        = 0x4002;
static const uint16_t QUICK_STOP_CMD     = 0x0040;

// Last Modbus result and cached feedback position (iCL-RS manual 4.3.4).
int32_t feedbackPositionPulses = 0;
bool feedbackPositionValid = false;
uint8_t lastModbusResult = 0;
uint32_t lastModbusMs = 0;

// ================= JOG STATE / TIMING =================
enum MotorState { MOTOR_STOPPED, MOTOR_CW, MOTOR_CCW };

static const uint32_t JOG_REFRESH_MS   = 30;    // < 50 ms required by the drive; a full
                                                // transaction lasts ~21 ms, so ~9 ms of
                                                // bus idle remains between frames
static const uint32_t POSITION_POLL_MS = 1000;  // feedback position poll period
                                                // (motor stopped ONLY)

MotorState motorState = MOTOR_STOPPED;
uint32_t lastJogRefresh = 0;     // last 0x4001/0x4002 write to the drive
uint32_t lastPositionPoll = 0;   // last background position read
bool jogConfigured = false;      // JOG speed/accel written once, not on every press

// ================= HELPER STRUCTURES =================
// Status result structures (small, minimal RAM) for logging to SD file
struct EthernetStatus {
  bool detected;
  int hwStatus;
  bool linkOn;
};

struct SDCardStatus {
  bool detected;
  bool initialized;
  uint8_t cardType; // 1=SD1, 2=SD2, 3=SDHC, 0=Unknown
  int fatType;
  uint32_t volumeKB;
};

// ================= HELPER FUNCTIONS =================
//                   Function Declarations:
EthernetStatus checkEthernetStatus(Print &out);
SDCardStatus checkSDCardStatus(Print &out);
void createtxt(const EthernetStatus &eth, const SDCardStatus &sd);
void printIPAddress(Print &out, const IPAddress &address);

void commandJog(MotorState dir);
void stopMotor();
void serviceMotorJog();
void servicePositionPoll();

void handleClient(EthernetClient client);
void sendStatusResponse(EthernetClient client);
void sendHWStatusResponse(EthernetClient client);
void sendHTMLPage(EthernetClient client);

EthernetStatus checkEthernetStatus(Print &out) {
  EthernetStatus status = {};

  out.println(F("\n--- Ethernet Communication Status ---"));

  delay(100); // Delay to allow hardware to stabilize before checking status
  Ethernet.begin(mac, ip, subnet); // Initialize Ethernet with manually defined MAC address and IP
                                   // (signature left as is: IP issues intentionally ignored here)

  // Check if Ethernet shield is present
  status.hwStatus = Ethernet.hardwareStatus();
  if (status.hwStatus == EthernetNoHardware) {
    out.print(F("Ethernet.hardwareStatus() = "));
    out.println(status.hwStatus);

    out.println(F("Ethernet shield: NOT DETECTED"));
    out.println(F("No Ethernet hardware found."));
    status.detected = false;
    status.linkOn = false;
  } else {
    status.detected = true;
    out.println(F("Ethernet shield: DETECTED"));

    out.print(F("MAC Address: "));
    for (int i = 0; i < 6; i++) {
      if (mac[i] < 16) out.print(F("0"));
      out.print(mac[i], HEX);
      if (i < 5) out.print(F(":"));
    }
    out.println();

    status.linkOn = (Ethernet.linkStatus() == LinkON);
    if (status.linkOn) {
      out.println(F("Ethernet link: CONNECTED"));
      out.println(F("Ethernet communication: READY"));
    } else {
      out.println(F("Ethernet link: DISCONNECTED"));
      out.println(F("Ethernet communication: NOT AVAILABLE"));
    }
  }

  return status;
}

SDCardStatus checkSDCardStatus(Print &out) {
  SDCardStatus status = {};

  out.println(F("\n--- SD Card Status ---"));

  // 1) Low-level diagnostics BEFORE the FAT mount.
  //    card.init() re-initializes the card: it must NEVER be called after
  //    SD.begin(), otherwise the SD library's internal state gets
  //    invalidated underneath it.
  Sd2Card card;
  SdVolume volume;

  if (card.init(SPI_HALF_SPEED, 4)) {
    status.detected = true;
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        status.cardType = 1;
        out.println(F("Card type: SD1"));
        break;
      case SD_CARD_TYPE_SD2:
        status.cardType = 2;
        out.println(F("Card type: SD2"));
        break;
      case SD_CARD_TYPE_SDHC:
        status.cardType = 3;
        out.println(F("Card type: SDHC"));
        break;
      default:
        status.cardType = 0;
        out.println(F("Card type: Unknown"));
    }

    if (volume.init(card)) {
      status.fatType = volume.fatType();
      out.print(F("Volume type: FAT"));
      out.println(status.fatType, DEC);

      uint32_t volumesize = volume.blocksPerCluster();
      volumesize *= volume.clusterCount();
      volumesize /= 2;  // SD card blocks are always 512 bytes (1/2 KB)
      status.volumeKB = volumesize;
      out.print(F("Volume size: "));
      out.print(status.volumeKB);
      out.println(F(" KB"));
    }
  } else {
    out.println(F("SD card: NOT DETECTED"));
    out.println(F("Possible reasons:"));
    out.println(F("- No SD card inserted"));
    out.println(F("- SD card not formatted properly"));
    out.println(F("- Wrong CS pin (currently using pin 4)"));
  }

  // 2) FAT mount: this is the ONE AND ONLY SD.begin() in the whole program.
  //    From here on, the SD and Ethernet libraries share the SPI bus by each
  //    driving their own CS on every transaction: no manual bus management.
  if (status.detected && SD.begin(4)) {
    status.initialized = true;
    sdReady = true;
    out.println(F("SD card: DETECTED and INITIALIZED"));
  } else if (status.detected) {
    status.initialized = false;
    out.println(F("SD card: FAILED TO INITIALIZE (FAT mount)"));
  }

  return status;
}

void printIPAddress(Print &out, const IPAddress &address) {
  for (uint8_t index = 0; index < 4; index++) {
    out.print(address[index], DEC);
    if (index < 3) {
      out.print(F("."));
    }
  }
}

void writeHardwareStatusToFile(File &file, const EthernetStatus &eth, const SDCardStatus &sd) {
  // Write hardware setup summary to the SD file (called after setup checks).
  file.println(F("\n--- Ethernet Communication Status ---"));
  if (!eth.detected) {
    file.print(F("Ethernet.hardwareStatus() = "));
    file.println(eth.hwStatus);
    file.println(F("Ethernet shield: NOT DETECTED"));
    file.println(F("No Ethernet hardware found."));
  } else {
    file.println(F("Ethernet shield: DETECTED"));

    file.print(F("MAC Address: "));
    for (int i = 0; i < 6; i++) {
      if (mac[i] < 16) file.print(F("0"));
      file.print(mac[i], HEX);
      if (i < 5) file.print(F(":"));
    }
    file.println();

    if (eth.linkOn) {
      file.println(F("Ethernet link: CONNECTED"));
      file.println(F("Ethernet communication: READY"));
      file.print(F("IP address: "));
      printIPAddress(file, ip);
      file.println();
    } else {
      file.println(F("Ethernet link: DISCONNECTED"));
      file.println(F("Ethernet communication: NOT AVAILABLE"));
    }
  }

  file.println(F("\n--- SD Card Status ---"));
  if (!sd.detected) {
    file.println(F("SD card: NOT DETECTED or FAILED TO INITIALIZE"));
    file.println(F("Possible reasons:"));
    file.println(F("- No SD card inserted"));
    file.println(F("- SD card not formatted properly"));
    file.println(F("- Wrong CS pin (currently using pin 4)"));
  } else {
    file.println(F("SD card: DETECTED and INITIALIZED"));
    switch (sd.cardType) {
      case 1:
        file.println(F("Card type: SD1"));
        break;
      case 2:
        file.println(F("Card type: SD2"));
        break;
      case 3:
        file.println(F("Card type: SDHC"));
        break;
      default:
        file.println(F("Card type: Unknown"));
        break;
    }

    file.print(F("Volume type: FAT"));
    file.println(sd.fatType, DEC);
    file.print(F("Volume size: "));
    file.print(sd.volumeKB);
    file.println(F(" KB"));
  }
}

void createtxt(const EthernetStatus &eth, const SDCardStatus &sd) {
  Serial.println(F("\n--- SD Card Create .txt ---"));

  // The volume is already mounted (single SD.begin() at setup): just check the flag.
  if (!sdReady) {
    Serial.println(F("Cannot create file - SD not initialized"));
    return;
  }

  // Overwrite existing file so status is fresh each run.
  // (FILE_WRITE = append: the file MUST be removed to start from scratch.)
  SD.remove("test.txt");

  File file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to open test.txt for writing"));
    return;
  }

  file.print(F("Timestamp (uptime): "));
  file.print(millis());
  file.println(F(" ms"));
  file.println();

  writeHardwareStatusToFile(file, eth, sd);

  file.close();

  Serial.println(F("test.txt created/updated successfully"));
}

// ================= MOTOR CONTROL =================
void recordModbusResult(uint8_t result) {
  lastModbusResult = result;
  lastModbusMs = millis();
}

bool readFeedbackPosition() {
  uint16_t regs[2];
  uint8_t result = mbReadHoldingRegisters(REG_FEEDBACK_POS_H, 2, regs);
  recordModbusResult(result);
  if (result != MB_OK) {
    feedbackPositionValid = false;
    Serial.print(F("Feedback position read error: "));
    Serial.println(result, HEX);
    return false;
  }
  feedbackPositionPulses = (int32_t)(((uint32_t)regs[0] << 16) | (uint32_t)regs[1]);
  feedbackPositionValid = true;
  return true;
}

void enableMotorRS485() {
  // Enable RS485 control: REG_RS485_ENABLE (0x000F) = 0x0001
  uint8_t result = mbWriteSingleRegister(REG_RS485_ENABLE, RS485_ENABLE_VALUE);
  recordModbusResult(result);
  if (result != MB_OK) {
    Serial.print(F("RS485 Enable Error: "));
    Serial.println(result, HEX);
  }
}

const char *motorStateText() {
  switch (motorState) {
    case MOTOR_CW:  return "cw";
    case MOTOR_CCW: return "ccw";
    default:        return "stopped";
  }
}

bool setupMotorJog() {
  enableMotorRS485();
  uint8_t r1 = mbWriteSingleRegister(REG_JOG_SPEED, JOG_SPEED_RPM);
  uint8_t r2 = mbWriteSingleRegister(REG_JOG_ACCEL, JOG_ACCEL_VALUE);
  recordModbusResult(r2);
  return (r1 == MB_OK) && (r2 == MB_OK);
  // Note: no delay() needed between writes anymore - mbTransaction() enforces
  // the 3.5-character inter-frame silence by itself.
}

// Latch the JOG direction. Called once per button press (/cw or /ccw).
void commandJog(MotorState dir) {
  if (!jogConfigured) {
    jogConfigured = setupMotorJog();
  }
  if (motorState != dir) {
    motorState = dir;
    Serial.println(dir == MOTOR_CW ? F("Jog CW") : F("Jog CCW"));
  }
  lastJogRefresh = millis();
  uint8_t result = mbWriteSingleRegister(REG_CONTROL_WORD,
      (dir == MOTOR_CW) ? JOG_CW_CMD : JOG_CCW_CMD);
  recordModbusResult(result);
}

void stopMotor() {
  motorState = MOTOR_STOPPED;
  uint8_t result = mbWriteSingleRegister(REG_QUICK_STOP, QUICK_STOP_CMD);
  recordModbusResult(result);
}

// HIGHEST PRIORITY task. Called first in loop() and between every 64-byte
// chunk while serving files, so the < 50 ms drive refresh is never starved.
// A lost response now costs at most 80 ms (one missed refresh, the drive
// pauses, the next successful refresh restarts it) instead of a 2 s freeze.
void serviceMotorJog() {
  if (motorState == MOTOR_STOPPED) return;
  uint32_t now = millis();
  if (now - lastJogRefresh < JOG_REFRESH_MS) return;
  lastJogRefresh = now;
  uint8_t result = mbWriteSingleRegister(REG_CONTROL_WORD,
      (motorState == MOTOR_CW) ? JOG_CW_CMD : JOG_CCW_CMD);
  recordModbusResult(result);
}

// LOWEST PRIORITY task. Feedback position is read ONLY while the motor is
// stopped (1 Hz), so a blocking Modbus read can never delay a JOG command.
void servicePositionPoll() {
  if (motorState != MOTOR_STOPPED) return;
  uint32_t now = millis();
  if (now - lastPositionPoll < POSITION_POLL_MS) return;
  lastPositionPoll = now;
  readFeedbackPosition();
}

void sendMotorOkResponse(EthernetClient client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println();
  client.print(F("{\"motor_state\":\""));
  client.print(motorStateText());
  client.print(F("\"}"));
  client.println();
}

// ================= WEB SERVER FUNCTIONS =================
void handleClient(EthernetClient client) {
  char request[48] = {0}; // Fixed buffer for HTTP request line
  size_t index = 0;
  while (client.available() && index < sizeof(request) - 1) {
    char c = client.read();
    request[index++] = c;
    if (c == '\n') break;
  }
  request[index] = '\0'; // Null terminate

  if (strstr(request, "GET /status ")) {
    sendStatusResponse(client);
  } else if (strstr(request, "GET /cw ")) {
    commandJog(MOTOR_CW);      // latches the JOG until /stop
    sendMotorOkResponse(client);
  } else if (strstr(request, "GET /ccw ")) {
    commandJog(MOTOR_CCW);     // latches the JOG until /stop
    sendMotorOkResponse(client);
  } else if (strstr(request, "GET /stop ")) {
    stopMotor();
    Serial.println(F("Motor stopped"));
    readFeedbackPosition();    // motor is stopped: immediate position read is allowed
    sendMotorOkResponse(client);
  } else if (strstr(request, "GET /hwstatus ")) {
    sendHWStatusResponse(client);
  } else if (strstr(request, "GET / ")) {
    sendHTMLPage(client);
  } else {
    client.println(F("HTTP/1.1 404 Not Found"));
    client.println();
  }
  client.stop();
}

void sendStatusResponse(EthernetClient client) {
  // No Modbus access here: position is cached, refreshed by the 1 Hz
  // background poll (motor stopped only) and on /stop.
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println();
  client.print(F("{\"uptime\":"));
  client.print(millis());
  client.print(F(",\"feedback_position\":"));
  client.print(feedbackPositionPulses);
  client.print(F(",\"feedback_position_valid\":"));
  client.print(feedbackPositionValid ? F("true") : F("false"));
  client.print(F(",\"last_modbus_result\":"));
  client.print(lastModbusResult);
  client.print(F(",\"last_modbus_ms\":"));
  client.print(lastModbusMs);
  client.print(F(",\"modbus_timeouts\":"));
  client.print(mbTimeouts);
  client.print(F(",\"motor_state\":\""));
  client.print(motorStateText());
  client.print(F("\""));
  client.println(F("}"));
}

void sendHWStatusResponse(EthernetClient client) {
  if (!sdReady) {
    client.println(F("HTTP/1.1 503 Service Unavailable"));
    client.println(F("Content-Type: text/plain"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("SD not available"));
    return;
  }

  // Open the file BEFORE sending the headers: allows returning a proper 404.
  File txt = SD.open("test.txt", FILE_READ);
  if (!txt) {
    client.println(F("HTTP/1.1 404 Not Found"));
    client.println(F("Content-Type: text/plain"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("test.txt not found"));
    return;
  }

  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/plain"));
  client.println(F("Connection: close"));
  client.println();

  uint8_t buf[64];
  int n;
  while ((n = txt.read(buf, sizeof(buf))) > 0) {
    client.write(buf, n);
    serviceMotorJog();   // PRIORITY: keep feeding the drive while serving
  }
  txt.close();
}

// Serve index.htm from SD card.
// Copy Web/index.html to the SD card as index.htm (SD library uses 8.3 filenames).
void sendHTMLPage(EthernetClient client) {
  if (!sdReady) {
    client.println(F("HTTP/1.1 503 Service Unavailable"));
    client.println(F("Content-Type: text/html"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("<html><body>SD not available</body></html>"));
    return;
  }

  File html = SD.open("index.htm", FILE_READ);
  if (!html) {
    client.println(F("HTTP/1.1 404 Not Found"));
    client.println(F("Content-Type: text/html"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("<html><body>index.htm not found on SD card</body></html>"));
    return;
  }

  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();

  uint8_t buf[64];
  int n;
  while ((n = html.read(buf, sizeof(buf))) > 0) {
    client.write(buf, n);
    serviceMotorJog();   // PRIORITY: keep feeding the drive while serving
  }
  html.close();
}

// ================= SETUP =================
void setup() {

  // ONLY SPI bus management required in the whole program: both CS pins as
  // outputs and idle (HIGH) BEFORE any initialization, so that no device is
  // selected by default while the other one initializes. From then on, each
  // library (Ethernet, SD) drives its own CS on every transaction.
  pinMode(10, OUTPUT);         // Ethernet CS (W5x00)
  digitalWrite(10, HIGH);
  pinMode(4, OUTPUT);          // SD CS
  digitalWrite(4, HIGH);

  // Initialize Modbus serial communication (9600 baud, 8N1 = drive Pr5.24 = 4)
  modbusSerial.begin(9600);
  delay(100);

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  Serial.println(F("Arduino Status Check Starting..."));
  delay(500); // Delay to let Hardware power up before check

  // Check Ethernet shield status (prints to Serial)
  EthernetStatus ethStatus = checkEthernetStatus(Serial);

  // Start web server only after Ethernet is initialized
  server.begin();

  // Check SD card status + single FAT mount (prints to Serial)
  SDCardStatus sdStatus = checkSDCardStatus(Serial);

  // Write the same status output to the SD card file
  createtxt(ethStatus, sdStatus);

  if (readFeedbackPosition()) {
    Serial.print(F("Initial feedback position (pulses): "));
    Serial.println(feedbackPositionPulses);
  }

  Serial.println(F("Status check complete."));
}

// ================= MAIN LOOP =================
// Strict priority order:
//   1) Modbus JOG command to the motor (refresh)  - every pass
//   2) HTTP (one client per pass; file serving interleaves 1) per chunk)
//   3) Feedback position poll - only while stopped, 1 Hz
void loop() {
  serviceMotorJog();

  EthernetClient client = server.available();
  if (client) {
    handleClient(client);
    return;
  }

  servicePositionPoll();
}
