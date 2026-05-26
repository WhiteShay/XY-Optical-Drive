/*
 * exemple_main.cpp
 * Hypothetical example: any Modbus motor controlled by changing only the slave address in the URL.
 *
 * CHANGES vs main.cpp:
 * 1. MOTOR_SLAVE_ID constant removed.
 * 2. Global `activeSlaveId` tracks which slave is currently moving (used by serviceMotorJog).
 * 3. Motor functions (startMotorCW, startMotorCCW, stopMotor, setupMotorDrive,
 *    motorTestSequence) each accept a `uint8_t slaveId` and call motor.begin() to
 *    re-point the single ModbusMaster object at that slave before any transaction.
 * 4. handleClient() dispatches on the pattern  GET /motor/{id}/{action}
 *    instead of the fixed /cw  /ccw  /stop  /motortest  endpoints.
 * 5. sendMotorActionResponse() and sendStatusResponse() include "slave_id" in their JSON.
 * 6. The HTML page (index.htm on SD) must add a motor-ID input — see the inline comment
 *    inside the SD-unavailable fallback page for the required JS change.
 *
 * Everything else (Ethernet, SD, hardware checks, SPI bus-swap helpers) is unchanged.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

// ================= NETWORK CONFIGURATION =================
byte mac[6] = { 0xA8, 0x61, 0x0A, 0xAE, 0x1C, 0xE4 };
IPAddress ip(169, 254, 43, 138);

// ================= WEB SERVER =================
EthernetServer server(80);
bool sdReady = false;

// ================= MOTOR CONTROL PINS =================
const uint8_t PIN_LED_motCW  = 2;
const uint8_t PIN_LED_motCCW = 5;

// ================= MODBUS MOTOR CONFIGURATION =================
SoftwareSerial modbusSerial(9, 8);
static const uint32_t MODBUS_BAUD  = 9600;
static const uint32_t DEBUG_BAUD   = 115200;

ModbusMaster motor;

// Register map
static const uint16_t REG_RS485_ENABLE = 0x000F;
static const uint16_t REG_DIRECTION    = 0x0007;
static const uint16_t REG_JOG_SPEED    = 0x01E1;
static const uint16_t REG_JOG_ACCEL    = 0x01E7;
static const uint16_t REG_CONTROL_WORD = 0x1801;
static const uint16_t REG_QUICK_STOP   = 0x6002;

static const uint16_t RS485_ENABLE_VALUE = 0x0001;
static const uint16_t DIR_CW             = 0x0000;
static const uint16_t DIR_CCW            = 0x0001;
static const uint16_t JOG_SPEED_RPM      = 60;
static const uint16_t JOG_ACCEL_VALUE    = 50;
static const uint16_t JOG_CW_CMD         = 0x4001;
static const uint16_t JOG_CCW_CMD        = 0x4002;
static const uint16_t STOP_CMD           = 0x0004;
static const uint16_t QUICK_STOP_CMD     = 0x0040;

static const uint32_t JOG_REFRESH_MS = 20;

enum MotorState { MOTOR_STOPPED, MOTOR_CW, MOTOR_CCW };

MotorState motorState  = MOTOR_STOPPED;
uint8_t activeSlaveId  = 1;   // ← NEW: which slave is currently active/moving
uint32_t lastJogRefresh = 0;
uint8_t lastModbusResult = 0;

// ================= HELPER STRUCTURES =================
struct EthernetStatus {
  bool detected;
  int hwStatus;
  bool linkOn;
};

struct SDCardStatus {
  bool detected;
  bool initialized;
  uint8_t cardType;
  int fatType;
  uint32_t volumeKB;
};

// ================= FORWARD DECLARATIONS =================
EthernetStatus checkEthernetStatus(Print &out);
SDCardStatus checkSDCardStatus(Print &out);
void createtxt(const EthernetStatus &eth, const SDCardStatus &sd);
void verifyAndClearTxtFile();
void ActiveSD();
void activeEthernet();
void printIPAddress(Print &out, const IPAddress &address);

bool readHttpLine(EthernetClient &client, char *buffer, size_t bufferSize, uint16_t timeoutMs);
void sendHttpCommonHeaders(EthernetClient &client, const __FlashStringHelper *contentType, bool keepAlive);

void handleClient(EthernetClient client);
void sendStatusResponse(EthernetClient client, bool keepAlive);
void sendHWStatusResponse(EthernetClient client, bool keepAlive);
void sendHTMLPage(EthernetClient client, bool keepAlive);

bool writeMotorRegister(uint16_t reg, uint16_t value);
void setupMotorDrive(uint8_t slaveId);       // ← slaveId param added
void startMotorCW(uint8_t slaveId);          // ← slaveId param added
void startMotorCCW(uint8_t slaveId);         // ← slaveId param added
void stopMotor(uint8_t slaveId);             // ← slaveId param added
void serviceMotorJog();
const __FlashStringHelper* motorStateText();
void sendMotorActionResponse(EthernetClient client, const char *message, bool keepAlive);

static const uint16_t HTTP_READ_TIMEOUT_MS = 1000;
static const uint16_t HTTP_KEEPALIVE_IDLE_MS = 1000;
static const uint8_t HTTP_MAX_REQUESTS_PER_CONNECTION = 4;

// ================= SPI BUS HELPERS =================
void ActiveSD() {
  digitalWrite(10, HIGH);
  digitalWrite(4, LOW);
}

void activeEthernet() {
  digitalWrite(4, HIGH);
  digitalWrite(10, LOW);
}

// ================= HARDWARE STATUS HELPERS =================
EthernetStatus checkEthernetStatus(Print &out) {
  EthernetStatus status = {};
  out.println(F("\n--- Ethernet Communication Status ---"));
  activeEthernet();
  delay(100);
  Ethernet.begin(mac, ip);
  status.hwStatus = Ethernet.hardwareStatus();
  if (status.hwStatus == EthernetNoHardware) {
    out.print(F("Ethernet.hardwareStatus() = ")); out.println(status.hwStatus);
    out.println(F("Ethernet shield: NOT DETECTED"));
    status.detected = false; status.linkOn = false;
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
    out.println(status.linkOn ? F("Ethernet link: CONNECTED") : F("Ethernet link: DISCONNECTED"));
  }
  return status;
}

SDCardStatus checkSDCardStatus(Print &out) {
  SDCardStatus status = {};
  out.println(F("\n--- SD Card Status ---"));
  ActiveSD();
  if (!SD.begin(4)) {
    out.println(F("SD card: NOT DETECTED or FAILED TO INITIALIZE"));
    status.detected = false; status.initialized = false;
  } else {
    status.detected = true; status.initialized = true; sdReady = true;
    out.println(F("SD card: DETECTED and INITIALIZED"));
    Sd2Card card; SdVolume volume;
    if (card.init(SPI_HALF_SPEED, 4)) {
      switch (card.type()) {
        case SD_CARD_TYPE_SD1:  status.cardType = 1; out.println(F("Card type: SD1"));   break;
        case SD_CARD_TYPE_SD2:  status.cardType = 2; out.println(F("Card type: SD2"));   break;
        case SD_CARD_TYPE_SDHC: status.cardType = 3; out.println(F("Card type: SDHC")); break;
        default: status.cardType = 0; out.println(F("Card type: Unknown"));
      }
      if (volume.init(card)) {
        status.fatType = volume.fatType();
        uint32_t vs = volume.blocksPerCluster(); vs *= volume.clusterCount(); vs /= 2;
        status.volumeKB = vs;
        out.print(F("Volume type: FAT")); out.println(status.fatType, DEC);
        out.print(F("Volume size: ")); out.print(vs); out.println(F(" KB"));
      }
    }
  }
  activeEthernet();
  return status;
}

void printIPAddress(Print &out, const IPAddress &address) {
  for (uint8_t i = 0; i < 4; i++) {
    out.print(address[i], DEC);
    if (i < 3) out.print(F("."));
  }
}

void writeHardwareStatusToFile(File &file, const EthernetStatus &eth, const SDCardStatus &sd) {
  file.println(F("\n--- Ethernet Communication Status ---"));
  if (!eth.detected) {
    file.println(F("Ethernet shield: NOT DETECTED"));
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
      file.print(F("IP address: ")); printIPAddress(file, ip); file.println();
    } else {
      file.println(F("Ethernet link: DISCONNECTED"));
    }
  }
  file.println(F("\n--- SD Card Status ---"));
  if (!sd.detected) {
    file.println(F("SD card: NOT DETECTED or FAILED TO INITIALIZE"));
  } else {
    file.println(F("SD card: DETECTED and INITIALIZED"));
    file.print(F("Volume type: FAT")); file.println(sd.fatType, DEC);
    file.print(F("Volume size: ")); file.print(sd.volumeKB); file.println(F(" KB"));
  }
}

void createtxt(const EthernetStatus &eth, const SDCardStatus &sd) {
  ActiveSD();
  if (!SD.begin(4)) return;
  SD.remove("test.txt");
  File file = SD.open("test.txt", FILE_WRITE);
  if (!file) return;
  file.print(F("Timestamp (uptime): ")); file.print(millis()); file.println(F(" ms")); file.println();
  writeHardwareStatusToFile(file, eth, sd);
  file.close();
  activeEthernet();
}

void verifyAndClearTxtFile() {
  ActiveSD();
  if (!SD.begin(4)) return;
  if (!SD.exists("test.txt")) return;
  File file = SD.open("test.txt", FILE_READ);
  if (!file) return;
  bool empty = (file.size() == 0);
  file.close();
  if (!empty) {
    file = SD.open("test.txt", FILE_WRITE);
    if (file) file.close();
  }
}

// ================= MODBUS MOTOR FUNCTIONS =================
bool writeMotorRegister(uint16_t reg, uint16_t value) {
  // motor object is already configured with the correct slave before calling this.
  uint8_t result = motor.writeSingleRegister(reg, value);
  lastModbusResult = result;
  return result == motor.ku8MBSuccess;
}

/*
 * KEY CONCEPT: motor.begin(slaveId, modbusSerial) re-points the single ModbusMaster
 * instance at a different slave address. It is cheap (no serial traffic), so calling
 * it at the start of every command is safe and costs only a few nanoseconds.
 * This means you never need multiple ModbusMaster objects — just one, re-configured
 * each time you want to address a different drive.
 */

void setupMotorDrive(uint8_t slaveId) {
  motor.begin(slaveId, modbusSerial);   // ← point at the requested slave
  writeMotorRegister(REG_RS485_ENABLE, RS485_ENABLE_VALUE); delay(5);
  writeMotorRegister(REG_JOG_SPEED, JOG_SPEED_RPM);        delay(5);
  writeMotorRegister(REG_JOG_ACCEL, JOG_ACCEL_VALUE);      delay(5);
  stopMotor(slaveId);
}

void startMotorCW(uint8_t slaveId) {
  activeSlaveId = slaveId;                                  // ← remember for serviceMotorJog
  motor.begin(slaveId, modbusSerial);
  writeMotorRegister(REG_RS485_ENABLE, RS485_ENABLE_VALUE); delay(5);
  writeMotorRegister(REG_DIRECTION, DIR_CW);                delay(5);
  writeMotorRegister(REG_JOG_SPEED, JOG_SPEED_RPM);        delay(5);
  motorState = MOTOR_CW;
  lastJogRefresh = 0;
  digitalWrite(PIN_LED_motCW, HIGH);
  digitalWrite(PIN_LED_motCCW, LOW);
}

void startMotorCCW(uint8_t slaveId) {
  activeSlaveId = slaveId;
  motor.begin(slaveId, modbusSerial);
  writeMotorRegister(REG_RS485_ENABLE, RS485_ENABLE_VALUE); delay(5);
  writeMotorRegister(REG_DIRECTION, DIR_CCW);               delay(5);
  writeMotorRegister(REG_JOG_SPEED, JOG_SPEED_RPM);        delay(5);
  motorState = MOTOR_CCW;
  lastJogRefresh = 0;
  digitalWrite(PIN_LED_motCW, LOW);
  digitalWrite(PIN_LED_motCCW, HIGH);
}

void stopMotor(uint8_t slaveId) {
  motor.begin(slaveId, modbusSerial);
  motorState = MOTOR_STOPPED;
  writeMotorRegister(REG_CONTROL_WORD, STOP_CMD);      delay(5);
  writeMotorRegister(REG_QUICK_STOP, QUICK_STOP_CMD);
  digitalWrite(PIN_LED_motCW, LOW);
  digitalWrite(PIN_LED_motCCW, LOW);
}

void serviceMotorJog() {
  if (motorState == MOTOR_STOPPED) return;
  uint32_t now = millis();
  if (now - lastJogRefresh < JOG_REFRESH_MS) return;
  lastJogRefresh = now;
  // activeSlaveId was set by the last startMotorCW/CCW call.
  // motor.begin() was already called then, so the object is still configured
  // for the right slave — no need to call begin() again here.
  if (motorState == MOTOR_CW)  writeMotorRegister(REG_CONTROL_WORD, JOG_CW_CMD);
  else                          writeMotorRegister(REG_CONTROL_WORD, JOG_CCW_CMD);
}

const __FlashStringHelper* motorStateText() {
  switch (motorState) {
    case MOTOR_CW:  return F("cw");
    case MOTOR_CCW: return F("ccw");
    default:        return F("stopped");
  }
}

void sendMotorActionResponse(EthernetClient client, const char *message, bool keepAlive) {
  sendHttpCommonHeaders(client, F("application/json"), keepAlive);
  client.print(F("{\"ok\":true,\"message\":\""));  client.print(message);
  client.print(F("\",\"slave_id\":"));             client.print(activeSlaveId);  // ← NEW field
  client.print(F(",\"motor_state\":\""));           client.print(motorStateText());
  client.print(F("\",\"jog_speed_rpm\":"));         client.print(JOG_SPEED_RPM);
  client.print(F(",\"jog_refresh_ms\":"));          client.print(JOG_REFRESH_MS);
  client.print(F(",\"last_modbus_result\":"));      client.print(lastModbusResult);
  client.print(F("}"));
  client.println();
}

// ================= MOTOR TEST SEQUENCE =================
void motorTestSequence(uint8_t slaveId) {
  unsigned long phaseStart = millis();
  while (millis() - phaseStart < 2000) { serviceMotorJog(); delay(1); }

  setupMotorDrive(slaveId);
  delay(300);

  startMotorCW(slaveId);
  phaseStart = millis();
  while (millis() - phaseStart < 3000) { serviceMotorJog(); delay(1); }

  stopMotor(slaveId);
  phaseStart = millis();
  while (millis() - phaseStart < 2000) { serviceMotorJog(); delay(1); }

  startMotorCCW(slaveId);
  phaseStart = millis();
  while (millis() - phaseStart < 3000) { serviceMotorJog(); delay(1); }

  stopMotor(slaveId);
}

// ================= HTTP HELPERS =================
bool readHttpLine(EthernetClient &client, char *buffer, size_t bufferSize, uint16_t timeoutMs) {
  if (bufferSize == 0) return false;
  size_t index = 0;
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    serviceMotorJog();
    while (client.available()) {
      char c = client.read();
      if (c == '\r') continue;
      if (c == '\n') { buffer[index] = '\0'; return true; }
      if (index < bufferSize - 1) buffer[index++] = c;
      start = millis();
    }
    if (!client.connected()) break;
  }
  buffer[index] = '\0';
  return index > 0;
}

void sendHttpCommonHeaders(EthernetClient &client, const __FlashStringHelper *contentType, bool keepAlive) {
  client.println(F("HTTP/1.1 200 OK"));
  client.print(F("Content-Type: ")); client.println(contentType);
  client.println(keepAlive ? F("Connection: keep-alive") : F("Connection: close"));
  if (keepAlive) {
    client.print(F("Keep-Alive: timeout="));
    client.print((HTTP_KEEPALIVE_IDLE_MS + 999) / 1000);
    client.print(F(", max="));
    client.println(HTTP_MAX_REQUESTS_PER_CONNECTION);
  }
  client.println();
}

// ================= URL ROUTING =================
/*
 * New URL scheme:   GET /motor/{slaveId}/{action}
 *
 * Examples:
 *   GET /motor/1/cw    → startMotorCW(1)
 *   GET /motor/2/ccw   → startMotorCCW(2)
 *   GET /motor/1/stop  → stopMotor(1)
 *   GET /motor/3/test  → motorTestSequence(3)
 *
 * slaveId must be 1–31 (valid Modbus RTU range).
 *
 * PARSING STRATEGY (no sscanf, no heap allocation — safe on Uno):
 *   After confirming the prefix "GET /motor/", walk character-by-character
 *   to extract the numeric ID, then read the action string.
 */
static bool parseMotorPath(const char *request, uint8_t &slaveId, char *action, uint8_t actionSize) {
  // request looks like: "GET /motor/2/ccw HTTP/1.1"
  const char *p = strstr(request, "GET /motor/");
  if (!p) return false;
  p += 11; // skip "GET /motor/"

  // Parse slave ID digits
  slaveId = 0;
  while (*p >= '0' && *p <= '9') {
    slaveId = slaveId * 10 + (*p - '0');
    p++;
  }
  if (slaveId < 1 || slaveId > 31) return false;
  if (*p != '/') return false;
  p++; // skip '/'

  // Copy action string until space or end
  uint8_t i = 0;
  while (*p && *p != ' ' && *p != '\r' && *p != '\n' && i < actionSize - 1) {
    action[i++] = *p++;
  }
  action[i] = '\0';
  return (i > 0);
}

void handleClient(EthernetClient client) {
  char request[56] = {0};
  size_t index = 0;
  while (client.available() && index < sizeof(request) - 1) {
    char c = client.read();
    request[index++] = c;
    if (c == '\n') break;
  }
  request[index] = '\0';

  // ---- Fixed system endpoints (unchanged) ----
  if (strstr(request, "GET /status ")) {
    sendStatusResponse(client, false);
  } else if (strstr(request, "GET /hwstatus ")) {
    sendHWStatusResponse(client, false);
  } else if (strstr(request, "GET / ")) {
    sendHTMLPage(client, false);

  // ---- Motor endpoints: /motor/{id}/{action} ----
  } else {
    uint8_t slaveId = 0;
    char action[8] = {0};

    if (parseMotorPath(request, slaveId, action, sizeof(action))) {
      if (strcmp(action, "cw") == 0) {
        startMotorCW(slaveId);
        sendMotorActionResponse(client, "CW JOG started", false);
      } else if (strcmp(action, "ccw") == 0) {
        startMotorCCW(slaveId);
        sendMotorActionResponse(client, "CCW JOG started", false);
      } else if (strcmp(action, "stop") == 0) {
        stopMotor(slaveId);
        sendMotorActionResponse(client, "Motor stopped", false);
      } else if (strcmp(action, "test") == 0) {
        motorTestSequence(slaveId);
        sendMotorActionResponse(client, "Motor test completed", false);
      } else {
        client.println(F("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n"));
      }
    } else {
      client.println(F("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n"));
    }
  }

  client.stop();
}

// ================= RESPONSE FUNCTIONS =================
void sendStatusResponse(EthernetClient client, bool keepAlive) {
  sendHttpCommonHeaders(client, F("application/json"), keepAlive);
  client.print(F("{\"uptime\":"));           client.print(millis());
  client.print(F(",\"slave_id\":"));          client.print(activeSlaveId);   // ← NEW field
  client.print(F(",\"motor_state\":\""));     client.print(motorStateText());
  client.print(F("\",\"jog_speed_rpm\":"));   client.print(JOG_SPEED_RPM);
  client.print(F(",\"jog_refresh_ms\":"));    client.print(JOG_REFRESH_MS);
  client.print(F(",\"last_modbus_result\":")); client.print(lastModbusResult);
  client.print(F("}"));
  client.println();
}

void sendHWStatusResponse(EthernetClient client, bool keepAlive) {
  if (!sdReady) {
    client.println(F("HTTP/1.1 503 Service Unavailable\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nSD not available"));
    return;
  }
  sendHttpCommonHeaders(client, F("text/plain"), keepAlive);
  ActiveSD();
  File txt = SD.open("test.txt", FILE_READ);
  if (!txt) { activeEthernet(); client.println(F("test.txt not found")); return; }
  uint8_t buf[32];
  while (true) {
    ActiveSD();
    if (!txt.available()) break;
    int n = txt.read(buf, sizeof(buf));
    activeEthernet();
    if (n > 0) client.write(buf, n);
    serviceMotorJog();
  }
  ActiveSD(); txt.close(); activeEthernet();
}

void sendHTMLPage(EthernetClient client, bool keepAlive) {
  if (!sdReady) {
    // SD unavailable fallback — inline the minimal page.
    // NOTE: This inline page shows the updated JS needed in index.htm on the SD card.
    client.println(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n"));
    client.println(F("<!DOCTYPE html><html><head><meta charset='UTF-8'><title>XY Optical Drive</title></head>"));
    client.println(F("<body style='font-family:Arial,sans-serif;padding:16px;background:#1e1e1e;color:#e0e0e0'>"));
    client.println(F("<h1>XY Optical Drive</h1>"));
    client.println(F("<p><em>SD not available — using embedded fallback page.</em></p>"));
    client.println(F("<h2>Motor Control</h2>"));
    client.println(F("<style>.mb{font-size:1.2em;padding:10px 20px;border:none;border-radius:6px;cursor:pointer;background:#2d2d2d;color:#e0e0e0;margin-right:4px;border:1px solid #404040;}</style>"));
    // Motor ID input
    client.println(F("<label>Motor (Slave ID): <input id='mid' type='number' min='1' max='31' value='1' style='width:50px;background:#2d2d2d;color:#e0e0e0;border:1px solid #404040;padding:4px;'></label>"));
    client.println(F("<div style='margin:12px 0;'>"));
    client.println(F("  <button class='mb' onclick=\"cmd('ccw')\">&#9664; CCLKW</button>"));
    client.println(F("  <button class='mb' onclick=\"cmd('stop')\">&#9646;&#9646; Stop</button>"));
    client.println(F("  <button class='mb' onclick=\"cmd('cw')\">CLKW &#9654;</button>"));
    client.println(F("  <button class='mb' onclick=\"runTest()\" style='background:#1565c0;color:#fff;margin-left:8px'>Run Test</button>"));
    client.println(F("</div>"));
    client.println(F("<p id='ms'>Status: Stopped</p>"));
    // KEY JS CHANGE: cmd() builds /motor/{id}/{action}
    client.println(F("<script>"));
    client.println(F("function motorId(){return document.getElementById('mid').value;}"));
    client.println(F("var L={cw:'Running CW',ccw:'Running CCW',stop:'Stopped'};"));
    client.println(F("function cmd(d){"));
    client.println(F("  fetch('/motor/'+motorId()+'/'+d)"));
    client.println(F("    .then(function(r){return r.json();})"));
    client.println(F("    .then(function(j){document.getElementById('ms').textContent='Status: '+j.motor_state+' (slave '+j.slave_id+')';});"));
    client.println(F("}"));
    client.println(F("function runTest(){fetch('/motor/'+motorId()+'/test').then(function(){document.getElementById('ms').textContent='Test done';}); }"));
    client.println(F("fetch('/status').then(function(r){return r.json();}).then(function(d){document.getElementById('ms').textContent='slave '+d.slave_id+': '+d.motor_state;});"));
    client.println(F("</script></body></html>"));
    return;
  }

  // Normal path: serve index.htm from SD card.
  ActiveSD();
  File html = SD.open("index.htm", FILE_READ);
  if (!html) {
    activeEthernet();
    client.println(F("HTTP/1.1 404 Not Found\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<h2>index.htm not found on SD card</h2>"));
    return;
  }
  activeEthernet();
  sendHttpCommonHeaders(client, F("text/html"), keepAlive);
  uint8_t buf[64];
  while (client.connected()) {
    ActiveSD();
    if (!html.available()) break;
    int n = html.read(buf, sizeof(buf));
    activeEthernet();
    if (n > 0) client.write(buf, n);
    serviceMotorJog();
  }
  ActiveSD(); html.close(); activeEthernet();
}

// ================= SETUP =================
void setup() {
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(4, OUTPUT);  digitalWrite(4, HIGH);

  pinMode(PIN_LED_motCW, OUTPUT);  digitalWrite(PIN_LED_motCW, LOW);
  pinMode(PIN_LED_motCCW, OUTPUT); digitalWrite(PIN_LED_motCCW, LOW);

  Serial.begin(DEBUG_BAUD);

  modbusSerial.begin(MODBUS_BAUD);
  // No motor.begin() here — each motor function calls it with the right slaveId.

  delay(500);

  EthernetStatus ethStatus = checkEthernetStatus(Serial);
  SDCardStatus sdStatus    = checkSDCardStatus(Serial);
  createtxt(ethStatus, sdStatus);

  delay(500);
  activeEthernet();
  server.begin();
}

// ================= LOOP =================
void loop() {
  EthernetClient client = server.available();
  if (client) handleClient(client);
  serviceMotorJog();
}
