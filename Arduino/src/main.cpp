/*
 * XY Optical Drive_V0.1.0.ino
 * * DESCRIPTION:
 * Reliable Lab Automation Controller for Arduino Uno + Ethernet Shield + Stepper Motors NEMA 17.
 *  * * HARDWARE STACK:
 * 1. Arduino Uno R3 (Bottom)
 * 2. Ethernet Shield W5100/W5500 (Middle) - Uses SPI (ICSP Header + Pin 10)
 * 3. Integrated Stepper Motor Drivers (e.g., A4988) connected to PC, drive via Modbus communication for step/dir and power.
 * 
 * * ================= REST API REFERENCE =================
 * * 1. GET SYSTEM STATUS
 * Endpoint: GET /status
 * Response: JSON object containing uptime.
 * * ======================================================
 * * HISTORY / UPDATES:
 * * 03/03/2026 ==> 06/03/2026 - V0.1.0 - Initial Development
 * * - Added Arduino Ethernet and SD hardware check on power up. -DONE-
 * * - Added txt file creation. -DONE-
 * * 09/03/2026 ==> 20/03/2026 - V0.1.0 -Initial Development
 * * - Status and control of everything to text file (1 Arduino memory, 2 arduino eth+SD card status) -DONE-
 * * - Html page to display status -DONE-
 * * - LED Blink from Webpage. -DONE- 
 * * 23/03/2026 ==> 30/03/2026 - V0.1.0 - Motor control development
 * * - Modbus library integration for stepper motor control - DONE-
 * * - Webpage buttons to control motor direction and stop - STANDBY (Requires ordered hardware)
 * * 30/03/2026 ==> 10/04/2026 - V0.1.0 - Finalize motor control and testing
 * *  - HTML on SD card - DONE -
 * *  - Full app architecture OK (PC - Arduino Web Server) - DONE -
 * *  - Functional IP adress sweep for Arduino discovery. - IN PROGRESS --> up to V1.0.0
 * * 13/04/2026 ==> 20/04/2026 - V1.0.1->V1.0.3 - First test Modbus motor control.
 * * *  - Test Modbus communication with motor drivers. - DONE (11/05)
 * *  - V1.0.2: Homing + 0 definition - IN PROGRESS (12/05)
 * *  - V1.0.3: Move CW/CCW + position increment - IN PROGRESS (12/05)
 * 27/04/2026 ==> 15/05/2026 - V1.0.3->V1.0.7 - Finalize motor control and testing
 * *  - V1.0.3: Speed switch. - TO DO
 * *  - V1.0.4: Emergency Stop. - DONE (11/05)
 * *  - V1.0.5: Save position - TO DO
 * *  - V1.0.6: Go to position mode - TO DO
 * 
 */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <ModbusMaster.h>
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
//IPAddress subnet(255, 255, 0 , 0);
// IPAddress myDns(8, 8, 8, 8); // Not strictly used for local control

// ================= WEB SERVER =================
EthernetServer server(80);
bool sdReady = false; // Set to true after successful SD.begin() during setup


// ================= MOTOR CONTROL PINS =================
// Pin 4 is reserved as SD card CS on the Ethernet Shield — do not use it here.
const uint8_t PIN_LED_motCW  = 2;  // DO: Clockwise direction output / test LED CW
const uint8_t PIN_LED_motCCW = 5;  // DO: Counter-clockwise direction output / test LED CCW

// ================= MODBUS MOTOR CONFIGURATION =================
// Modbus RTU uses SoftwareSerial on pins 9 (RX) and 8 (TX).
SoftwareSerial modbusSerial(9, 8);
static const uint32_t MODBUS_BAUD = 9600;
static const uint8_t MOTOR_SLAVE_ID = 1;

// Debug Serial hardware line : RX=pin0, TX=pin1
static const uint32_t DEBUG_BAUD = 115200;

ModbusMaster motor;

// Register map from the motor drive manual
static const uint16_t REG_RS485_ENABLE = 0x000F;  // Pr0.07: 1 = control by RS485
static const uint16_t REG_DIRECTION    = 0x0007;  // 0 = CW, 1 = CCW
static const uint16_t REG_JOG_SPEED    = 0x01E1;  // Pr6.00: JOG velocity
static const uint16_t REG_JOG_ACCEL    = 0x01E7;  // Pr6.03: JOG acceleration/deceleration time
static const uint16_t REG_CONTROL_WORD = 0x1801;  // JOG command/control word
static const uint16_t REG_QUICK_STOP   = 0x6002;  // JOG quick stop register

// Values to write
static const uint16_t RS485_ENABLE_VALUE = 0x0001;
static const uint16_t DIR_CW             = 0x0000;
static const uint16_t DIR_CCW            = 0x0001;
static const uint16_t JOG_SPEED_RPM      = 60;
static const uint16_t JOG_ACCEL_VALUE    = 50;     // Adjust to the unit used by the drive manual
static const uint16_t JOG_CW_CMD         = 0x4001;
static const uint16_t JOG_CCW_CMD        = 0x4002;
static const uint16_t STOP_CMD           = 0x0004;
static const uint16_t QUICK_STOP_CMD     = 0x0040;

// The drive manual says the JOG trigger interval must be less than 50 ms.
// 20 ms gives margin while still being light enough for the Uno.
static const uint32_t JOG_REFRESH_MS = 20;

enum MotorState {
  MOTOR_STOPPED,
  MOTOR_CW,
  MOTOR_CCW
};

MotorState motorState = MOTOR_STOPPED;
uint32_t lastJogRefresh = 0;
uint8_t lastModbusResult = 0;

// ================= MOTOR TEST STATE MACHINE =================
enum MotorTestPhase {
  MTEST_IDLE = 0,
  MTEST_POWER_ON_DELAY,   // wait 2000 ms before first command
  MTEST_SETUP_SETTLE,     // wait 300 ms after setupMotorDrive()
  MTEST_FORWARD_JOG,      // run CW for 3000 ms
  MTEST_STOP_PAUSE,       // stopped for 2000 ms
  MTEST_REVERSE_JOG,      // run CCW for 3000 ms
  MTEST_DONE              // stop and return to IDLE next tick
};

MotorTestPhase motorTestPhase     = MTEST_IDLE;
uint32_t       motorTestPhaseStart = 0;

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
// Helpers to report hardware status to any Print target (Serial or File)
EthernetStatus checkEthernetStatus(Print &out);
SDCardStatus checkSDCardStatus(Print &out);

void createtxt(const EthernetStatus &eth, const SDCardStatus &sd); // Create .txt file to store hardware setup results on SD card
void verifyAndClearTxtFile(); //Used to erase the content of the SD .txt file.
void ActiveSD(); // Activate SD card on SPI bus (de-select Ethernet, select SD)
void activeEthernet(); // Activate Ethernet on SPI bus (de-select SD, select Ethernet) - not currently used but can be useful for future development of HTML page to display status and control motors.

void printIPAddress(Print &out, const IPAddress &address);

bool readHttpLine(EthernetClient &client, char *buffer, size_t bufferSize, uint16_t timeoutMs);
bool parseRequestLine(const char *line, char *method, size_t methodSize, char *path, size_t pathSize, bool &http11);
void sendHttpCommonHeaders(EthernetClient &client, const __FlashStringHelper *contentType, bool keepAlive);

void handleClient(EthernetClient client);
void sendStatusResponse(EthernetClient client, bool keepAlive);
void sendHWStatusResponse(EthernetClient client, bool keepAlive);
void sendHTMLPage(EthernetClient client, bool keepAlive);

bool writeMotorRegister(uint16_t reg, uint16_t value);
void setupMotorDrive();
void startMotorCW();
void startMotorCCW();
void stopMotor();
void serviceMotorJog();
const __FlashStringHelper* motorStateText();
void sendMotorActionResponse(EthernetClient client, const char *message, bool keepAlive);
void startMotorTest();
void serviceMotorTest();
void diagSDInit();

static const uint16_t HTTP_READ_TIMEOUT_MS = 1000;
static const uint16_t HTTP_KEEPALIVE_IDLE_MS = 1000;
static const uint8_t HTTP_MAX_REQUESTS_PER_CONNECTION = 4;

EthernetStatus checkEthernetStatus(Print &out) {
  EthernetStatus status = {};

  out.println(F("\n--- Ethernet Communication Status ---"));

  activeEthernet(); // Ensures SD is de-selected (HIGH) and Ethernet CS is active (LOW)
  delay(100); // Delay to allow hardware to stabilize before checking status
  Ethernet.begin(mac, ip); // Initialize Ethernet with manually defined MAC address and IP

  // Check if Ethernet shield is present
  status.hwStatus = Ethernet.hardwareStatus();
  if (status.hwStatus == EthernetNoHardware) {
    // Print the raw hardware status value
    out.print(F("Ethernet.hardwareStatus() = "));
    out.println(status.hwStatus);

    out.println(F("Ethernet shield: NOT DETECTED"));
    out.println(F("No Ethernet hardware found."));
    status.detected = false;
    status.linkOn = false;
  } else {
    status.detected = true;
    out.println(F("Ethernet shield: DETECTED"));

    // Print configured MAC address
    out.print(F("MAC Address: "));
    for (int i = 0; i < 6; i++) {
      if (mac[i] < 16) out.print(F("0"));
      out.print(mac[i], HEX);
      if (i < 5) out.print(F(":"));
    }
    out.println();

    // Check link status
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

  ActiveSD();

  // Check if SD card is present and can be initialized
  if (!SD.begin(4)) {  // SD card CS pin is usually pin 4 on Ethernet shields
    out.println(F("SD card: NOT DETECTED or FAILED TO INITIALIZE"));
    out.println(F("Possible reasons:"));
    out.println(F("- No SD card inserted"));
    out.println(F("- SD card not formatted properly"));
    out.println(F("- Wrong CS pin (currently using pin 4)"));
    status.detected = false;
    status.initialized = false;
  } else {
    status.detected = true;
    status.initialized = true;
    sdReady = true;
    out.println(F("SD card: DETECTED and INITIALIZED"));

    // Sd2Card/SdVolume direct calls commented out — they re-initialize the card
    // at SPI_HALF_SPEED and corrupt the SD library's internal volume handle,
    // causing SD.open("/") to fail in subsequent calls.
    /*
    Sd2Card card;
    SdVolume volume;
    uint32_t volumesize;

    if (card.init(SPI_HALF_SPEED, 4)) {
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

        volumesize = volume.blocksPerCluster();
        volumesize *= volume.clusterCount();
        volumesize /= 2;  // SD card blocks are always 512 bytes (1/2 KB)
        status.volumeKB = volumesize;
        out.print(F("Volume size: "));
        out.print(status.volumeKB);
        out.println(F(" KB"));

        out.println(F("Remaining space: Unknown"));
      }
    }
    */
  }

  // Hand SPI bus control back to Ethernet for network operations.
  activeEthernet();
  return status;
}

void ActiveSD() {
  // De-select Ethernet (shield) so SD can use the SPI bus
  digitalWrite(10, HIGH); // Ensure Ethernet CS is HIGH (de-selected)
  digitalWrite(4, LOW);  // SD CS
}

void activeEthernet() {
  // De-select SDso Ethernet (shield)  can use the SPI bus
  digitalWrite(4, HIGH); // Ensure SD CS is HIGH (de-selected)
  digitalWrite(10, LOW);  // Ethernet CS is active (LOW)

}



void printIPAddress(Print &out, const IPAddress &address) {
  for (uint8_t index = 0; index < 4; index++) {
    out.print(address[index], DEC);
    if (index < 3) {
      out.print(F("."));
    }
  }
}

bool readHttpLine(EthernetClient &client, char *buffer, size_t bufferSize, uint16_t timeoutMs) {
  if (bufferSize == 0) {
    return false;
  }

  size_t index = 0;
  uint32_t start = millis();

  while ((millis() - start) < timeoutMs) {
    serviceMotorJog();
    while (client.available()) {
      char c = client.read();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        buffer[index] = '\0';
        return true;
      }
      if (index < bufferSize - 1) {
        buffer[index++] = c;
      }
      start = millis();
    }

    if (!client.connected()) {
      break;
    }
  }

  buffer[index] = '\0';
  return index > 0;
}

bool parseRequestLine(const char *line, char *method, size_t methodSize, char *path, size_t pathSize, bool &http11) {
  method[0] = '\0';
  path[0] = '\0';
  http11 = false;

  const char *firstSpace = strchr(line, ' ');
  if (!firstSpace) {
    return false;
  }
  const char *secondSpace = strchr(firstSpace + 1, ' ');
  if (!secondSpace) {
    return false;
  }

  size_t methodLen = firstSpace - line;
  size_t pathLen = secondSpace - (firstSpace + 1);
  if (methodLen == 0 || pathLen == 0 || methodLen >= methodSize || pathLen >= pathSize) {
    return false;
  }

  memcpy(method, line, methodLen);
  method[methodLen] = '\0';
  memcpy(path, firstSpace + 1, pathLen);
  path[pathLen] = '\0';

  http11 = (strstr(secondSpace + 1, "HTTP/1.1") != NULL);
  return true;
}

void sendHttpCommonHeaders(EthernetClient &client, const __FlashStringHelper *contentType, bool keepAlive) {
  client.println(F("HTTP/1.1 200 OK"));
  client.print(F("Content-Type: "));
  client.println(contentType);
  client.println(keepAlive ? F("Connection: keep-alive") : F("Connection: close"));
  if (keepAlive) {
    client.print(F("Keep-Alive: timeout="));
    client.print((HTTP_KEEPALIVE_IDLE_MS + 999) / 1000);
    client.print(F(", max="));
    client.println(HTTP_MAX_REQUESTS_PER_CONNECTION);
  }
  client.println();
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

    // Print configured MAC address
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
    file.println(F("Remaining space: Unknown"));
  }
}

void createtxt(const EthernetStatus &eth, const SDCardStatus &sd) {
  ActiveSD();
  delayMicroseconds(100); // settle after SPI bus switch

  // Overwrite existing file so status is fresh each run
  SD.remove("test.txt");

  File file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    activeEthernet(); // release SPI bus before returning
    return;
  }

  // Add creation date and timestamp
  file.print(F("Timestamp (uptime): "));
  file.print(millis());
  file.println(F(" ms"));
  file.println();

  writeHardwareStatusToFile(file, eth, sd);

  file.close();
  activeEthernet();

}

void verifyAndClearTxtFile() {

  ActiveSD();
  delayMicroseconds(100); // settle after SPI bus switch

  // Check if the file exists
  if (!SD.exists("test.txt")) {
    return;
  }


  // Open the file to check if it's empty
  File file = SD.open("test.txt", FILE_READ);
  if (!file) {
    return;
  }

  // Check if the file is empty
  if (file.size() == 0) {
    file.close();
    return;
  }

  file.close();

  // Open the file in write mode to clear it (this truncates the file)
  file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    return;
  }

  // Since we opened in FILE_WRITE without writing anything, the file is now empty
  file.close();

}


// ================= MODBUS MOTOR FUNCTIONS =================
bool writeMotorRegister(uint16_t reg, uint16_t value) {
  uint8_t result = motor.writeSingleRegister(reg, value);
  lastModbusResult = result;
  return result == motor.ku8MBSuccess;
}

void setupMotorDrive() {
  // Enable drive control by RS485, then preload the JOG parameters.
  writeMotorRegister(REG_RS485_ENABLE, RS485_ENABLE_VALUE);
  delay(5);
  writeMotorRegister(REG_JOG_SPEED, JOG_SPEED_RPM);
  delay(5);
  writeMotorRegister(REG_JOG_ACCEL, JOG_ACCEL_VALUE);
  delay(5);

  stopMotor();
}

void startMotorCW() {
  // Configure direction and speed first. The continuous JOG command is sent by serviceMotorJog().
  writeMotorRegister(REG_RS485_ENABLE, RS485_ENABLE_VALUE);
  delay(5);
  writeMotorRegister(REG_DIRECTION, DIR_CW);
  delay(5);
  writeMotorRegister(REG_JOG_SPEED, JOG_SPEED_RPM);
  delay(5);

  motorState = MOTOR_CW;
  lastJogRefresh = 0;

  digitalWrite(PIN_LED_motCW, HIGH);
  digitalWrite(PIN_LED_motCCW, LOW);
}

void startMotorCCW() {
  // Configure direction and speed first. The continuous JOG command is sent by serviceMotorJog().
  writeMotorRegister(REG_RS485_ENABLE, RS485_ENABLE_VALUE);
  delay(5);
  writeMotorRegister(REG_DIRECTION, DIR_CCW);
  delay(5);
  writeMotorRegister(REG_JOG_SPEED, JOG_SPEED_RPM);
  delay(5);

  motorState = MOTOR_CCW;
  lastJogRefresh = 0;

  digitalWrite(PIN_LED_motCW, LOW);
  digitalWrite(PIN_LED_motCCW, HIGH);
}

void stopMotor() {
  motorState = MOTOR_STOPPED;

  // Requested STOP command on the control word.
  writeMotorRegister(REG_CONTROL_WORD, STOP_CMD);
  delay(5);

  // Manual quick-stop command. Comment this line out if your drive only expects 0x1801 = 0x0004.
  writeMotorRegister(REG_QUICK_STOP, QUICK_STOP_CMD);

  digitalWrite(PIN_LED_motCW, LOW);
  digitalWrite(PIN_LED_motCCW, LOW);
}

void serviceMotorJog() {
  if (motorState == MOTOR_STOPPED) {
    return;
  }

  uint32_t now = millis();
  if (now - lastJogRefresh < JOG_REFRESH_MS) {
    return;
  }

  lastJogRefresh = now;

  if (motorState == MOTOR_CW) {
    writeMotorRegister(REG_CONTROL_WORD, JOG_CW_CMD);
  } else if (motorState == MOTOR_CCW) {
    writeMotorRegister(REG_CONTROL_WORD, JOG_CCW_CMD);
  }
}

const __FlashStringHelper* motorStateText() {
  switch (motorState) {
    case MOTOR_CW: return F("cw");
    case MOTOR_CCW: return F("ccw");
    default: return F("stopped");
  }
}

void sendMotorActionResponse(EthernetClient client, const char *message, bool keepAlive) {
  sendHttpCommonHeaders(client, F("application/json"), keepAlive);
  client.print(F("{\"ok\":true,\"message\":\""));
  client.print(message);
  client.print(F("\",\"motor_state\":\""));
  client.print(motorStateText());
  client.print(F("\",\"jog_speed_rpm\":"));
  client.print(JOG_SPEED_RPM);
  client.print(F(",\"jog_refresh_ms\":"));
  client.print(JOG_REFRESH_MS);
  client.print(F(",\"last_modbus_result\":"));
  client.print(lastModbusResult);
  client.print(F("}"));
  client.println();
}

// ================= MOTOR TEST STATE MACHINE =================
// Non-blocking replacement for the old blocking motorTestSequence().
// Call startMotorTest() to arm; serviceMotorTest() (called from loop()) advances
// the sequence one tick at a time so the web server stays responsive throughout.

void startMotorTest() {
  if (motorTestPhase != MTEST_IDLE) return;   // already running, ignore
  motorTestPhase      = MTEST_POWER_ON_DELAY;
  motorTestPhaseStart = millis();
}

void serviceMotorTest() {
  if (motorTestPhase == MTEST_IDLE) return;

  uint32_t now = millis();

  switch (motorTestPhase) {

    case MTEST_POWER_ON_DELAY:
      // Wait 2 s, then load JOG parameters into the drive.
      if (now - motorTestPhaseStart >= 2000) {
        setupMotorDrive();            // ~15 ms of Modbus writes, acceptable
        motorTestPhase      = MTEST_SETUP_SETTLE;
        motorTestPhaseStart = now;
      }
      break;

    case MTEST_SETUP_SETTLE:
      // Give the drive 300 ms to apply the new parameters.
      if (now - motorTestPhaseStart >= 300) {
        startMotorCW();
        motorTestPhase      = MTEST_FORWARD_JOG;
        motorTestPhaseStart = now;
      }
      break;

    case MTEST_FORWARD_JOG:
      serviceMotorJog();
      if (now - motorTestPhaseStart >= 3000) {
        stopMotor();
        motorTestPhase      = MTEST_STOP_PAUSE;
        motorTestPhaseStart = now;
      }
      break;

    case MTEST_STOP_PAUSE:
      if (now - motorTestPhaseStart >= 2000) {
        startMotorCCW();
        motorTestPhase      = MTEST_REVERSE_JOG;
        motorTestPhaseStart = now;
      }
      break;

    case MTEST_REVERSE_JOG:
      serviceMotorJog();
      if (now - motorTestPhaseStart >= 3000) {
        stopMotor();
        motorTestPhase = MTEST_DONE;
      }
      break;

    case MTEST_DONE:
      motorTestPhase = MTEST_IDLE;
      break;

    default:
      motorTestPhase = MTEST_IDLE;
      break;
  }
}

// ================= WEB SERVER FUNCTIONS =================
void handleClient(EthernetClient client) {
  // Read only the HTTP request line (the first line ending in \n).
  // server.available() guarantees data is already in the W5100 FIFO when we
  // arrive here, so no timeout logic is needed — just drain bytes until \n.
  char request[48] = {0};
  size_t index = 0;
  while (client.available() && index < sizeof(request) - 1) {
    char c = client.read();
    request[index++] = c;
    if (c == '\n') break;
  }
  request[index] = '\0';

  // Dispatch on the request line. strstr() handles any trailing \r\n cleanly.
  if (strstr(request, "GET /status ")) {
    sendStatusResponse(client, false);
  } else if (strstr(request, "GET /cw ")) {
    startMotorCW();
    sendMotorActionResponse(client, "CW JOG started", false);
  } else if (strstr(request, "GET /ccw ")) {
    startMotorCCW();
    sendMotorActionResponse(client, "CCW JOG started", false);
  } else if (strstr(request, "GET /stop ")) {
    stopMotor();
    sendMotorActionResponse(client, "Motor stopped", false);
  } else if (strstr(request, "GET /motortest ")) {
    startMotorTest();
    sendMotorActionResponse(client, "Motor test started", false);
  } else if (strstr(request, "GET /hwstatus ")) {
    sendHWStatusResponse(client, false);
  } else if (strstr(request, "GET / ")) {
    sendHTMLPage(client, false);
  } else {
    client.println(F("HTTP/1.1 404 Not Found"));
    client.println(F("Connection: close"));
    client.println();
  }

  client.stop();
}

void sendStatusResponse(EthernetClient client, bool keepAlive) {
  sendHttpCommonHeaders(client, F("application/json"), keepAlive);
  client.print(F("{\"uptime\":"));
  client.print(millis());
  client.print(F(",\"motor_state\":\""));
  client.print(motorStateText());
  client.print(F("\",\"jog_speed_rpm\":"));
  client.print(JOG_SPEED_RPM);
  client.print(F(",\"jog_refresh_ms\":"));
  client.print(JOG_REFRESH_MS);
  client.print(F(",\"last_modbus_result\":"));
  client.print(lastModbusResult);
  client.print(F("}"));
  client.println();
}

void sendHWStatusResponse(EthernetClient client, bool keepAlive) {
  // sdReady is a plain bool — no SPI needed to check it.
  // The SPI bus is already in Ethernet mode when we arrive here from handleClient().
  if (!sdReady) {
    client.println(F("HTTP/1.1 503 Service Unavailable"));
    client.println(F("Content-Type: text/plain"));
    client.println(keepAlive ? F("Connection: keep-alive") : F("Connection: close"));
    client.println();
    client.println(F("SD not available"));
    return;
  }

  // Send HTTP headers while SPI is still in Ethernet mode.
  sendHttpCommonHeaders(client, F("text/plain"), keepAlive);

  // Switch to SD only now that the headers are out.
  ActiveSD();
  delayMicroseconds(100); // settle after SPI bus switch
  File txt = SD.open("test.txt", FILE_READ);
  if (!txt) {
    activeEthernet();
    client.println(F("test.txt not found"));
    return;
  }

  uint8_t buf[32];
  while (true) {
    ActiveSD();
    if (!txt.available()) break;
    int n = txt.read(buf, sizeof(buf));
    activeEthernet();
    if (n > 0) client.write(buf, n);
    serviceMotorJog();
  }
  ActiveSD();
  txt.close();
  activeEthernet();
}

// Serve index.htm from SD card.

void sendHTMLPage(EthernetClient client, bool keepAlive) {
  // sdReady is a plain bool — no SPI needed to check it.
  // The SPI bus is already in Ethernet mode when we arrive here from handleClient().
  if (!sdReady) {
    client.println(F("HTTP/1.1 503 Service Unavailable"));
    client.println(F("Content-Type: text/html"));
    client.println(keepAlive ? F("Connection: keep-alive") : F("Connection: close"));
    client.println();
    client.println(F("<html><body style='margin:0; padding:16px; background:#1e1e1e; color:#e0e0e0; font-family:Arial,sans-serif;'><h2>SD not available</h2><p>The web page could not be loaded because the SD card is not ready.</p></body></html>"));
    return;
  }

  // Switch to SD to open the file, then immediately assess success/failure.
  ActiveSD();
  delayMicroseconds(100); // settle after SPI bus switch
  File html = SD.open("/index.htm", FILE_READ);
  if (!html) {
    // File not found: switch to Ethernet and send a full 404 response.
    activeEthernet();
    client.println(F("HTTP/1.1 404 Not Found"));
    client.println(F("Content-Type: text/html"));
    client.println(keepAlive ? F("Connection: keep-alive") : F("Connection: close"));
    client.println();
    client.println(F("<html><body style='margin:0; padding:16px; background:#1e1e1e; color:#e0e0e0; font-family:Arial,sans-serif;'><h2>index.htm not found on SD card</h2><p>Place the web page on the SD card to enable the dark-mode interface.</p></body></html>"));
    return;
  }

  // File is open. Switch back to Ethernet to send the 200 headers before streaming.
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
  ActiveSD();
  html.close();
  activeEthernet();
}

// ================= SD DIAGNOSTIC =================
// Call once from setup() to print a full SD health report to Serial.
// Checks: SD.begin(), root file listing, and direct open of "/index.htm".
void diagSDInit() {
  Serial.println(F("\n===== SD DIAGNOSTIC ====="));

  if (!sdReady) {
    Serial.println(F("SD not ready — SD.begin() failed in checkSDCardStatus, skipping."));
    Serial.println(F("========================="));
    return;
  }

  ActiveSD();
  delayMicroseconds(100); // settle after SPI bus switch

  // List every entry in root so we can see exact filenames as stored on FAT.
  Serial.println(F("Root directory contents:"));
  File root = SD.open("/");
  if (!root) {
    Serial.println(F("  (could not open root)"));
  } else {
    uint8_t count = 0;
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      Serial.print(F("  ["));
      Serial.print(entry.isDirectory() ? F("DIR") : F("FILE"));
      Serial.print(F("] "));
      Serial.print(entry.name());
      if (!entry.isDirectory()) {
        Serial.print(F("  "));
        Serial.print(entry.size());
        Serial.println(F(" bytes"));
      } else {
        Serial.println();
      }
      entry.close();
      count++;
    }
    root.close();
    if (count == 0) Serial.println(F("  (empty — no files found)"));
  }

  // Try to open the exact file main.cpp expects.
  File f = SD.open("/index.htm", FILE_READ);
  Serial.print(F("SD.open(\"/index.htm\"): "));
  if (f) {
    Serial.print(F("OK  size="));
    Serial.print(f.size());
    Serial.println(F(" bytes"));
    f.close();
  } else {
    Serial.println(F("FAILED (file not found)"));
  }

  activeEthernet();
  Serial.println(F("========================="));
}

// ================= SETUP =================
void setup() {

  // Force SPI master mode and ensure both CS lines are de-selected
  pinMode(10, OUTPUT);         // Ethernet CS (W5500)
  digitalWrite(10, HIGH);
  pinMode(4, OUTPUT);          // SD CS
  digitalWrite(4, HIGH);

  // Motor direction outputs — both LOW (stopped) at startup
  pinMode(PIN_LED_motCW, OUTPUT);
  digitalWrite(PIN_LED_motCW, LOW);
  pinMode(PIN_LED_motCCW, OUTPUT);
  digitalWrite(PIN_LED_motCCW, LOW);

  // Hardware Serial is the debug/status channel over USB.
  Serial.begin(DEBUG_BAUD);

  // Modbus RTU is on SoftwareSerial (pins 9 RX, 8 TX).
  modbusSerial.begin(MODBUS_BAUD);
  motor.begin(MOTOR_SLAVE_ID, modbusSerial);

  delay(500); // Delay to let hardware power up before checks

  // Check Ethernet shield status (prints to hardware serial)
  EthernetStatus ethStatus = checkEthernetStatus(Serial);

  // Check SD card status (prints to hardware serial)
  SDCardStatus sdStatus = checkSDCardStatus(Serial);

  // SD diagnostic: prints SD health + exact filename list to Serial monitor.
  // Must run before createtxt() so it sees clean SD state right after SD.begin().
  diagSDInit();

  // Write the same status output to the SD card file
  createtxt(ethStatus, sdStatus);

  delay(500); // Delay to ensure SD file operations complete before starting the server
  activeEthernet();
  // Start web server only after Ethernet is initialized
  server.begin();

  //setupMotorDrive();
}

void loop() {
  // server.available() returns a client only when data is already buffered in the
  // W5100 receive FIFO. This guarantees handleClient() always has data ready on entry
  // and avoids the ERR_CONNECTION_RESET that occurs when accept() hands off the socket
  // before the browser has finished sending its request line.
  EthernetClient client = server.available();
  if (client) {
    handleClient(client);
  }

  // Critical: refresh 0x4001/0x4002 faster than 50 ms while moving.
  //serviceMotorJog();
  serviceMotorTest();
}
