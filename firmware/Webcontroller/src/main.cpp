/*
 * XY Optical Drive_V0.1.0.ino
 * * DESCRIPTION:
 * Reliable Lab Automation Controller for Arduino Uno + Ethernet Shield + Stepper Motors NEMA 17.
 *  * * HARDWARE STACK:
 * 1. Arduino Uno R3 (Bottom)
 * 2. Ethernet Shield W5100/W5500 (Middle) - Uses SPI bus over Pin 10.
 * 3. Modbus RTU RS485 hardware converter (Top) - Uses SoftwareSerial on Pins 8 (RX) and 9 (TX).
 * 4. Integrated Stepper Motor Drivers (e.g., A4988) connected to PC, drive via Modbus communication for step/dir and power.
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
 * * 23/03/2026 ==> 03/04/2026 - V0.1.0 - Motor control development
 * * - Modbus library integration for stepper motor control - IN PROGRESS
 * * - Webpage buttons to control motor direction and stop - IN PROGRESS => Halted, needs RS485 hardware and testing.
 * * - SD BUG: SD card not initializing on first attempt, needs multiple retries and SPI reset - IN PROGRESS 
 * * 06/04/2026 ==> 10/04/2026 - V0.1.0 - Motor control development => Integration and debugging.
 * * - SD debugging and reliability improvements - DONE-
 * * - Motor control Modbus Library choice and integration - IN PROGRESS 
 * * 13/04/2026 ==> 17/04/2026 - V1.1.0 - Motor control development => Integration and debugging.
 * * - SD HTML handling
 * * - Simple Motor control via Modubus RTU over RS485 (hardware testing and debugging) - IN PROGRESS
 * * - Motor control algorithm.
 * */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <ModbusMaster.h>

// ================= NETWORK CONFIGURATION =================//
//                    MAC Address: 
// Specific to your shield. If you change shields, update this.
byte mac[6] = { 0xA8, 0x61, 0x0A, 0xAE, 0x1C, 0xE4 };

//                    IP Configuration:
// Currently set to my PC ethernet settings (PC Ip : 169.254.43.137, local).
IPAddress ip(169, 254, 43, 138);
// IPAddress gateway(169, 254, 43, 1);
IPAddress subnet(255, 255, 0 , 0);
// IPAddress myDns(8, 8, 8, 8); // Not strictly used for local control

// ================= WEB SERVER =================//
EthernetServer server(80);

// ================= MOTOR CONTROL PINS =================//
// Pin 4 is reserved as SD card CS on the Ethernet Shield — do not use it here.
const uint8_t PIN_MOTOR_CW  = 2;  // DO: Clockwise direction output
const uint8_t PIN_MOTOR_CCW = 5;  // DO: Counter-clockwise direction output

// ================= GLOBAL STATE =================//
bool gSdReady = false;
bool gEthReady = false;

// ================= HELPER STRUCTURES =================//
// Status result structures (small, minimal RAM) for logging to SD file
struct EthernetStatus {
  bool detected;
  int hwStatus;
  uint8_t cardType; // 1=SD1, 2=SD2, 3=SDHC, 0=Unknown
  bool linkOn;
};

struct SDCardStatus {
  bool detected;
  bool initialized;
  uint8_t cardType; // 1=SD1, 2=SD2, 3=SDHC, 0=Unknown
  int fatType;
  uint32_t volumeKB;
  uint32_t freeKB;
  bool freeSpaceKnown;
};

// ================= HELPER FUNCTIONS =================//
//                   Function Declarations:
// Helpers to report hardware status to any Print target (Serial or File)
EthernetStatus checkEthernetStatus(Print &out);
SDCardStatus checkSDCardStatus(Print &out);

void createtxt(const EthernetStatus &eth, const SDCardStatus &sd); // Create .txt file to store hardware setup results on SD card
void verifyAndClearTxtFile(); //Used to erase the content of the SD .txt file.
void ActiveSD(); // Activate SD card on SPI bus (de-select Ethernet, select SD)
void activeEthernet(); // Activate Ethernet on SPI bus (de-select SD, select Ethernet) - not currently used but can be useful for future development of HTML page to display status and control motors.
bool getSDFreeSpaceKB(const SdVolume &volume, uint32_t &freeKB);
void printIPAddress(Print &out, const IPAddress &address);
void printSPIPinStatus(Print &out, const __FlashStringHelper *label);

void handleClient(EthernetClient client);
void sendStatusResponse(EthernetClient client);
void sendHardwareStatusResponse(EthernetClient client);
void sendHTMLPage(EthernetClient client);

void printSPIPinStatus(Print &out, const __FlashStringHelper *label) {
  struct PinDescription {
    uint8_t pin;
    const __FlashStringHelper *name;
  };
  // Identification of the pins for SPI(SD)/ETHERNET handling
  const PinDescription pins[] = {
    {4, F("SD_CS")},
    {10, F("ETH_CS")},
    {11, F("MOSI")},
    {12, F("MISO")},
    {13, F("SCK")}
  };

  out.println();
  out.print(F("--- SPI Pin Status: "));
  out.print(label);
  out.println(F(" ---"));

  for (uint8_t index = 0; index < (sizeof(pins) / sizeof(pins[0])); index++) {
    const uint8_t pin = pins[index].pin;
    const uint8_t port = digitalPinToPort(pin);

    out.print(pins[index].name);
    out.print(F(" (D"));
    out.print(pin);
    out.print(F("): mode="));

    if (port == NOT_A_PIN) {
      out.print(F("UNKNOWN"));
    } else {
      volatile uint8_t *modeRegister = portModeRegister(port);
      const uint8_t bitMask = digitalPinToBitMask(pin);
      out.print((*modeRegister & bitMask) ? F("OUTPUT") : F("INPUT"));
    }

    out.print(F(", level="));
    out.println(digitalRead(pin) == HIGH ? F("1") : F("0"));
  }
}

EthernetStatus checkEthernetStatus(Print &out) {
  EthernetStatus status = {};

  out.println(F("\n--- Ethernet Communication Status ---"));

  activeEthernet(); // Ensures SD is de-selected (HIGH) and Ethernet CS is active (LOW)
  delay(100); // Delay to allow hardware to stabilize before checking status
  Ethernet.begin(mac, ip, subnet); // Initialize Ethernet with manually defined MAC address and IP

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
    status.cardType = 0;
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

    // Record card type
    switch (Ethernet.hardwareStatus()) {
      default:
        status.cardType = 0;
        break;
    }
  }

  return status;
}

SDCardStatus checkSDCardStatus(Print &out) {
  SDCardStatus status = {};
  out.println(F("\n--- SD Card Status ---"));
  //printSPIPinStatus(out, F("Before ActiveSD"));
  
  // delay(500);
  ActiveSD();
  delay(100);

  //printSPIPinStatus(out, F("After ActiveSD"));
  
  if (!gSdReady) {
    gSdReady = SD.begin(4);  // Retry SD init once if it failed during setup
  }

  // Check if SD card is present and can be initialized
  if (!gSdReady) {
    out.println(F("SD card: NOK"));
    out.println(F("Possible reasons:"));
    out.println(F("- No SD card inserted"));
    out.println(F("- SD card not formatted properly"));
    out.println(F("- SPI conflict with Ethernet shield"));
    status.detected = false;
    status.initialized = false;
    activeEthernet();
    return status;
  }

  status.detected = true;
  status.initialized = true;
  out.println(F("SD card: OK"));

  // Get card information from the already-initialized SD library internals.
  // Do NOT create a new Sd2Card or call card.init() here — that would
  // re-send the CMD0/CMD8/ACMD41 reset sequence and corrupt the SD
  // library's internal state, causing subsequent file operations to fail.
  Sd2Card *card = SdVolume::sdCard();

  if (card) {
    switch (card->type()) {
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
        break;
    }

    // Init a SdVolume from the existing card to read partition/FAT info.
    // This only reads sectors — it does NOT re-initialize the card hardware.
    SdVolume volume;
    if (volume.init(*card)) {
      status.fatType = volume.fatType();
      out.print(F("Volume type: FAT"));
      out.println(status.fatType);

      uint32_t volumesize = volume.blocksPerCluster();
      volumesize *= volume.clusterCount();
      volumesize /= 2;  // SD card blocks are always 512 bytes (1/2 KB)
      status.volumeKB = volumesize;

      out.print(F("Volume size: "));
      out.print(status.volumeKB);
      out.println(F(" KB"));

      status.freeSpaceKnown = getSDFreeSpaceKB(volume, status.freeKB);
      if (status.freeSpaceKnown) {
        out.print(F("Remaining space: "));
        out.print(status.freeKB);
        out.println(F(" KB"));
      } else {
        out.println(F("Remaining space: Unknown"));
      }
    }
  }

  activeEthernet();
  return status;
}

void ActiveSD() {
  // De-select Ethernet (shield) so SD can use the SPI bus
  digitalWrite(10, HIGH); // Ensure Ethernet CS is HIGH (de-selected)
  digitalWrite(4, LOW);  // SD CS is active LOW
}

void activeEthernet() {
  // De-select SDso Ethernet (shield)  can use the SPI bus
  digitalWrite(4, HIGH); // Ensure SD CS is HIGH (de-selected)
  digitalWrite(10, LOW);  // Ethernet CS is active (LOW)
}

bool getSDFreeSpaceKB(const SdVolume &volume, uint32_t &freeKB) {
  // Walk the FAT table to estimate remaining free space on the mounted SD volume.
  Sd2Card *card = SdVolume::sdCard();
  if (!card) {
    return false;
  }

  const uint8_t fatType = volume.fatType();
  if (fatType != 16 && fatType != 32) {
    return false;
  }

  uint8_t buffer[512];
  uint32_t cachedBlock = 0xFFFFFFFFUL;
  uint32_t freeClusters = 0;
  const uint32_t lastCluster = volume.clusterCount() + 1;

  for (uint32_t cluster = 2; cluster <= lastCluster; cluster++) {
    uint32_t blockNumber = volume.fatStartBlock();
    if (fatType == 16) {
      blockNumber += cluster >> 8;
    } else {
      blockNumber += cluster >> 7;
    }

    if (blockNumber != cachedBlock) {
      if (!card->readBlock(blockNumber, buffer)) {
        return false;
      }
      cachedBlock = blockNumber;
    }

    uint32_t entryValue;
    if (fatType == 16) {
      uint16_t offset = (cluster & 0xFF) * 2;
      entryValue = buffer[offset] | (static_cast<uint16_t>(buffer[offset + 1]) << 8);
    } else {
      uint16_t offset = (cluster & 0x7F) * 4;
      entryValue = static_cast<uint32_t>(buffer[offset]);
      entryValue |= static_cast<uint32_t>(buffer[offset + 1]) << 8;
      entryValue |= static_cast<uint32_t>(buffer[offset + 2]) << 16;
      entryValue |= static_cast<uint32_t>(buffer[offset + 3]) << 24;
      entryValue &= 0x0FFFFFFFUL;
    }

    if (entryValue == 0) {
      freeClusters++;
    }
  }

  freeKB = (freeClusters * volume.blocksPerCluster()) / 2;
  return true;
}

void printIPAddress(Print &out, const IPAddress &address) {
  // Print an IP address in dotted decimal format to any Print target.
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
    file.print(F("Remaining space: "));
    if (sd.freeSpaceKnown) {
      file.print(sd.freeKB);
      file.println(F(" KB"));
    } else {
      file.println(F("Unknown"));
    }
  }
}

void createtxt(const EthernetStatus &eth, const SDCardStatus &sd) {
  Serial.println(F("\n--- SD Card Create .txt Test ---"));

  if (!gSdReady) {
    Serial.println(F("Cannot create file - SD not initialized"));
    return;
  }

  ActiveSD();

  // Overwrite existing file so status is fresh each run
  SD.remove("test.txt");

  File file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to open test.txt for writing"));
    activeEthernet();
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

  Serial.println(F("test.txt created/updated successfully"));
}

void verifyAndClearTxtFile() {
  Serial.println(F("\n--- Verifying SD Card .txt File ---"));

  if (!gSdReady) {
    Serial.println(F("Cannot verify file - SD not initialized"));
    return;
  }

  ActiveSD();

  // Check if the file exists
  if (!SD.exists("test.txt")) {
    Serial.println(F("test.txt does not exist"));
    activeEthernet();
    return;
  }

  Serial.println(F("test.txt exists"));

  // Open the file to check if it's empty
  File file = SD.open("test.txt", FILE_READ);
  if (!file) {
    Serial.println(F("Failed to open test.txt for reading"));
    activeEthernet();
    return;
  }

  // Check if the file is empty
  if (file.size() == 0) {
    Serial.println(F("test.txt is empty"));
    file.close();
    activeEthernet();
    return;
  }

  Serial.println(F("test.txt is not empty, clearing content..."));
  file.close();

  // Open the file in write mode to clear it (this truncates the file)
  file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to open test.txt for clearing"));
    activeEthernet();
    return;
  }

  // Since we opened in FILE_WRITE without writing anything, the file is now empty
  file.close();
  activeEthernet();

  Serial.println(F("test.txt content cleared successfully"));
}

// ================= WEB SERVER FUNCTIONS =================//
void handleClient(EthernetClient client) {
  // Route a single HTTP request to the matching status, control, or page handler.
  char request[128] = {0}; // Fixed buffer for HTTP request line
  size_t index = 0;
  while (client.available() && index < sizeof(request) - 1) {
    char c = client.read();
    request[index++] = c;
    if (c == '\n') break;
  }
  request[index] = '\0'; // Null terminate

  if (strstr(request, "GET /status ")) {
    sendStatusResponse(client);
  } else if (strstr(request, "GET /hwstatus ")) {
    sendHardwareStatusResponse(client);
  } else if (strstr(request, "GET /cw ")) {
    digitalWrite(PIN_MOTOR_CW, HIGH);
    digitalWrite(PIN_MOTOR_CCW, LOW);
    client.println(F("HTTP/1.1 200 OK"));
    client.println();
  } else if (strstr(request, "GET /ccw ")) {
    digitalWrite(PIN_MOTOR_CCW, HIGH);
    digitalWrite(PIN_MOTOR_CW, LOW);
    client.println(F("HTTP/1.1 200 OK"));
    client.println();
  } else if (strstr(request, "GET /stop ")) {
    digitalWrite(PIN_MOTOR_CW, LOW);
    digitalWrite(PIN_MOTOR_CCW, LOW);
    client.println(F("HTTP/1.1 200 OK"));
    client.println();
  } else if (strstr(request, "GET / ")) {
    sendHTMLPage(client);
  } else {
    client.println(F("HTTP/1.1 404 Not Found"));
    client.println();
  }
  client.stop();
}

void sendStatusResponse(EthernetClient client) {
  // Return a minimal JSON status payload for external monitoring or debugging.
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println();
  client.print(F("{\"uptime\":"));
  client.print(millis());
  client.println(F("}"));
}

void sendHardwareStatusResponse(EthernetClient client) {
  // Return hardware status from SD card as plain text.
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/plain"));
  client.println();

  // Read test.txt from SD and stream it directly to client
  ActiveSD();
  if (!gSdReady) {
    activeEthernet();
    client.println(F("SD card not available"));
  } else {
    File txt = SD.open("test.txt", FILE_READ);
    if (!txt) {
      activeEthernet();
      client.println(F("test.txt not found"));
    } else {
      uint8_t buf[32];
      while (txt.available()) {
        int n = txt.read(buf, sizeof(buf));
        if (n > 0) client.write(buf, n);
      }
      txt.close();
      activeEthernet();
    }
  }
}

void sendHTMLPage(EthernetClient client) {
  // Render the firmware-served control page with status text and motor buttons.
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();

  client.println(F("<!DOCTYPE html>"));
  client.println(F("<html><head><meta charset=\"UTF-8\">"));
  client.println(F("<title>XY Optical Drive V0.1.0</title></head>"));
  client.println(F("<body style='font-family:Arial, sans-serif;padding:16px;'>"));
  client.println(F("<h1>XY Optical Drive</h1>"));
  client.println(F("<p><strong>Version:</strong> V0.1.0</p>"));
  client.println(F("<p>Page generated directly by firmware.</p>"));
  client.println(F("<p>API status endpoint: <a href='/status'>/status</a></p>"));
  client.print(F("<p><em>Uptime: "));
  client.print(millis() / 1000UL);
  client.println(F(" s</em></p>"));

  // --- Hardware Setup Report (test.txt) ---
  client.println(F("<hr><h2>Hardware Setup Report</h2>"));
  client.println(F("<pre style='background:#f4f4f4;padding:12px;border:1px solid #ccc;white-space:pre-wrap;'>"));
  client.println(F("(Use /hwstatus endpoint to fetch hardware report separately)"));
  client.println(F("</pre>"));

  // --- Motor Control ---
  client.println(F("<hr><h2>Motor Control</h2>"));
  client.println(F("<style>.mb{font-size:1.2em;padding:10px 20px;border:none;border-radius:6px;cursor:pointer;background:#e0e0e0;margin-right:4px;}</style>"));
  client.println(F("<div style='margin:12px 0;'>"));
  client.println(F("<button id='b-ccw' class='mb' onclick='cmd(\"ccw\")'>&#9664; CCLKW</button>"));
  client.println(F("<button id='b-stop' class='mb' onclick='cmd(\"stop\")'>&#9646;&#9646; Stop</button>"));
  client.println(F("<button id='b-cw' class='mb' onclick='cmd(\"cw\")'>CLKW &#9654;</button>"));
  client.println(F("</div>"));
  client.println(F("<p id='ms'>Status: Stopped</p>"));
  client.println(F("<script>"));
  client.println(F("var L={cw:'Running CW',ccw:'Running CCLKW',stop:'Stopped'};"));
  client.println(F("var B=['cw','ccw','stop'];"));
  client.println(F("function cmd(d){fetch('/'+d).then(function(){"));
  client.println(F("document.getElementById('ms').textContent='Status: '+L[d];"));
  client.println(F("B.forEach(function(x){var b=document.getElementById('b-'+x);"));
  client.println(F("b.style.background=x===d?'#4CAF50':'#e0e0e0';"));
  client.println(F("b.style.color=x===d?'white':'black';});});}"));
  client.println(F("</script>"));

  client.println(F("</body></html>"));
}

// ================= SETUP =================//
void setup() {
  pinMode(10, OUTPUT);   // Ethernet CS
  digitalWrite(10, HIGH); // Ensure Ethernet is de-selected at startup

  pinMode(4, OUTPUT);    // SD CS
  digitalWrite(4, HIGH); // Ensure SD is de-selected at startup

  pinMode(PIN_MOTOR_CW, OUTPUT);
  digitalWrite(PIN_MOTOR_CW, LOW); // Ensure "motor" CW is stopped at startup
  pinMode(PIN_MOTOR_CCW, OUTPUT);
  digitalWrite(PIN_MOTOR_CCW, LOW);

  Serial.begin(9600);
  delay(1000);

  Serial.println(F("Arduino startup"));

  // ---------- Ethernet init ----------//
  activeEthernet();
  delay(100);
  Ethernet.begin(mac, ip, subnet);
  server.begin();
  gEthReady = true;

  Serial.println(F("Ethernet init done"));
  delay(100);
  // ---------- SD init ----------//
  // The W5100/W5500 can hold the SPI bus after Ethernet.begin().
  // Force it to release by toggling CS lines and resetting SPI.
  digitalWrite(10, HIGH);   // force Ethernet CS high (deselect)
  digitalWrite(4, HIGH);    // SD CS high (deselect)
  
  ActiveSD(); // Select SD to ensure it can initialize without Ethernet interference
  delay(100);

  // Try SD init with multiple attempts — some cards need retries
  for (uint8_t attempt = 1; attempt <= 3 && !gSdReady; attempt++) {
    Serial.print(F("SD init attempt "));
    Serial.println(attempt);
    gSdReady = SD.begin(4);
    if (!gSdReady) {
      digitalWrite(4, HIGH);   // deselect SD between attempts
      delay(200 * attempt);    // increasing delay between retries
      ActiveSD();
      delay(50);
    }
  }

  if (gSdReady) {
    Serial.println(F("SD init OK"));
  } else {
    Serial.println(F("SD init FAILED"));
    Serial.println(F("Check: card inserted? FAT16/FAT32 format? contacts clean?"));
  }

  activeEthernet();

  EthernetStatus ethStatus = checkEthernetStatus(Serial);
  SDCardStatus sdStatus = checkSDCardStatus(Serial);

  if (gSdReady) {
    createtxt(ethStatus, sdStatus);
  } else {
    Serial.println(F("Skipping test.txt creation because SD is not ready"));
  }

  Serial.println(F("Status check complete."));

}

void loop() {
  // Process incoming Ethernet clients one request at a time.
  // Handle web clients first
  EthernetClient client = server.available();
  if (client) {
    handleClient(client);
    return;
  }
}