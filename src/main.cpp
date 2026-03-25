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
 * * - Modbus library integration for stepper motor control - IN PROGRESS
 * * - Webpage buttons to control motor direction and stop - IN PROGRESS
 */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>

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


// ================= MOTOR CONTROL PINS =================
// Pin 4 is reserved as SD card CS on the Ethernet Shield — do not use it here.
const uint8_t PIN_MOTOR_CW  = 2;  // DO: Clockwise direction output
const uint8_t PIN_MOTOR_CCW = 5;  // DO: Counter-clockwise direction output

// ================= HELPER STRUCTURES =================
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

// ================= HELPER FUNCTIONS =================
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

void handleClient(EthernetClient client);
void sendStatusResponse(EthernetClient client);
void sendHTMLPage(EthernetClient client);

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
    out.println(F("SD card: DETECTED and INITIALIZED"));

    // Get card information
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

bool getSDFreeSpaceKB(const SdVolume &volume, uint32_t &freeKB) {
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

  ActiveSD();

  // Ensure SD is initialized (assumes CS already set correctly)
  if (!SD.begin(4)) {
    Serial.println(F("Cannot create file - SD not initialized"));
    return;
  }

  // Overwrite existing file so status is fresh each run
  SD.remove("test.txt");

  File file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to open test.txt for writing"));
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

  ActiveSD();

  // Ensure SD is initialized
  if (!SD.begin(4)) {
    Serial.println(F("Cannot verify file - SD not initialized"));
    return;
  }

  // Check if the file exists
  if (!SD.exists("test.txt")) {
    Serial.println(F("test.txt does not exist"));
    return;
  }

  Serial.println(F("test.txt exists"));

  // Open the file to check if it's empty
  File file = SD.open("test.txt", FILE_READ);
  if (!file) {
    Serial.println(F("Failed to open test.txt for reading"));
    return;
  }

  // Check if the file is empty
  if (file.size() == 0) {
    Serial.println(F("test.txt is empty"));
    file.close();
    return;
  }

  Serial.println(F("test.txt is not empty, clearing content..."));
  file.close();

  // Open the file in write mode to clear it (this truncates the file)
  file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to open test.txt for clearing"));
    return;
  }

  // Since we opened in FILE_WRITE without writing anything, the file is now empty
  file.close();

  Serial.println(F("test.txt content cleared successfully"));
}

// ================= WEB SERVER FUNCTIONS =================
void handleClient(EthernetClient client) {
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
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println();
  client.print(F("{\"uptime\":"));
  client.print(millis());
  client.println(F("}"));
}

void sendHTMLPage(EthernetClient client) {
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

  // Read test.txt from SD and stream it directly to client
  ActiveSD();
  bool sdReady = SD.begin(4);
  if (!sdReady) {
    activeEthernet();
    client.println(F("(SD not available)"));
  } else {
    File txt = SD.open("test.txt", FILE_READ);
    if (!txt) {
      activeEthernet();
      client.println(F("(test.txt not found)"));
    } else {
      uint8_t buf[32];
      while (true) {
        ActiveSD();
        if (!txt.available()) break;
        int n = txt.read(buf, sizeof(buf));
        activeEthernet();
        if (n > 0) client.write(buf, n);
      }
      ActiveSD();
      txt.close();
      activeEthernet();
    }
  }

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

// ================= SETUP =================
void setup() {

  // Force SPI master mode and ensure both CS lines are de-selected
  pinMode(10, OUTPUT);         // Ethernet CS (W5500)
  digitalWrite(10, HIGH);
  pinMode(4, OUTPUT);          // SD CS
  digitalWrite(4, HIGH);

  // Motor direction outputs — both LOW (stopped) at startup
  pinMode(PIN_MOTOR_CW, OUTPUT);
  digitalWrite(PIN_MOTOR_CW, LOW);
  pinMode(PIN_MOTOR_CCW, OUTPUT);
  digitalWrite(PIN_MOTOR_CCW, LOW);

    Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  Serial.println(F("Arduino Status Check Starting..."));
  delay(500); // Delay to let Hardware power up before check

  // Check SPI pin status (10-13) ==> test function to verify SPI communication and pin configuration before checking Ethernet and SD card status
  // printSPIPinStatus();

  // Check Ethernet shield status (prints to Serial)
  EthernetStatus ethStatus = checkEthernetStatus(Serial);

  // Start web server only after Ethernet is initialized
  server.begin();

  // Check SPI pin status (10-13)==> test funct
  //printSPIPinStatus();

  // Check SD card status (prints to Serial)
  SDCardStatus sdStatus = checkSDCardStatus(Serial);

  // Write the same status output to the SD card file
  createtxt(ethStatus, sdStatus);

  Serial.println(F("Status check complete."));
}

void loop() {
  // Handle web clients first
  EthernetClient client = server.available();
  if (client) {
    handleClient(client);
    return;
  }
}