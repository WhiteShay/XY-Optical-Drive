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
 * * *  - Test Modbus communication with motor drivers. - IN PROGRESS
 * *  - V1.0.2: Homing + 0 definition - TO DO
 * *  - V1.0.3: Move CW/CCW + position increment - TO DO
 * 27/04/2026 ==> 15/05/2026 - V1.0.3->V1.0.7 - Finalize motor control and testing
 * *  - V1.0.3: Speed switch. - TO DO
 * *  - V1.0.4: Emergency Stop. - TO DO
 * *  - V1.0.5: Save position - TO DO
 * *  - V1.0.6: Go to position mode - TO DO
 * 
 */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <ModbusMaster.h>
#include <EEPROM.h>

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
bool sdReady = false; // Set to true after successful SD.begin() during setup


// ================= MOTOR CONTROL PINS =================
// Pin 4 is reserved as SD card CS on the Ethernet Shield — do not use it here.
const uint8_t PIN_MOTOR_CW  = 2;  // DO: Clockwise direction output
const uint8_t PIN_MOTOR_CCW = 5;  // DO: Counter-clockwise direction output

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

void handleClient(EthernetClient client);
void sendStatusResponse(EthernetClient client);
void sendHWStatusResponse(EthernetClient client);
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

        out.println(F("Remaining space: Unknown"));
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
    file.println(F("Remaining space: Unknown"));
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
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println();
  client.print(F("{\"uptime\":"));
  client.print(millis());
  client.println(F("}"));
}

void sendHWStatusResponse(EthernetClient client) {
  ActiveSD();
  if (!sdReady) {
    activeEthernet();
    client.println(F("HTTP/1.1 503 Service Unavailable"));
    client.println(F("Content-Type: text/plain"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("SD not available"));
    return;
  }
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/plain"));
  client.println(F("Connection: close"));
  client.println();
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
  }
  ActiveSD();
  txt.close();
  activeEthernet();
}

// Serve index.htm from SD card.
// Copy Web/index.html to the SD card as index.htm (SD library uses 8.3 filenames).
void sendHTMLPage(EthernetClient client) {
  ActiveSD();
  if (!sdReady) {
    activeEthernet();
    client.println(F("HTTP/1.1 503 Service Unavailable"));
    client.println(F("Content-Type: text/html"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("<html><body style='margin:0; padding:16px; background:#1e1e1e; color:#e0e0e0; font-family:Arial,sans-serif;'><h2>SD not available</h2><p>The web page could not be loaded because the SD card is not ready.</p></body></html>"));
    return;
  }
  File html = SD.open("index.htm", FILE_READ);
  if (!html) {
    activeEthernet();
    client.println(F("HTTP/1.1 404 Not Found"));
    client.println(F("Content-Type: text/html"));
    client.println(F("Connection: close"));
    client.println();
    client.println(F("<html><body style='margin:0; padding:16px; background:#1e1e1e; color:#e0e0e0; font-family:Arial,sans-serif;'><h2>index.htm not found on SD card</h2><p>Place the web page on the SD card to enable the dark-mode interface.</p></body></html>"));
    return;
  }
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html"));
  client.println(F("Connection: close"));
  client.println();
  uint8_t buf[32];
  while (true) {
    ActiveSD();
    if (!html.available()) break;
    int n = html.read(buf, sizeof(buf));
    activeEthernet();
    if (n > 0) client.write(buf, n);
  }
  ActiveSD();
  html.close();
  activeEthernet();
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