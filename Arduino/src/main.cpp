/*
 * XY_Optical_Drive_V1_1_1.ino
 * * DESCRIPTION:
 * Reliable Lab Automation Controller for Arduino Uno + Ethernet Shield + Stepper Motors NEMA 17.
 *  * * HARDWARE STACK:
 * 1. Arduino Uno R3 (Bottom)
 * 2. Ethernet Shield W5100/W5500 (Middle) - Uses SPI (ICSP Header + Pin 10), SD CS = Pin 4
 * 3. Integrated Stepper Motor Drivers connected via Modbus RS485 (TTL->RS485 module P7156-1).
 *
 * * ================= REST API REFERENCE =================
 * * GET /status   : JSON (uptime, cached position, Modbus diagnostics, motor + homing state)
 * * GET /cw, /ccw : latch the JOG in that direction (refreshed every 30 ms
 * *                 until /stop). Once homed with limits active, each button
 * *                 becomes a GUARDED bounded move instead (CW -> Limit 2,
 * *                 CCW -> Limit 1); release = stop.
 * *                 Returns HTTP 409 while a homing is in progress.
 * * GET /stop     : immediate quick stop / E-stop (0x6002 = 0x0040). Aborts homing.
 * * GET /home     : start the two-pass torque homing sequence (see below)
 * * GET /goto?p=N : move to absolute position N (in pulses, HOMED frame).
 * *                 With limits active, N must be within [limit1, limit2]
 * *                 or the request is rejected (409 "out_of_limits").
 * * GET /speed?v=N: set the motion speed in rpm for JOG, guarded jogs and
 * *                 /goto (NOT the homing, fixed at 200 rpm). Accepted
 * *                 range 5-400 rpm; applied at the next motion command;
 * *                 rejected (409) while any motion is running.
 * *                 The web page converts mm -> pulses. Executed as a PR0
 * *                 RELATIVE move, so it works even though the drive never
 * *                 resets its raw feedback counter. Requires a completed
 * *                 homing. Returns HTTP 409 if not homed / jogging / busy.
 * * GET /hwstatus : content of test.txt (hardware report written at boot)
 * * GET /         : serves index.htm from the SD card
 * * ======================================================
 *
 * * ============ TWO-PASS TORQUE HOMING + TRAVEL MEASUREMENT (V1.0.9) ============
 * * PREREQUISITE: the homing TYPE is configured as "Torque Homing" in
 * * MotionStudio. Field-captured encoding: Pr8.10 (0x600A) = 0x000C, with
 * * bit0 = direction (0 = CCW, 1 = CW, manual 5.2.1 - confirmed on hardware).
 * * The firmware now WRITES bit0 to steer each pass, keeping base 0x000C.
 * * Sequence on GET /home:
 * *   1) write homing speeds 0x600F/0x6010 = 200 rpm
 * *   2) PASS 1: 0x600A = 0x000D (CW), trigger 0x6002 = 0x020. The motor
 * *      runs CW to the FAR hard stop. There, the raw feedback position is
 * *      captured as the PASS-1 REFERENCE ("first zero").
 * *   3) PASS 2: 0x600A = 0x000C (CCW), trigger again. The motor runs CCW
 * *      across the whole axis to the NEAR hard stop. There, BEFORE setting
 * *      the final zero, the firmware reads the position and computes
 * *      MAX TRAVEL = |raw_pass2 - raw_pass1| (feedback counts), exposed in
 * *      /status as "max_travel_counts" and shown on the page in mm.
 * *   4) Final zero at the CCW stop: 0x021 written to 0x6002 + HOME OFFSET
 * *      captured (raw counts). All positions and /goto targets are in this
 * *      HOMED frame: homed = raw - home_offset (0 at the CCW hard stop,
 * *      positive travel toward CW, up to ~max travel).
 * * Completion of each pass: 0x1003 polled every 200 ms - bit0 fault ->
 * * error; done when bit6 (homing OK) is set AND bit2 (running) is clear,
 * * after a 500 ms guard. The raw counter is NEVER reset by the drive.
 * *
 * * ============ UNITS (field-derived, V1.0.9 scale fix) ============
 * * TWO different units coexist on this drive:
 * *   - COMMAND pulses (PR positions, /goto targets): Pr0.08 = 10000 /rev.
 * *   - FEEDBACK counts (register 0x1014/0x1015):     65536 /rev.
 * * Field proof: goto 20 mm (40000 cmd pulses = 4 rev) -> feedback read
 * * 262138 ~= 4 x 65536; goto 120 mm (24 rev) -> 1572865 ~= 24 x 65536.
 * * The page converts mm <-> each unit with the right constant, and
 * * startMove() converts feedback counts -> command pulses (x10000/65536,
 * * 64-bit exact) before computing the relative delta.
 * * Safety: the STOP BUTTON IS THE ONLY ABORT (E-stop). No automatic
 * * timeout, no automatic abort on status-read failures (the firmware just
 * * keeps polling). JOG requests are rejected (409) during homing only to
 * * avoid sending conflicting commands to the drive.
 * * ======================================================
 *
 * * HISTORY / UPDATES:
 * * 03/03/2026 ==> 10/04/2026 - V0.1.0 - Initial development: Eth+SD checks,
 * *   test.txt, web server, HTML on SD, ModbusMaster integration, JOG CW/CCW.
 * * 11/06/2026 - V1.0.1 - SD / SPI bus fix (single SD.begin, 64-byte chunks).
 * * 12/06/2026 - V1.0.2 - JOG keep-alive watchdog + Modbus priority. Field
 * *   test: E2 timeouts froze the sketch 2 s each (ModbusMaster hard-coded
 * *   timeout) -> watchdog stopped the motor in a loop.
 * * 12/06/2026 - V1.0.3 - Watchdog removed. ModbusMaster replaced by built-in
 * *   minimal RTU master (80 ms timeout, RX purge before TX, 3.5-char
 * *   inter-frame silence, modbus_timeouts counter). VALIDATED ON HARDWARE.
 *
 * * 12/06/2026 - V1.0.4 - Torque homing step 1. Field test: config writes
 * *   failed ("init failed (code 0)", "homing speed write failed") and the
 * *   boot-time 3-attempt init loop spammed the log on every JOG press.
 * *   Root issues: (a) the failure log only printed the LAST write's code,
 * *   masking which write actually failed; (b) chained one-shot parameter
 * *   writes fail intermittently (drive briefly busy right after answering
 * *   a previous parameter write) while isolated JOG writes always pass.
 *
 * * 13/06/2026 - V1.0.6 - Explicit zero-set (0x6002 = 0x021) on homing OK +
 * *   minimal web page. Field test: the drive ACKs the zero-set but the raw
 * *   feedback register 0x1014/0x1015 STILL keeps the power-on frame.
 * * 12/06/2026 - V1.0.5 - Config-write retries (mbWriteConfigRegister, 3
 * *   attempts / 20 ms), init once (lazy, V1.0.3 style), per-write result
 * *   logging, Stop-only homing safety. HOMING VALIDATED ON HARDWARE, but
 * *   the position still displayed the pre-homing coordinate of the hard
 * *   stop (-626623 pulses): the drive's "homing OK" does not reset the
 * *   feedback position register 0x1014/0x1015.
 *
 * * 13/06/2026 - V1.0.7 - Homed coordinate frame + /goto (PR0 relative).
 * *   Field test: moves execute, but the page stayed locked on "moving":
 * *   reading 0x6002 never returned 0x0000 after path completion on this
 * *   drive, and chained READS frequently timed out (E2, mb-timeouts=291;
 * *   writes were already protected by retries since V1.0.5). Bonus: the
 * *   MotionStudio torque-homing encoding was captured: Pr8.10 = 0x000C.
 *
 * * 13/06/2026 - V1.0.8 - Move completion via 0x1003 bit2 + read retries
 * *   (mbReadRetry). Field test of /goto revealed the SCALE issue: target
 * *   20 mm -> display 131.069 mm, 120 -> 786.43, ratio 6.5536 = 65536/10000
 * *   exactly: the feedback register counts 65536/rev while commands are in
 * *   10000 pulses/rev. The MOTION was correct, the display and the /goto
 * *   delta mixed the two units.
 *
 * * 13/06/2026 - V1.0.9 - Two-pass homing (CW then CCW) + travel
 * *   measurement (max_travel_counts) + feedback/command unit fix.
 *
 * 13/06/2026 - V1.1.0 - Soft limits + guarded CCW/CW (bounded by Limits 1/2)
 * *   + flash diet (same features, same version): the build hit 104% of the
 * *   32256-byte flash. Removed: duplicated Serial/file hardware report
 * *   (now single-sourced via printHardwareReport), low-level Sd2Card /
 * *   SdVolume diagnostics (card type / FAT type / volume size), 64-bit
 * *   unit math (replaced by 32-bit shifts, drops __divdi3), duplicated
 * *   HTTP header literals (shared httpHeader helper), verbose log strings.
 *
 * 13/06/2026 - V1.1.1 - User-adjustable motion speed (5-400 rpm)
 * *   One speed (default 100 rpm, volatile) used by the raw JOGs, the
 * *   guarded jogs and /goto, set via GET /speed?v=N and reported in
 * *   /status as "speed_rpm". Pr6.00 (JOG velocity) is re-synced with one
 * *   retried write before the next JOG when the value changed; the PR0
 * *   speed (Pr9.03) is written on every move anyway. MANUAL CHECK: Pr6.00
 * *   documented range 0-5000 r/min (parameter table); Pr9.03 unit rpm,
 * *   the manual's own PR example writes 600 rpm - the 5-400 window is the
 * *   project's conservative envelope, well inside the drive's limits.
 * *  - SOFT LIMITS (firmware-side, HOMED frame): set when a homing
 * *    completes with a measured travel > 20 mm:
 * *      Limit 1 = 0 + 10 mm        (lower bound, near the zero/CCW stop)
 * *      Limit 2 = max travel - 10 mm (upper bound, near the far/CW stop)
 * *    /goto targets outside [limit1, limit2] are rejected. The limits live
 * *    in the Arduino (the homed frame is the only verified frame); the
 * *    drive's own soft-limit registers are NOT used.
 * *  - GUARDED CCW / CW (replace the raw JOGs once homed + limits active):
 * *    the 9600-baud link cannot interleave position reads with the < 50 ms
 * *    JOG refresh dead-man (one transaction = 21-25 ms), so instead of
 * *    "monitor and bounce", each jog button starts a PR0 move WHOSE TARGET
 * *    IS ITS LIMIT (CCW -> Limit 1, CW -> Limit 2) at the user speed:
 * *    overshoot is impossible by construction.
 * *      - releasing the button = /stop = E-stop (normal end, no error);
 * *      - holding to the end = motor stops exactly AT the limit, even with
 * *        the button still pressed, and "limit1_hit" / "limit2_hit" is
 * *        raised for the page's acknowledgeable warning pop-up;
 * *      - if the position is ALREADY beyond the limit (e.g. right after a
 * *        homing, position = 0 < Limit 1), the very same move brings the
 * *        motor BACK to the limit, button held or not, then raises the flag.
 * *    Before homing (or without limits), both buttons stay raw JOGs.
 * *  - TWO-PASS HOMING: pass 1 CW to the far stop (pass-1 reference
 * *    captured), pass 2 CCW to the near stop; MAX TRAVEL measured BEFORE
 * *    the final zero-set, exposed as "max_travel_counts" in /status and
 * *    displayed as "Max length" (mm) on the page. Final zero = CCW stop.
 * *    The firmware now writes the homing direction bit (0x600A bit0) using
 * *    the field-captured torque-homing base 0x000C.
 * *  - UNIT FIX: display conversions use 65536 counts/rev, /goto targets
 * *    stay in 10000 cmd pulses/rev, and startMove() converts the current
 * *    feedback position counts -> cmd pulses before computing the delta
 * *    (the previous code mixed units; harmless only right after a homing).
 * *  - MOVE COMPLETION (V1.0.8): serviceMove() no longer polls the 0x6002
 * *    read-back (never returned 0x0000 on this drive). It now polls the
 * *    motion status 0x1003 once per 200 ms: bit0 (fault) -> error 2,
 * *    bit2 ("running") cleared after the 500 ms start guard -> DONE.
 * *    The page then restores the "Go to position" button to its default
 * *    color and re-enables the Motor Control buttons (requested behavior:
 * *    it was already wired page-side, the firmware just never said "done").
 * *  - READ RETRIES: new mbReadRetry() (3 attempts / 20 ms, like the config
 * *    writes) used for ALL reads - position, homing poll, move poll,
 * *    Pr8.10 logging. Field tests showed a read chained right after another
 * *    transaction often times out (E2) while isolated ones pass; none of
 * *    these reads are in the 30 ms JOG hot path, which stays untouched.
 * *    mb_timeouts still counts every failed attempt (diagnostics).
 * *  - DOC: torque homing encoding, field-observed: Pr8.10 (0x600A) = 0x000C
 * *    (bits 2+3), bit0 = 0 -> CCW on this setup.
 * *  - HOMED FRAME: the raw feedback counter cannot be reset, so the firmware
 * *    captures it as home_offset when homing completes (homed = true) and
 * *    reports homed positions everywhere: /status "feedback_position" now
 * *    reads ~0 at the hard stop after a homing, exactly as required.
 * *    The offset is volatile: re-run the homing after any Arduino reset or
 * *    drive power cycle ("homed" goes back to false at every boot and at
 * *    every homing start).
 * *  - GO TO POSITION: new /goto?p=<pulses> endpoint + web input in mm.
 * *    The firmware reads the current raw position, computes
 * *    delta = target_homed - (raw - home_offset), loads PR0 as a RELATIVE
 * *    position move (0x6200 = 0x0041, position in 0x6201/0x6202, speed
 * *    0x6203 = 200 rpm, acc/dec 0x6204/0x6205 = 100 ms/krpm) and triggers
 * *    path 0 with 0x6002 = 0x010. Completion: motion status 0x1003 bit2
 * *    ("running") polled every 200 ms - move done when it clears (after a
 * *    500 ms start guard); drive fault (bit0) reported as an error.
 * *    Stop button = the only abort (E-stop), consistent with the homing.
 * *  - Modbus layer unchanged: mbTransaction() identical to the validated
 * *    V1.0.3; one-shot parameter writes via mbWriteConfigRegister().
 * *
 * * HOMING ERROR CODES (homing_error):
 * *   1 = homing speed write failed     2 = homing trigger write failed
 * *   3 = zero-set write failed         4 = drive fault during homing
 * *   5 = aborted by Stop (E-stop)      6 = position read failed after homing
 * * MOVE ERROR CODES (move_error):
 * *   1 = PR0 config/trigger write failed
 * *   2 = drive fault during the move
 * *   3 = aborted by Stop (E-stop)
 * *   4 = position read failed before the move
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

// ================= MODBUS RTU MASTER (built-in, identical to V1.0.3) =================
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

// FC06 with retries, for ONE-SHOT configuration writes only (RS485 enable,
// JOG speed/accel, homing speeds, homing trigger). Field tests showed that a
// parameter write can fail intermittently right after the drive answered a
// previous parameter write (drive briefly busy internally), while isolated
// writes (JOG refresh) always pass. Up to 3 attempts, 20 ms apart, gives the
// drive time to recover. NEVER used in the 30 ms JOG refresh path.
static const uint8_t MB_CFG_RETRIES  = 3;
static const uint8_t MB_CFG_PAUSE_MS = 20;

uint8_t mbReadHoldingRegisters(uint16_t reg, uint8_t count, uint16_t *out);

uint8_t mbWriteConfigRegister(uint16_t reg, uint16_t value) {
  uint8_t rc = MB_ERR_TIMEOUT;
  for (uint8_t i = 0; i < MB_CFG_RETRIES; i++) {
    rc = mbWriteSingleRegister(reg, value);
    if (rc == MB_OK) return MB_OK;
    delay(MB_CFG_PAUSE_MS);
  }
  return rc;  // code of the last failed attempt
}

// FC03 with retries, for ALL reads outside the 30 ms JOG hot path (position,
// homing/move status polls, Pr8.10 logging). Field tests: a read chained
// right after another transaction often times out (E2) while isolated reads
// pass - same drive-recovery behavior already handled for config writes.
uint8_t mbReadRetry(uint16_t reg, uint8_t count, uint16_t *out) {
  uint8_t rc = MB_ERR_TIMEOUT;
  for (uint8_t i = 0; i < MB_CFG_RETRIES; i++) {
    rc = mbReadHoldingRegisters(reg, count, out);
    if (rc == MB_OK) return MB_OK;
    delay(MB_CFG_PAUSE_MS);
  }
  return rc;  // code of the last failed attempt
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
static const uint16_t REG_TRIGGER      = 0x6002;  // Pr8.02: trigger register (homing, E-stop...)
static const uint16_t REG_MOTION_STATUS  = 0x1003;  // bit0 fault, bit2 running, bit6 homing OK
static const uint16_t REG_HOMING_MODE    = 0x600A;  // Pr8.10: homing mode. Field-captured
                                                    // torque-homing base = 0x000C (bits 2+3);
                                                    // bit0 = direction: 0 = CCW, 1 = CW
                                                    // (manual 5.2.1, confirmed on hardware).
static const uint16_t REG_HOMING_SPD_HIGH = 0x600F; // Pr8.15: homing high speed (rpm)
static const uint16_t REG_HOMING_SPD_LOW  = 0x6010; // Pr8.16: homing low speed (rpm)
static const uint16_t REG_PR0_MODE       = 0x6200;  // Pr9.00: PR0 mode (0x0041 = relative position)
static const uint16_t REG_PR0_POS_H      = 0x6201;  // Pr9.01: PR0 position, high 16 bits
static const uint16_t REG_PR0_POS_L      = 0x6202;  // Pr9.02: PR0 position, low 16 bits
static const uint16_t REG_PR0_SPEED      = 0x6203;  // Pr9.03: PR0 speed (rpm)
static const uint16_t REG_PR0_ACC        = 0x6204;  // Pr9.04: PR0 acceleration (ms/1000 rpm)
static const uint16_t REG_PR0_DEC        = 0x6205;  // Pr9.05: PR0 deceleration (ms/1000 rpm)
static const uint16_t REG_FEEDBACK_POS_H = 0x1014;  // Feedback position (high 16 bits), pulses
static const uint16_t REG_FEEDBACK_POS_L = 0x1015;  // Feedback position (low 16 bits), pulses

// Common values written to the command registers.
static const uint16_t RS485_ENABLE_VALUE = 0x0001;
static const uint16_t DIR_CW             = 0x0000;
static const uint16_t DIR_CCW            = 0x0001;
static const uint16_t JOG_ACCEL_VALUE    = 50;

// ---- User-adjustable motion speed (raw JOGs, guarded jogs, /goto) ----
// Manual check: Pr6.00 (JOG velocity) documented range 0-5000 r/min;
// Pr9.03 (PR velocity) unit rpm, manual example writes 600 rpm. The
// 5-400 rpm window is this project's own conservative envelope.
static const uint16_t SPEED_MIN_RPM     = 5;
static const uint16_t SPEED_MAX_RPM     = 400;
static const uint16_t SPEED_DEFAULT_RPM = 100;
static const uint16_t JOG_CW_CMD         = 0x4001;
static const uint16_t JOG_CCW_CMD        = 0x4002;
static const uint16_t TRIG_HOMING        = 0x0020;  // 0x6002 = 0x020 -> start homing
static const uint16_t TRIG_SET_ZERO      = 0x0021;  // 0x6002 = 0x021 -> set current point
                                                    // as zero position (manual 5.2.1)
static const uint16_t TRIG_PATH0         = 0x0010;  // 0x6002 = 0x010 -> trigger PR path 0
static const uint16_t PR0_MODE_REL_POS   = 0x0041;  // PR0: position mode, RELATIVE
static const uint16_t HOMING_MODE_TORQUE = 0x000C;  // field-captured MotionStudio value
static const uint16_t HOMING_DIR_CW_BIT  = 0x0001;  // 0x600A bit0: 1 = CW, 0 = CCW

// ---- Units (field-derived, see header) ----
static const int32_t CMD_PULSES_PER_REV      = 10000L;  // PR / /goto command unit (Pr0.08)
static const int32_t FEEDBACK_COUNTS_PER_REV = 65536L;  // 0x1014/0x1015 feedback unit

// Converts homed feedback counts (65536/rev) to command pulses (10000/rev)
// in PURE 32-bit math (no 64-bit libgcc routines): counts = rev*65536 + frac
// with an arithmetic shift, and frac * 10000 <= 6.6e8 < 2^31.
int32_t countsToCmdPulses(int32_t counts) {
  int32_t  rev  = counts >> 16;                 // floor, two's complement
  uint32_t frac = (uint32_t)counts & 0xFFFFUL;
  return rev * CMD_PULSES_PER_REV
       + (int32_t)((frac * (uint32_t)CMD_PULSES_PER_REV) >> 16);
}

// ---- Soft limits (HOMED frame, command pulses). Screw pitch 5 mm/rev. ----
static const int32_t  LIMIT_MARGIN_CMD     = 20000L;  // 10 mm * 10000 / 5
static const uint16_t TRIG_ESTOP         = 0x0040;  // 0x6002 = 0x040 -> emergency stop
static const uint16_t MOTION_BIT_FAULT     = 0x0001;  // 0x1003 bit0
static const uint16_t MOTION_BIT_RUNNING   = 0x0004;  // 0x1003 bit2: motor in motion
static const uint16_t MOTION_BIT_HOMING_OK = 0x0040;  // 0x1003 bit6

// Last Modbus result and cached feedback position (iCL-RS manual 4.3.4).
int32_t feedbackPositionPulses = 0;
bool feedbackPositionValid = false;
uint8_t lastModbusResult = 0;
uint32_t lastModbusMs = 0;

// Records the result of the last Modbus transaction (shown in /status).
// (Definition restored here: the V1.1.0 flash-diet splice had removed it.)
void recordModbusResult(uint8_t result) {
  lastModbusResult = result;
  lastModbusMs = millis();
}

// ================= JOG STATE / TIMING =================
enum MotorState { MOTOR_STOPPED, MOTOR_CW, MOTOR_CCW };

static const uint32_t JOG_REFRESH_MS   = 30;    // < 50 ms required by the drive
static const uint32_t POSITION_POLL_MS = 1000;  // feedback position poll period
                                                // (motor stopped, no homing, ONLY)

MotorState motorState = MOTOR_STOPPED;
uint32_t lastJogRefresh = 0;     // last 0x4001/0x4002 write to the drive
uint32_t lastPositionPoll = 0;   // last background position read
bool jogConfigured = false;      // true once setupMotorJog() has succeeded (lazy, on first JOG/homing)
uint16_t motorSpeedRpm = SPEED_DEFAULT_RPM;  // user speed: raw JOGs, guarded jogs, /goto
uint16_t jogSpeedWritten = 0;                // last value written to Pr6.00 (0 = never)

// ================= HOMED COORDINATE FRAME =================
// The drive NEVER resets its raw feedback counter (0x1014/0x1015) - neither
// the homing nor the 0x021 zero-set do it (field tests V1.0.5/V1.0.6). The
// firmware therefore captures the raw value at the hard stop when a homing
// completes, and reports/uses HOMED positions: homed = raw - homeOffsetPulses.
// Volatile: re-run the homing after every Arduino reset or drive power cycle.
int32_t homeOffsetPulses = 0;
bool homed = false;              // true only after a successful homing this session

// ================= HOMING STATE / TIMING =================
enum HomingState { H_IDLE, H_PASS1, H_PASS2_START, H_PASS2, H_DONE, H_ERROR };

static const uint16_t HOMING_SPEED_HIGH_RPM = 200;   // requested homing speed
static const uint16_t HOMING_SPEED_LOW_RPM  = 200;   // kept equal to high speed; lower it
                                                     // if a slow precision approach is wanted
static const uint32_t HOMING_POLL_MS        = 200;   // motion status poll period
static const uint32_t HOMING_OK_GUARD_MS    = 500;   // ignore a possibly latched "homing OK"
                                                     // bit right after triggering
static const uint32_t HOMING_PASS_DWELL_MS  = 500;   // settle time between pass 1 and pass 2

HomingState homingState = H_IDLE;
uint8_t  homingError = 0;            // see HOMING ERROR CODES in the header
uint32_t homingStartMs = 0;          // start of the CURRENT pass (guard reference)
uint32_t lastHomingPoll = 0;
int32_t  pass1OffsetRaw = 0;         // raw feedback counts at the CW (far) hard stop
int32_t  maxTravelCounts = 0;        // |pass2 - pass1| in feedback counts (0 = not measured)

bool homingBusy() {
  return homingState == H_PASS1 || homingState == H_PASS2_START || homingState == H_PASS2;
}

// ================= GO-TO-POSITION (PR0) STATE =================
enum MoveState { M_IDLE, M_MOVING, M_DONE, M_ERROR };

static const uint16_t MOVE_ACCDEC         = 100;   // PR0 acc/dec (ms/1000 rpm)
static const uint32_t MOVE_POLL_MS        = 200;   // 0x6002 completion poll period
static const uint32_t MOVE_START_GUARD_MS = 500;   // let the path start before
                                                   // accepting "0x0000 = done"

MoveState moveState = M_IDLE;
uint8_t  moveError = 0;              // see MOVE ERROR CODES in the header
int32_t  moveTargetHomed = 0;        // requested target (homed frame, pulses)
uint32_t moveStartMs = 0;
uint32_t lastMovePoll = 0;
uint8_t  moveGuardLimit = 0;         // 0 = normal /goto move, 1 = guarded CCW
                                     // (target = Limit 1), 2 = guarded CW
                                     // (target = Limit 2)

// ================= SOFT LIMITS (set at homing completion) =================
bool    limitsActive = false;        // true once a homing measured travel > 20 mm
int32_t limit1Cmd = 0;               // lower limit (cmd pulses) = 0 + 10 mm
int32_t limit2Cmd = 0;               // upper limit (cmd pulses) = max travel - 10 mm
bool    limit1Hit = false;           // guarded CCW reached Limit 1 -> page pop-up
bool    limit2Hit = false;           // guarded CW reached Limit 2 -> page pop-up

// ================= HELPER STRUCTURES =================
// Status result structures (small, minimal RAM) for logging to SD file
struct EthernetStatus {
  bool detected;
  int hwStatus;
  bool linkOn;
};

// ================= HELPER FUNCTIONS =================
//                   Function Declarations:
EthernetStatus checkEthernetStatus();
bool checkSDCardStatus();
void createtxt(const EthernetStatus &eth);
void printHardwareReport(Print &out, const EthernetStatus &eth);
void printIPAddress(Print &out, const IPAddress &address);

bool setupMotorJog();
void commandJog(MotorState dir);
void stopMotor();
void serviceMotorJog();
void servicePositionPoll();
void startHoming();
void serviceHoming();
const char *homingStateText();
void startMove(int32_t targetHomedPulses, uint16_t speedRpm, uint8_t guardLimit);
void serviceMove();
const char *moveStateText();

void handleClient(EthernetClient client);
void sendStatusResponse(EthernetClient client);
void sendHomingStatusResponse(EthernetClient client);
void sendMoveStatusResponse(EthernetClient client);
void sendBusyResponse(EthernetClient client, const __FlashStringHelper *reason);
void sendHWStatusResponse(EthernetClient client);
void sendHTMLPage(EthernetClient client);

const char CT_JSON[] PROGMEM = "application/json";
const char CT_TEXT[] PROGMEM = "text/plain";
const char CT_HTML[] PROGMEM = "text/html";

// Shared HTTP response header: every status / content-type string lives ONCE
// in flash (each F() literal is otherwise duplicated at every call site).
void httpHeader(EthernetClient &client, uint16_t code, const char *ctypePgm) {
  client.print(F("HTTP/1.1 "));
  client.println(code == 200 ? F("200 OK")
               : code == 409 ? F("409 Conflict")
               : code == 503 ? F("503 Service Unavailable")
                             : F("404 Not Found"));
  client.print(F("Content-Type: "));
  client.println((const __FlashStringHelper *)ctypePgm);
  client.println();
}

EthernetStatus checkEthernetStatus() {
  EthernetStatus status = {};
  delay(100);                      // let the shield power up
  Ethernet.begin(mac, ip, subnet); // signature left as is (IP issues intentionally ignored)
  status.hwStatus = Ethernet.hardwareStatus();
  status.detected = (status.hwStatus != EthernetNoHardware);
  status.linkOn = status.detected && (Ethernet.linkStatus() == LinkON);
  return status;
}

// Single FAT mount for the whole program. From here on the SD and Ethernet
// libraries share the SPI bus by driving their own CS on every transaction.
// (The previous low-level Sd2Card/SdVolume diagnostics - card type, FAT
// type, volume size - were removed in the V1.1.0 flash diet: they pulled in
// the SdVolume cluster math and a large set of report strings.)
bool checkSDCardStatus() {
  sdReady = SD.begin(4);
  return sdReady;
}

void printIPAddress(Print &out, const IPAddress &address) {
  for (uint8_t i = 0; i < 4; i++) {
    out.print(address[i], DEC);
    if (i < 3) out.print('.');
  }
}

// ONE hardware report, printed to Serial at boot AND into test.txt: each
// string now lives once in flash (the old duplicated Serial/file versions
// were a major flash cost - avr-gcc does not merge duplicate F() literals).
void printHardwareReport(Print &out, const EthernetStatus &eth) {
  out.println(F("--- Ethernet ---"));
  if (!eth.detected) {
    out.print(F("shield: NOT DETECTED, hw="));
    out.println(eth.hwStatus);
  } else {
    out.print(F("shield: OK, MAC "));
    for (int i = 0; i < 6; i++) {
      if (mac[i] < 16) out.print('0');
      out.print(mac[i], HEX);
      if (i < 5) out.print(':');
    }
    out.println();
    out.print(F("link: "));
    if (eth.linkOn) {
      out.print(F("UP, IP "));
      printIPAddress(out, Ethernet.localIP());
      out.println();
    } else {
      out.println(F("DOWN"));
    }
  }
  out.println(F("--- SD ---"));
  out.println(sdReady ? F("SD: OK (FAT mounted)") : F("SD: FAILED (card/format/CS=4)"));
}

void createtxt(const EthernetStatus &eth) {
  if (!sdReady) {
    Serial.println(F("No SD: test.txt skipped"));
    return;
  }
  SD.remove("test.txt");           // FILE_WRITE = append: remove to start fresh
  File file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("test.txt open failed"));
    return;
  }
  file.print(F("Uptime: "));
  file.print(millis());
  file.println(F(" ms"));
  printHardwareReport(file, eth);
  file.close();
  Serial.println(F("test.txt OK"));
}

bool readFeedbackPosition() {
  uint16_t regs[2];
  uint8_t result = mbReadRetry(REG_FEEDBACK_POS_H, 2, regs);
  recordModbusResult(result);
  if (result != MB_OK) {
    feedbackPositionValid = false;
    Serial.print(F("Pos read err: "));
    Serial.println(result, HEX);
    return false;
  }
  feedbackPositionPulses = (int32_t)(((uint32_t)regs[0] << 16) | (uint32_t)regs[1]);
  feedbackPositionValid = true;
  return true;
}

const char *motorStateText() {
  switch (motorState) {
    case MOTOR_CW:  return "cw";
    case MOTOR_CCW: return "ccw";
    default:        return "stopped";
  }
}

const char *homingStateText() {
  switch (homingState) {
    case H_PASS1:       return "pass1";
    case H_PASS2_START:
    case H_PASS2:       return "pass2";
    case H_DONE:        return "done";
    case H_ERROR:       return "error";
    default:            return "idle";
  }
}

// One-time motor link configuration, run LAZILY on the first JOG or homing
// request (V1.0.3 behavior: initialize once, then leave the link alone).
// Sequence per the drive operating procedure: ENABLE RS485 control first
// (Pr0.07 = 1), then the JOG speed/accel. Each write logs ITS OWN result
// code. jogConfigured is gated on the enable only: speed/accel have
// drive-stored fallback values, so a failure there is a warning, not a
// blocker (and does not cause endless re-initialization).
bool setupMotorJog() {
  uint8_t rEnable = mbWriteConfigRegister(REG_RS485_ENABLE, RS485_ENABLE_VALUE);
  uint8_t rSpeed  = mbWriteConfigRegister(REG_JOG_SPEED, motorSpeedRpm);
  uint8_t rAccel  = mbWriteConfigRegister(REG_JOG_ACCEL, JOG_ACCEL_VALUE);
  recordModbusResult(rAccel);

  Serial.print(F("Motor config: enable=0x"));
  Serial.print(rEnable, HEX);
  Serial.print(F(" jog_speed=0x"));
  Serial.print(rSpeed, HEX);
  Serial.print(F(" jog_accel=0x"));
  Serial.println(rAccel, HEX);
  if (rSpeed == MB_OK) jogSpeedWritten = motorSpeedRpm;
  if (rSpeed != MB_OK || rAccel != MB_OK) {
    Serial.println(F("WARN: JOG cfg write failed"));
  }
  return (rEnable == MB_OK);
}

// Latch the JOG direction. Called once per button press (/cw or /ccw).
void commandJog(MotorState dir) {
  if (!jogConfigured) {
    jogConfigured = setupMotorJog();    // one-time lazy link configuration
  } else if (jogSpeedWritten != motorSpeedRpm) {
    // Speed changed since the last Pr6.00 write: re-sync it BEFORE latching
    // the JOG (one retried config write; the motor is stopped here).
    uint8_t rs = mbWriteConfigRegister(REG_JOG_SPEED, motorSpeedRpm);
    recordModbusResult(rs);
    if (rs == MB_OK) {
      jogSpeedWritten = motorSpeedRpm;
    } else {
      Serial.println(F("WARN: jog speed write failed, old speed in use"));
    }
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
  uint8_t result = mbWriteSingleRegister(REG_TRIGGER, TRIG_ESTOP);
  recordModbusResult(result);
}

// HIGHEST PRIORITY task. Called first in loop() and between every 64-byte
// chunk while serving files, so the < 50 ms drive refresh is never starved.
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
// stopped AND no homing is running, so a blocking Modbus read can never
// delay a JOG command nor collide with the homing status polling.
void servicePositionPoll() {
  if (motorState != MOTOR_STOPPED) return;
  if (homingBusy()) return;              // motor is moving on its own
  if (moveState == M_MOVING) return;     // motor is moving on its own
  uint32_t now = millis();
  if (now - lastPositionPoll < POSITION_POLL_MS) return;
  lastPositionPoll = now;
  readFeedbackPosition();
}

// ================= TORQUE HOMING (STEP 1) =================
// Start the torque homing configured in MotionStudio: the drive runs CCW at
// 200 rpm to the mechanical hard stop and defines position 0 there.
void startHoming() {
  if (homingBusy()) return;              // already running

  // The frame is about to change: previous offset, travel measurement,
  // limits and any move result are void.
  homed = false;
  maxTravelCounts = 0;
  limitsActive = false;
  limit1Hit = false;
  limit2Hit = false;
  moveState = M_IDLE;
  moveError = 0;

  if (motorState != MOTOR_STOPPED) {
    stopMotor();                         // never start a homing over a JOG
  }
  if (!jogConfigured) {
    jogConfigured = setupMotorJog();     // RS485 enable required before triggering
  }

  uint8_t r1 = mbWriteConfigRegister(REG_HOMING_SPD_HIGH, HOMING_SPEED_HIGH_RPM);
  uint8_t r2 = mbWriteConfigRegister(REG_HOMING_SPD_LOW,  HOMING_SPEED_LOW_RPM);
  recordModbusResult(r2);
  Serial.print(F("Homing spd: h=0x"));
  Serial.print(r1, HEX);
  Serial.print(F(" l=0x"));
  Serial.println(r2, HEX);
  if (r1 != MB_OK || r2 != MB_OK) {
    homingState = H_ERROR;
    homingError = 1;
    Serial.println(F("Homing ERR 1 (speeds)"));
    return;
  }

  // PASS 1: direction = CW (far hard stop). Base 0x000C = torque homing.
  uint8_t rd = mbWriteConfigRegister(REG_HOMING_MODE,
                                     HOMING_MODE_TORQUE | HOMING_DIR_CW_BIT);
  recordModbusResult(rd);
  if (rd != MB_OK) {
    homingState = H_ERROR;
    homingError = 1;
    Serial.print(F("Homing ERR 1 (dir): 0x"));
    Serial.println(rd, HEX);
    return;
  }

  uint8_t rt = mbWriteConfigRegister(REG_TRIGGER, TRIG_HOMING);
  recordModbusResult(rt);
  if (rt != MB_OK) {
    homingState = H_ERROR;
    homingError = 2;
    Serial.print(F("Homing ERR 2: 0x"));
    Serial.println(rt, HEX);
    return;
  }

  homingState = H_PASS1;
  homingError = 0;
  homingStartMs = millis();
  lastHomingPoll = homingStartMs;
  Serial.println(F("Homing 1/2: CW to far stop"));
}

// Reads the feedback position with up to 3 attempts. Used at each pass end.
static bool readPositionRetry3() {
  for (uint8_t i = 0; i < 3; i++) {
    if (readFeedbackPosition()) return true;
    delay(50);
  }
  return false;
}

// Polls the drive motion status (0x1003) every 200 ms while a homing pass
// runs. The drive performs each pass on its own (no refresh needed, unlike
// the JOG): the firmware only supervises and reports.
// Pass completion: bit6 (homing OK) set AND bit2 (running) clear, after the
// 500 ms guard. SAFETY MODEL (on request): the Stop button is the ONLY
// abort; a failed status read is simply retried at the next poll.
void serviceHoming() {
  uint32_t now = millis();

  // Between the two passes: short settle, then launch PASS 2 (CCW).
  if (homingState == H_PASS2_START) {
    if (now - homingStartMs < HOMING_PASS_DWELL_MS) return;

    uint8_t rd = mbWriteConfigRegister(REG_HOMING_MODE, HOMING_MODE_TORQUE); // bit0 = 0 -> CCW
    recordModbusResult(rd);
    if (rd != MB_OK) {
      homingState = H_ERROR;
      homingError = 1;
      Serial.print(F("Homing ERR 1 (p2 dir): 0x"));
      Serial.println(rd, HEX);
      return;
    }
    uint8_t rt = mbWriteConfigRegister(REG_TRIGGER, TRIG_HOMING);
    recordModbusResult(rt);
    if (rt != MB_OK) {
      homingState = H_ERROR;
      homingError = 2;
      Serial.print(F("Homing ERR 2 (p2): 0x"));
      Serial.println(rt, HEX);
      return;
    }
    homingState = H_PASS2;
    homingStartMs = now;
    lastHomingPoll = now;
    Serial.println(F("Homing 2/2: CCW across axis"));
    return;
  }

  if (homingState != H_PASS1 && homingState != H_PASS2) return;

  if (now - lastHomingPoll < HOMING_POLL_MS) return;
  lastHomingPoll = now;

  uint16_t st;
  uint8_t rc = mbReadRetry(REG_MOTION_STATUS, 1, &st);
  recordModbusResult(rc);
  if (rc != MB_OK) return;   // keep polling; only the Stop button aborts a homing

  if (st & MOTION_BIT_FAULT) {
    homingState = H_ERROR;
    homingError = 4;
    Serial.println(F("Homing ERR 4: fault"));
    return;
  }

  bool passDone = (st & MOTION_BIT_HOMING_OK) &&
                  !(st & MOTION_BIT_RUNNING) &&
                  (now - homingStartMs > HOMING_OK_GUARD_MS);
  if (!passDone) return;

  if (homingState == H_PASS1) {
    // CW (far) hard stop reached: capture the pass-1 reference position.
    if (!readPositionRetry3()) {
      homingState = H_ERROR;
      homingError = 6;
      Serial.println(F("Homing ERR 6 (p1)"));
      return;
    }
    pass1OffsetRaw = feedbackPositionPulses;
    Serial.print(F("Pass 1 done. Raw ref: "));
    Serial.println(pass1OffsetRaw);
    homingState = H_PASS2_START;
    homingStartMs = now;             // dwell reference
    return;
  }

  // PASS 2 done: CCW (near) hard stop reached.
  // 1) BEFORE setting the final zero: measure the travel vs the pass-1 ref.
  if (!readPositionRetry3()) {
    homingState = H_ERROR;
    homingError = 6;
    Serial.println(F("Homing ERR 6 (p2)"));
    return;
  }
  maxTravelCounts = labs(feedbackPositionPulses - pass1OffsetRaw);
  // counts * 500 / 65536 in pure 32-bit (counts is positive here)
  int32_t mmCenti = (int32_t)(((maxTravelCounts >> 16) * 500L)
                  + ((((uint32_t)maxTravelCounts & 0xFFFFUL) * 500UL) >> 16));
  Serial.print(F("Max travel: "));
  Serial.print(maxTravelCounts);
  Serial.print(F(" cnt = "));
  Serial.print(mmCenti / 100);
  Serial.print(F("."));
  if ((mmCenti % 100) < 10) Serial.print(F("0"));
  Serial.print(mmCenti % 100);
  Serial.println(F(" mm"));

  // 2) Final zero at the CCW stop (documented zero-set, drive-frame side).
  uint8_t rz = mbWriteConfigRegister(REG_TRIGGER, TRIG_SET_ZERO);
  recordModbusResult(rz);
  if (rz != MB_OK) {
    homingState = H_ERROR;
    homingError = 3;
    Serial.print(F("Homing ERR 3: 0x"));
    Serial.println(rz, HEX);
    return;
  }

  // 3) Capture the HOME OFFSET: this raw position is the homed-frame zero.
  homeOffsetPulses = feedbackPositionPulses;
  homed = true;
  homingState = H_DONE;
  Serial.print(F("Homing DONE. Raw offset: "));
  Serial.println(homeOffsetPulses);
  lastPositionPoll = millis();

  // 4) Soft limits: Limit 1 = +10 mm, Limit 2 = max travel - 10 mm.
  int32_t travelCmd = countsToCmdPulses(maxTravelCounts);
  if (travelCmd > 2L * LIMIT_MARGIN_CMD) {
    limit1Cmd = LIMIT_MARGIN_CMD;
    limit2Cmd = travelCmd - LIMIT_MARGIN_CMD;
    limitsActive = true;
    Serial.print(F("Limits(cmd): L1="));
    Serial.print(limit1Cmd);
    Serial.print(F(" L2="));
    Serial.println(limit2Cmd);
  } else {
    limitsActive = false;
    Serial.println(F("WARN: travel<=20mm, no limits"));
  }
}

const char *moveStateText() {
  switch (moveState) {
    case M_MOVING: return "moving";
    case M_DONE:   return "done";
    case M_ERROR:  return "error";
    default:       return "idle";
  }
}

// ================= GO TO POSITION (PR0 RELATIVE MOVE) =================
// Target is an ABSOLUTE position in the HOMED frame (pulses). Because the
// drive's raw counter cannot be trusted as a frame, the move is executed as
// a RELATIVE PR0 move of (target - current_homed) pulses: frame-independent.
// guardLimit = 1 for the guarded CCW (target = Limit 1) or 2 for the guarded
// CW (target = Limit 2), at the user-set speed: reaching the target raises the
// matching limitX_hit flag; releasing the button (/stop) is a normal end.
// guardLimit = 0 for a normal /goto move.
void startMove(int32_t targetHomedPulses, uint16_t speedRpm, uint8_t guardLimit) {
  if (moveState == M_MOVING || homingBusy()) return;
  moveGuardLimit = guardLimit;
  if (!jogConfigured) {
    jogConfigured = setupMotorJog();     // RS485 enable required for PR triggers
  }

  // Fresh raw position (motor is stopped here, read is allowed).
  if (!readFeedbackPosition()) {
    moveState = M_ERROR;
    moveError = 4;
    Serial.println(F("Move ERR 4: pos read"));
    return;
  }
  // UNIT FIX: the feedback position is in 65536 counts/rev, /goto targets
  // are in 10000 command pulses/rev. Convert counts -> cmd pulses (64-bit
  // exact) BEFORE computing the relative delta.
  int32_t currentHomedCounts = feedbackPositionPulses - homeOffsetPulses;
  int32_t currentCmdPulses = countsToCmdPulses(currentHomedCounts);
  int32_t delta = targetHomedPulses - currentCmdPulses;
  moveTargetHomed = targetHomedPulses;

  Serial.print(guardLimit == 1 ? F("G-CCW: tgt=")
             : guardLimit == 2 ? F("G-CW: tgt=")
                               : F("Goto: tgt="));
  Serial.print(targetHomedPulses);
  Serial.print(F(" cur="));
  Serial.print(currentCmdPulses);
  Serial.print(F(" delta="));
  Serial.println(delta);

  if (delta == 0) {
    moveState = M_DONE;
    moveError = 0;
    if (guardLimit == 1) {
      limit1Hit = true;                // already exactly at Limit 1
      Serial.println(F("G-CCW: at L1"));
    } else if (guardLimit == 2) {
      limit2Hit = true;                // already exactly at Limit 2
      Serial.println(F("G-CW: at L2"));
    }
    return;
  }

  uint32_t d = (uint32_t)delta;          // two's complement for negative deltas
  uint8_t r1 = mbWriteConfigRegister(REG_PR0_MODE,  PR0_MODE_REL_POS);
  uint8_t r2 = mbWriteConfigRegister(REG_PR0_POS_H, (uint16_t)(d >> 16));
  uint8_t r3 = mbWriteConfigRegister(REG_PR0_POS_L, (uint16_t)(d & 0xFFFF));
  uint8_t r4 = mbWriteConfigRegister(REG_PR0_SPEED, speedRpm);
  uint8_t r5 = mbWriteConfigRegister(REG_PR0_ACC,   MOVE_ACCDEC);
  uint8_t r6 = mbWriteConfigRegister(REG_PR0_DEC,   MOVE_ACCDEC);
  uint8_t rt = (r1 | r2 | r3 | r4 | r5 | r6) == MB_OK
                 ? mbWriteConfigRegister(REG_TRIGGER, TRIG_PATH0)
                 : MB_ERR_TIMEOUT;
  recordModbusResult(rt);
  if (r1 != MB_OK || r2 != MB_OK || r3 != MB_OK ||
      r4 != MB_OK || r5 != MB_OK || r6 != MB_OK || rt != MB_OK) {
    moveState = M_ERROR;
    moveError = 1;
    Serial.print(F("Move ERR 1: cfg=0x"));
    Serial.print((uint8_t)(r1 | r2 | r3 | r4 | r5 | r6), HEX);
    Serial.print(F(" trig=0x"));
    Serial.print(rt, HEX);
    Serial.println(F(")"));
    return;
  }

  moveState = M_MOVING;
  moveError = 0;
  moveStartMs = millis();
  lastMovePoll = moveStartMs;
  Serial.println(guardLimit == 1 ? F("G-CCW -> L1 started")
               : guardLimit == 2 ? F("G-CW -> L2 started")
                                 : F("PR0 move started"));
}

// Polls the drive while a /goto move runs. ONE read per cycle: motion status
// 0x1003 - bit0 (fault) -> error; bit2 ("running") cleared after the 500 ms
// start guard -> move DONE. (The 0x6002 read-back never returned 0x0000 on
// this drive after path completion, so it is not used anymore.)
// SAFETY MODEL: the Stop button is the ONLY abort, like the homing.
void serviceMove() {
  if (moveState != M_MOVING) return;
  uint32_t now = millis();
  if (now - lastMovePoll < MOVE_POLL_MS) return;
  lastMovePoll = now;

  uint16_t st;
  uint8_t rc = mbReadRetry(REG_MOTION_STATUS, 1, &st);
  recordModbusResult(rc);
  if (rc != MB_OK) return;   // keep polling; only the Stop button aborts

  if (st & MOTION_BIT_FAULT) {
    moveState = M_ERROR;
    moveError = 2;
    Serial.println(F("Move ERR 2: fault"));
    return;
  }

  if (!(st & MOTION_BIT_RUNNING) && (now - moveStartMs > MOVE_START_GUARD_MS)) {
    moveState = M_DONE;
    if (moveGuardLimit == 1) {
      limit1Hit = true;                  // reached Limit 1 with the button held
      Serial.println(F("G-CCW DONE: at L1"));
    } else if (moveGuardLimit == 2) {
      limit2Hit = true;                  // reached Limit 2 with the button held
      Serial.println(F("G-CW DONE: at L2"));
    } else {
      Serial.print(F("Move DONE (st=0x"));
      Serial.print(st, HEX);
      Serial.println(F(")"));
    }
    readFeedbackPosition();              // motor stopped: refresh the display
    lastPositionPoll = millis();
  }
}

void sendMotorOkResponse(EthernetClient client) {
  httpHeader(client, 200, CT_JSON);
  client.print(F("{\"motor_state\":\""));
  client.print(motorStateText());
  client.print(F("\"}"));
  client.println();
}

void sendHomingStatusResponse(EthernetClient client) {
  httpHeader(client, 200, CT_JSON);
  client.print(F("{\"homing_state\":\""));
  client.print(homingStateText());
  client.print(F("\",\"homing_error\":"));
  client.print(homingError);
  client.println(F("}"));
}

void sendMoveStatusResponse(EthernetClient client) {
  httpHeader(client, 200, CT_JSON);
  client.print(F("{\"move_state\":\""));
  client.print(moveStateText());
  client.print(F("\",\"move_error\":"));
  client.print(moveError);
  client.println(F("}"));
}

// Generic 409 used for every "request rejected" case (homing in progress,
// move in progress, jog active, not homed): {"error":"<reason>"}.
void sendBusyResponse(EthernetClient client, const __FlashStringHelper *reason) {
  httpHeader(client, 409, CT_JSON);
  client.print(F("{\"error\":\""));
  client.print(reason);
  client.println(F("\"}"));
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
    if (homingBusy()) {
      sendBusyResponse(client, F("homing_in_progress"));
    } else if (moveState == M_MOVING) {
      sendBusyResponse(client, F("move_in_progress"));
    } else if (homed && limitsActive) {
      // GUARDED CW: bounded PR0 move whose target IS Limit 2 - it can never
      // go above it, and if the motor is already at/beyond Limit 2 it moves
      // BACK DOWN to it, even with the button still pressed.
      if (motorState != MOTOR_STOPPED) stopMotor();
      limit1Hit = false;
      limit2Hit = false;
      startMove(limit2Cmd, motorSpeedRpm, 2);
      sendMoveStatusResponse(client);     // page sees move_state + move_guard
    } else {
      limit1Hit = false;                  // new motion command acknowledges the flags
      limit2Hit = false;
      commandJog(MOTOR_CW);               // raw JOG (not homed / no limits)
      sendMotorOkResponse(client);
    }
  } else if (strstr(request, "GET /ccw ")) {
    if (homingBusy()) {
      sendBusyResponse(client, F("homing_in_progress"));
    } else if (moveState == M_MOVING) {
      sendBusyResponse(client, F("move_in_progress"));
    } else if (homed && limitsActive) {
      // GUARDED CCW: bounded PR0 move whose target IS Limit 1 - it can never
      // go below it, and if the motor is already at/below Limit 1 it moves
      // BACK UP to it, even with the button still pressed.
      if (motorState != MOTOR_STOPPED) stopMotor();
      limit1Hit = false;
      limit2Hit = false;
      startMove(limit1Cmd, motorSpeedRpm, 1);
      sendMoveStatusResponse(client);     // page sees move_state + move_guard
    } else {
      limit1Hit = false;
      limit2Hit = false;
      commandJog(MOTOR_CCW);              // raw JOG (not homed / no limits)
      sendMotorOkResponse(client);
    }
  } else if (strstr(request, "GET /stop ")) {
    bool wasHoming = homingBusy();
    bool wasMoving = (moveState == M_MOVING);
    stopMotor();               // E-stop: also aborts a running homing or PR0 move
    if (wasHoming) {
      homingState = H_ERROR;
      homingError = 5;
      Serial.println(F("Homing aborted (Stop)"));
    }
    if (wasMoving) {
      if (moveGuardLimit != 0) {
        moveState = M_IDLE;              // jog button released: normal end, no error
        Serial.println(F("Guard released"));
      } else {
        moveState = M_ERROR;
        moveError = 3;
        Serial.println(F("Move aborted (Stop)"));
      }
    }
    if (!wasHoming && !wasMoving) {
      Serial.println(F("Motor stopped"));
    }
    readFeedbackPosition();    // motor is stopped: immediate position read is allowed
    sendMotorOkResponse(client);
  } else if (strstr(request, "GET /home ")) {
    if (moveState == M_MOVING) {
      sendBusyResponse(client, F("move_in_progress"));
    } else {
      startHoming();           // no-op if already homing
      sendHomingStatusResponse(client);
    }
  } else if (strstr(request, "GET /goto?p=")) {
    if (homingBusy()) {
      sendBusyResponse(client, F("homing_in_progress"));
    } else if (moveState == M_MOVING) {
      sendBusyResponse(client, F("move_in_progress"));
    } else if (motorState != MOTOR_STOPPED) {
      sendBusyResponse(client, F("jog_active"));
    } else if (!homed) {
      sendBusyResponse(client, F("not_homed"));
    } else {
      long target = atol(strstr(request, "?p=") + 3);   // homed frame, cmd pulses
      if (limitsActive &&
          ((int32_t)target < limit1Cmd || (int32_t)target > limit2Cmd)) {
        sendBusyResponse(client, F("out_of_limits"));
      } else {
        limit1Hit = false;
        limit2Hit = false;
        startMove((int32_t)target, motorSpeedRpm, 0);
        sendMoveStatusResponse(client);
      }
    }
  } else if (strstr(request, "GET /speed?v=")) {
    long v = atol(strstr(request, "?v=") + 3);
    if (homingBusy()) {
      sendBusyResponse(client, F("homing_in_progress"));
    } else if (moveState == M_MOVING) {
      sendBusyResponse(client, F("move_in_progress"));
    } else if (motorState != MOTOR_STOPPED) {
      sendBusyResponse(client, F("jog_active"));
    } else if (v < SPEED_MIN_RPM || v > SPEED_MAX_RPM) {
      sendBusyResponse(client, F("speed_out_of_range"));
    } else {
      motorSpeedRpm = (uint16_t)v;     // applied at the next JOG / move
      Serial.print(F("Speed set: "));
      Serial.println(motorSpeedRpm);
      httpHeader(client, 200, CT_JSON);
      client.print(F("{\"speed_rpm\":"));
      client.print(motorSpeedRpm);
      client.println(F("}"));
    }
  } else if (strstr(request, "GET /hwstatus ")) {
    sendHWStatusResponse(client);
  } else if (strstr(request, "GET / ")) {
    sendHTMLPage(client);
  } else {
    httpHeader(client, 404, CT_TEXT);
  }
  client.stop();
}

void sendStatusResponse(EthernetClient client) {
  // No Modbus access here: position is cached, refreshed by the 1 Hz
  // background poll (motor stopped, no homing) and on /stop.
  httpHeader(client, 200, CT_JSON);
  client.print(F("{\"uptime\":"));
  client.print(millis());
  client.print(F(",\"feedback_position\":"));
  client.print(feedbackPositionPulses - homeOffsetPulses);  // HOMED frame
  client.print(F(",\"homed\":"));
  client.print(homed ? F("true") : F("false"));
  client.print(F(",\"feedback_position_valid\":"));
  client.print(feedbackPositionValid ? F("true") : F("false"));
  client.print(F(",\"last_modbus_result\":"));
  client.print(lastModbusResult);
  client.print(F(",\"last_modbus_ms\":"));
  client.print(lastModbusMs);
  client.print(F(",\"modbus_timeouts\":"));
  client.print(mbTimeouts);
  client.print(F(",\"speed_rpm\":"));
  client.print(motorSpeedRpm);
  client.print(F(",\"homing_state\":\""));
  client.print(homingStateText());
  client.print(F("\",\"homing_error\":"));
  client.print(homingError);
  client.print(F(",\"max_travel_counts\":"));
  client.print(maxTravelCounts);
  client.print(F(",\"limits_active\":"));
  client.print(limitsActive ? F("true") : F("false"));
  client.print(F(",\"limit1_cmd\":"));
  client.print(limit1Cmd);
  client.print(F(",\"limit2_cmd\":"));
  client.print(limit2Cmd);
  client.print(F(",\"limit1_hit\":"));
  client.print(limit1Hit ? F("true") : F("false"));
  client.print(F(",\"limit2_hit\":"));
  client.print(limit2Hit ? F("true") : F("false"));
  client.print(F(",\"move_guard\":"));
  client.print(moveGuardLimit != 0 ? F("true") : F("false"));
  client.print(F(",\"guard_limit\":"));
  client.print(moveGuardLimit);
  client.print(F(",\"move_state\":\""));
  client.print(moveStateText());
  client.print(F("\",\"move_error\":"));
  client.print(moveError);
  client.print(F(",\"motor_state\":\""));
  client.print(motorStateText());
  client.print(F("\""));
  client.println(F("}"));
}

void sendHWStatusResponse(EthernetClient client) {
  if (!sdReady) {
    httpHeader(client, 503, CT_TEXT);
    client.println(F("SD not available"));
    return;
  }

  // Open the file BEFORE sending the headers: allows returning a proper 404.
  File txt = SD.open("test.txt", FILE_READ);
  if (!txt) {
    httpHeader(client, 404, CT_TEXT);
    client.println(F("test.txt not found"));
    return;
  }

  httpHeader(client, 200, CT_TEXT);

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
    httpHeader(client, 503, CT_HTML);
    client.println(F("<html><body>SD not available</body></html>"));
    return;
  }

  File html = SD.open("index.htm", FILE_READ);
  if (!html) {
    httpHeader(client, 404, CT_HTML);
    client.println(F("<html><body>index.htm not found on SD card</body></html>"));
    return;
  }

  httpHeader(client, 200, CT_HTML);

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

  Serial.println(F("Boot"));
  delay(500);                      // let hardware power up before the checks

  EthernetStatus ethStatus = checkEthernetStatus();
  server.begin();                  // web server only after Ethernet init
  checkSDCardStatus();             // single FAT mount, sets sdReady
  printHardwareReport(Serial, ethStatus);
  createtxt(ethStatus);

  // Motor link configuration (RS485 enable + JOG speed/accel) is done ONCE,
  // lazily, on the first JOG or homing request - V1.0.3 behavior.

  if (readFeedbackPosition()) {
    Serial.print(F("Init pos: "));
    Serial.println(feedbackPositionPulses);
  }

  Serial.println(F("Ready."));
}

// ================= MAIN LOOP =================
// Strict priority order:
//   1) Modbus JOG command to the motor (refresh)  - every pass
//   2) HTTP (one client per pass; file serving interleaves 1) per chunk)
//   3) Homing / PR0 move supervision - polls every 200 ms while active
//   4) Feedback position poll - only while stopped and idle, 1 Hz
void loop() {
  serviceMotorJog();

  EthernetClient client = server.available();
  if (client) {
    handleClient(client);
    return;
  }

  serviceHoming();
  serviceMove();
  servicePositionPoll();
}
