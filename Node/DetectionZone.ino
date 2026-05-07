#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <LittleFS.h>

// ================= Pins =================
#define TRIG1_PIN  25
#define ECHO1_PIN  39
#define TRIG2_PIN  16
#define ECHO2_PIN  36
#define SOIL_PIN   26
#define SOIL_DRY   1800   // ADC reading at 0% moisture  <-- CALIBRATE for your sensor
#define SOIL_WET   800    // ADC reading at 100% moisture <-- CALIBRATE for your sensor

// ================= LoRa RA-08H (UART2) =================
#define LORA_RX_PIN 13
#define LORA_TX_PIN 14
HardwareSerial LoRaSerial(2);
bool loraReady = false;

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================= BME280 =================
Adafruit_BME280 bme;

// ================= EU868 Duty Cycle Config =================
#define SAMPLE_INTERVAL_MS   10000UL
#define SEND_INTERVAL_MS     60000UL
#define SAMPLE_BUFFER_SIZE   6

// ================= [DB] Storage Config =================
#define LOG_FILE        "/log.csv"
#define LOG_OLD_FILE    "/log.old.csv"
#define QUEUE_FILE      "/queue.csv"
#define QUEUE_TMP_FILE  "/queue_tmp.csv"
#define MAX_LOG_SIZE    1761280UL
#define MAX_QUEUE_ROWS  12150
int queueCount = 0;

// ================= [TIME] Software Clock (millis-based) =================
int   clk_year    = 2026, clk_month  = 4,  clk_day     = 19;
int   clk_hours   = 8,    clk_minutes = 0,  clk_seconds = 0;
unsigned long clk_prevMillis = 0;

int getDaysInMonth(int m, int y) {
  if (m == 2) return (((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0)) ? 29 : 28;
  if (m == 4 || m == 6 || m == 9 || m == 11) return 30;
  return 31;
}

void tickClock() {
  unsigned long now = millis();
  if (now - clk_prevMillis < 1000UL) return;
  clk_prevMillis = now;

  clk_seconds++;
  if (clk_seconds >= 60) { clk_seconds = 0; clk_minutes++; }
  if (clk_minutes >= 60) { clk_minutes = 0; clk_hours++;   }
  if (clk_hours   >= 24) { clk_hours   = 0; clk_day++;     }
  if (clk_day > getDaysInMonth(clk_month, clk_year)) { clk_day = 1; clk_month++; }
  if (clk_month   > 12) { clk_month = 1; clk_year++; }
}

String getTimestamp() {
  char buf[22];
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
    clk_year, clk_month, clk_day, clk_hours, clk_minutes, clk_seconds);
  return String(buf);
}

uint32_t getEpochSeconds() {
  uint32_t totalDays = 0;
  int baseYear = 2026;
  for (int y = baseYear; y < clk_year; y++)
    totalDays += (((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0)) ? 366 : 365;
  for (int m = 1; m < clk_month; m++)
    totalDays += getDaysInMonth(m, clk_year);
  totalDays += (clk_day - 1);
  return totalDays * 86400UL + clk_hours * 3600UL + clk_minutes * 60UL + clk_seconds;
}

// ================= Sample Buffer =================
struct SampleBuffer {
  float temp[SAMPLE_BUFFER_SIZE];
  float hum[SAMPLE_BUFFER_SIZE];
  float pres[SAMPLE_BUFFER_SIZE];
  float dist1[SAMPLE_BUFFER_SIZE];
  float dist2[SAMPLE_BUFFER_SIZE];
  float dist[SAMPLE_BUFFER_SIZE];
  float moist[SAMPLE_BUFFER_SIZE];
  int   count = 0;
  int   head  = 0;

  void push(float t, float h, float p, float d1, float d2, float d, float m) {
    temp[head]  = t;  hum[head]   = h;
    pres[head]  = p;  dist1[head] = d1;
    dist2[head] = d2; dist[head]  = d;
    moist[head] = m;
    head = (head + 1) % SAMPLE_BUFFER_SIZE;
    if (count < SAMPLE_BUFFER_SIZE) count++;
  }

  float avg(float* buf) {
    float sum = 0; int n = 0;
    for (int i = 0; i < count; i++) {
      if (buf[i] >= 0) { sum += buf[i]; n++; }
    }
    return n > 0 ? sum / n : 0;
  }
} samples;

// ================= Per-read smoothing =================
const int SMOOTH_SIZE = 5;
float dist1Buf[SMOOTH_SIZE] = {0};
float dist2Buf[SMOOTH_SIZE] = {0};
int   smoothIdx = 0;
bool  displayPage = false;

// ================= Median of 3 =================
float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ================= Ultrasonic =================
float readUltrasonic(int trigPin, int echoPin) {
  float readings[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseInLong(echoPin, HIGH, 38000);
    if (duration > 0) {
      float distance = (duration * 0.0343 / 2.0) / 100.0;
      readings[i] = (distance < 0.20 || distance > 6.0) ? -1 : distance;
    } else {
      readings[i] = -1;
    }
    delay(60);
  }
  if (readings[0] < 0 && readings[1] < 0 && readings[2] < 0) return -1;
  float a = readings[0] < 0 ? readings[1] : readings[0];
  float b = readings[1] < 0 ? readings[2] : readings[1];
  float c = readings[2] < 0 ? readings[0] : readings[2];
  return median3(a, b, c);
}

// ================= Combine Ultrasonics =================
float combineDistances(float d1, float d2) {
  if (d1 <= 0 && d2 <= 0) return -1;
  if (d1 <= 0) return d2;
  if (d2 <= 0) return d1;
  float avg = (d1 + d2) / 2.0;
  float diff1 = abs(d1 - avg), diff2 = abs(d2 - avg);
  const float DIST_EPS = 0.001;
  if (diff1 > diff2 + DIST_EPS) return d1;
  if (diff2 > diff1 + DIST_EPS) return d2;
  return avg;
}

// ================= Moving Average =================
float movingAverage(float* buf) {
  float sum = 0; int cnt = 0;
  for (int i = 0; i < SMOOTH_SIZE; i++) {
    if (buf[i] >= 0) { sum += buf[i]; cnt++; }
  }
  return cnt > 0 ? sum / cnt : -1;
}

// ================= LoRa Helpers =================

// FIX: Increased flush window to 3500ms to fully cover RA-08H RX1 (1s) + RX2 (2s) windows.
// The original 800ms was too short and left +RECV/+CDRX data in the UART buffer,
// which contaminated subsequent AT command responses and caused malformed/long
// packets to appear on ChirpStack.
void loraFlush() {
  unsigned long start = millis();
  while (millis() - start < 3500) {
    while (LoRaSerial.available()) LoRaSerial.read();
    delay(10);
  }
}

// FIX: After AT+DTRX, the RA-08H sends OK+SEND(T) immediately, then a delayed
// +RECV or +CDRX response during the RX window (up to ~3s later).
// The old code broke out on the first "OK" match and left that trailing data
// in the UART buffer, which prepended garbage to the next AT response.
// Now we do a post-DTRX drain pass to consume the full RX window.
String loraAT(const String& cmd, unsigned long timeout = 3000) {
  // Flush any stale data before sending
  while (LoRaSerial.available()) LoRaSerial.read();

  LoRaSerial.println(cmd);
  String response = "";
  unsigned long start = millis();

  // Wait for primary OK/ERROR response
  while (millis() - start < timeout) {
    while (LoRaSerial.available()) response += (char)LoRaSerial.read();
    if (response.indexOf("OK") >= 0 || response.indexOf("ERROR") >= 0) break;
  }

  // FIX: For DTRX commands, drain the RX1/RX2 window data that arrives after
  // the initial OK+SEND(T). Without this, delayed +RECV/+CDRX bytes sit in the
  // UART buffer and corrupt the next AT command's response string, which is what
  // was causing unexpectedly long / malformed packets seen on ChirpStack.
  if (cmd.indexOf("DTRX") >= 0) {
    Serial.println("[LoRa] Draining RX window (up to 3500ms)...");
    unsigned long drainStart = millis();
    while (millis() - drainStart < 3500) {
      while (LoRaSerial.available()) {
        response += (char)LoRaSerial.read();
      }
      delay(10);
    }
  }

  response.trim();
  Serial.println("[LoRa] << " + cmd);
  Serial.println("[LoRa] >> " + response);
  return response;
}

// ================= LoRa ABP Init =================
void loraInit() {
  Serial.println("[LoRa] Configuring RA-08H for ABP (EU868)...");

  loraAT("AT");
  loraFlush();

  loraAT("AT+CJOINMODE=1");
  loraAT("AT+CCLASS=0");
  loraAT("AT+CFREQBANDMASK=0001");
  loraAT("AT+CULDLMODE=2");
  loraAT("AT+CADR=0");
  loraAT("AT+CDATARATE=5");
  loraAT("AT+CAPPPORT=10");
  loraAT("AT+CCONFIRM=0");
  loraAT("AT+CTXP=0");

  loraAT("AT+CDEVADDR=0037ad51");
  loraAT("AT+CNWKSKEY=1a67c22cd980b14ab067a8cd8a335f58");
  loraAT("AT+CAPPSKEY=6b4a830871a7ce59fa05a6cb011dbd1b");

  loraAT("AT+CSAVE");

  loraReady = true;
  Serial.println("[LoRa] ABP ready. Sending every 60s (EU868 compliant).");
  loraFlush();
}

// ================= LoRa Send =================
// Payload layout (18 bytes = 36 hex chars):
//   [0-3]   epoch   s     uint32
//   [4-5]   temp    x10   uint16
//   [6-7]   hum     x10   uint16
//   [8-9]   pres    x1    uint16
//   [10-11] dist1   x100  uint16
//   [12-13] dist2   x100  uint16
//   [14-15] dist    x100  uint16
//   [16-17] moist   x10   uint16
void loraSend(float temp, float hum, float pres,
              float dist1, float dist2, float dist, float moist) {
  if (!loraReady) { Serial.println("[LoRa] Not ready."); return; }

  if (isnan(temp))               temp  = 0;
  if (isnan(hum))                hum   = 0;
  if (isnan(pres))               pres  = 0;
  if (isnan(dist1) || dist1 < 0) dist1 = 0;
  if (isnan(dist2) || dist2 < 0) dist2 = 0;
  if (isnan(dist)  || dist  < 0) dist  = 0;
  if (isnan(moist) || moist < 0) moist = 0;

  uint32_t epoch = getEpochSeconds();
  // 36 hex chars + null = 37 bytes
  char payload[37];
  snprintf(payload, sizeof(payload), "%08lX%04X%04X%04X%04X%04X%04X%04X",
    (unsigned long)epoch,
    (uint16_t)(temp  * 10),
    (uint16_t)(hum   * 10),
    (uint16_t)(pres),
    (uint16_t)(dist1 * 100),
    (uint16_t)(dist2 * 100),
    (uint16_t)(dist  * 100),
    (uint16_t)(moist * 10));

  // 18 bytes: uint32 epoch + 7x uint16
  String cmd = "AT+DTRX=0,1,18,";
  cmd += String(payload);

  // loraAT will now automatically drain the RX window after DTRX
  String response = loraAT(cmd, 8000);

  if (response.indexOf("OK+SENT") >= 0 || response.indexOf("OK+SEND") >= 0) {
    Serial.println("[LoRa] OK Uplink sent (avg of " + String(samples.count) + " samples): " + String(payload));
  } else if (response.indexOf("ERR+SEND:00") >= 0) {
    Serial.println("[LoRa] ERR: Not joined - reapplying ABP...");
    loraAT("AT+CJOINMODE=1");
    loraAT("AT+CDEVADDR=0037ad51");
    loraAT("AT+CNWKSKEY=1a67c22cd980b14ab067a8cd8a335f58");
    loraAT("AT+CAPPSKEY=6b4a830871a7ce59fa05a6cb011dbd1b");
    loraAT("AT+CSAVE");
    loraFlush();
  } else {
    Serial.println("[LoRa] Response: " + response);
  }
}

// ================= [DB] CSV Helpers =================
// Column order: timestamp, moist_pct, temp_c, hum_pct, pres_hpa, dist1_m, dist2_m, dist_m
String buildCsvRow(float moist, float temp, float hum,
                   float pres, float d1, float d2, float dist) {
  char buf[200];
  snprintf(buf, sizeof(buf),
    "%s,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    getTimestamp().c_str(),
    moist, temp, hum, pres, d1, d2, dist);
  return String(buf);
}

void appendLog(const String& row) {
  File f = LittleFS.open(LOG_FILE, "a");
  if (!f) { Serial.println("[FS] Cannot open log.csv"); return; }
  f.seek(0, SeekEnd);
  size_t sz = f.position();
  f.close();

  if (sz >= MAX_LOG_SIZE) {
    LittleFS.remove(LOG_OLD_FILE);
    LittleFS.rename(LOG_FILE, LOG_OLD_FILE);
    Serial.println("[FS] log.csv rotated -> log.old.csv");
    File nf = LittleFS.open(LOG_FILE, "w");
    if (nf) {
      nf.println("timestamp,moist_pct,temp_c,hum_pct,pres_hpa,dist1_m,dist2_m,dist_m");
      nf.close();
    }
  }

  File af = LittleFS.open(LOG_FILE, "a");
  if (af) { af.print(row); af.close(); }
}

// ================= [DB] Queue Helpers =================
void appendQueue(const String& row) {
  if (queueCount >= MAX_QUEUE_ROWS) {
    File src = LittleFS.open(QUEUE_FILE, "r");
    if (!src) { Serial.println("[FS] appendQueue: cannot open queue"); return; }
    src.readStringUntil('\n');
    File tmp = LittleFS.open(QUEUE_TMP_FILE, "w");
    if (!tmp) { src.close(); Serial.println("[FS] appendQueue: cannot open tmp"); return; }
    uint8_t buf[128];
    while (src.available()) {
      int n = src.read(buf, sizeof(buf));
      if (n > 0) tmp.write(buf, n);
    }
    src.close(); tmp.close();
    LittleFS.remove(QUEUE_FILE);
    LittleFS.rename(QUEUE_TMP_FILE, QUEUE_FILE);
    queueCount--;
    Serial.println("[FS] Queue full - dropped oldest row");
  }
  File qf = LittleFS.open(QUEUE_FILE, "a");
  if (qf) { qf.print(row); qf.close(); queueCount++; }
  Serial.printf("[FS] Queued (total: %d / %d rows)\n", queueCount, MAX_QUEUE_ROWS);
}

String peekQueueRow() {
  if (queueCount == 0) return "";
  File qf = LittleFS.open(QUEUE_FILE, "r");
  if (!qf) return "";
  String line = qf.readStringUntil('\n');
  qf.close();
  line.trim();
  if (line.startsWith("timestamp")) {
    Serial.println("[FS] peekQueue: skipped accidental header line");
    return "";
  }
  return line;
}

void popQueueRow() {
  File src = LittleFS.open(QUEUE_FILE, "r");
  if (!src) return;
  src.readStringUntil('\n');
  File tmp = LittleFS.open(QUEUE_TMP_FILE, "w");
  if (!tmp) { src.close(); Serial.println("[FS] popQueue: cannot open tmp"); return; }
  uint8_t buf[128];
  while (src.available()) {
    int n = src.read(buf, sizeof(buf));
    if (n > 0) tmp.write(buf, n);
  }
  src.close(); tmp.close();
  LittleFS.remove(QUEUE_FILE);
  LittleFS.rename(QUEUE_TMP_FILE, QUEUE_FILE);
  if (queueCount > 0) queueCount--;
  Serial.printf("[FS] Replayed & popped queue (remaining: %d)\n", queueCount);
}

int countQueueRows() {
  File qf = LittleFS.open(QUEUE_FILE, "r");
  if (!qf) return 0;
  int n = 0;
  while (qf.available()) { qf.readStringUntil('\n'); n++; }
  qf.close();
  return n;
}

// ================= [DB] Serial Monitor Commands =================
void handleSerialCommand() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "l") {
    Serial.println("\n--- log.csv ---");
    File f = LittleFS.open(LOG_FILE, "r");
    if (!f) { Serial.println("File not found"); }
    else { while (f.available()) Serial.write(f.read()); f.close(); }
    Serial.println("--- end ---\n");

  } else if (input == "q") {
    Serial.println("\n--- queue.csv ---");
    File f = LittleFS.open(QUEUE_FILE, "r");
    if (!f) { Serial.println("File not found or empty"); }
    else { while (f.available()) Serial.write(f.read()); f.close(); }
    Serial.printf("--- end (queueCount=%d) ---\n\n", queueCount);

  } else if (input == "x") {
    LittleFS.remove(LOG_FILE);
    LittleFS.remove(QUEUE_FILE);
    queueCount = 0;
    File f = LittleFS.open(LOG_FILE, "w");
    if (f) {
      f.println("timestamp,moist_pct,temp_c,hum_pct,pres_hpa,dist1_m,dist2_m,dist_m");
      f.close();
    }
    Serial.println("[FS] Files wiped. log.csv recreated with header.");

  } else if (input == "s") {
    Serial.println("\n--- storage ---");
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    Serial.printf("Used:        %u KB / %u KB (%.1f%%)\n",
      used/1024, total/1024, (used*100.0f)/total);
    if (LittleFS.exists(LOG_FILE)) {
      File f = LittleFS.open(LOG_FILE, "r");
      if (f) { Serial.printf("log.csv:     %u bytes\n", f.size()); f.close(); }
    }
    if (LittleFS.exists(LOG_OLD_FILE)) {
      File f = LittleFS.open(LOG_OLD_FILE, "r");
      if (f) { Serial.printf("log.old.csv: %u bytes\n", f.size()); f.close(); }
    }
    if (LittleFS.exists(QUEUE_FILE)) {
      File f = LittleFS.open(QUEUE_FILE, "r");
      if (f) { Serial.printf("queue.csv:   %u bytes (%d rows)\n", f.size(), queueCount); f.close(); }
    } else { Serial.println("queue.csv:   empty"); }
    Serial.printf("Clock:       %s (epoch: %lu)\n", getTimestamp().c_str(), (unsigned long)getEpochSeconds());
    Serial.println("--- end ---\n");

  } else if (input == "t") {
    uint32_t epoch = getEpochSeconds();
    Serial.printf("[TIME] Current : %s\n", getTimestamp().c_str());
    Serial.printf("[TIME] Epoch   : %lu s since 2026-01-01 PHT\n", (unsigned long)epoch);
    Serial.printf("[TIME] Encoded : %08lX (bytes 0-3 of payload)\n", (unsigned long)epoch);

  } else if (input == "p") {
    if (samples.count == 0) {
      Serial.println("[PAY] No samples yet - wait for first 60s window.");
    } else {
      float t  = samples.avg(samples.temp);
      float h  = samples.avg(samples.hum);
      float p  = samples.avg(samples.pres);
      float d1 = samples.avg(samples.dist1);
      float d2 = samples.avg(samples.dist2);
      float d  = samples.avg(samples.dist);
      float m  = samples.avg(samples.moist);
      uint32_t epoch = getEpochSeconds();

      char payload[37];
      snprintf(payload, sizeof(payload), "%08lX%04X%04X%04X%04X%04X%04X%04X",
        (unsigned long)epoch,
        (uint16_t)(t  * 10),
        (uint16_t)(h  * 10),
        (uint16_t)(p),
        (uint16_t)(d1 * 100),
        (uint16_t)(d2 * 100),
        (uint16_t)(d  * 100),
        (uint16_t)(m  * 10));

      Serial.println("\n[PAY] --- Dry-run payload (18 bytes / 36 hex chars) ---");
      Serial.printf( "[PAY] Raw hex  : %s\n", payload);
      Serial.println("[PAY] Decoded  :");
      Serial.printf( "[PAY]   epoch  : %lu s  -> %08lX  (bytes 0-3)\n",  (unsigned long)epoch, (unsigned long)epoch);
      Serial.printf( "[PAY]   temp   : %.2f C  -> %04X  (x10  = %d)\n",  t,  (uint16_t)(t*10),   (uint16_t)(t*10));
      Serial.printf( "[PAY]   hum    : %.2f %%  -> %04X  (x10  = %d)\n", h,  (uint16_t)(h*10),   (uint16_t)(h*10));
      Serial.printf( "[PAY]   pres   : %.1f hPa -> %04X  (x1   = %d)\n", p,  (uint16_t)(p),      (uint16_t)(p));
      Serial.printf( "[PAY]   dist1  : %.3f m  -> %04X  (x100 = %d)\n",  d1, (uint16_t)(d1*100), (uint16_t)(d1*100));
      Serial.printf( "[PAY]   dist2  : %.3f m  -> %04X  (x100 = %d)\n",  d2, (uint16_t)(d2*100), (uint16_t)(d2*100));
      Serial.printf( "[PAY]   dist   : %.3f m  -> %04X  (x100 = %d)\n",  d,  (uint16_t)(d*100),  (uint16_t)(d*100));
      Serial.printf( "[PAY]   moist  : %.1f %%  -> %04X  (x10  = %d)\n", m,  (uint16_t)(m*10),   (uint16_t)(m*10));
      Serial.printf( "[PAY] Timestamp: %s\n", getTimestamp().c_str());
      Serial.printf( "[PAY] AT cmd   : AT+DTRX=0,1,18,%s\n", payload);
      Serial.println("[PAY] ---\n");
    }

  } else if (input.startsWith("settime ")) {
    int y, mo, d, h, mi, s;
    int parsed = sscanf(input.c_str() + 8, "%d/%d/%d %d:%d:%d",
                        &y, &mo, &d, &h, &mi, &s);
    if (parsed == 6
        && y  >= 2026 && y  <= 2099
        && mo >= 1    && mo <= 12
        && d  >= 1    && d  <= 31
        && h  >= 0    && h  <= 23
        && mi >= 0    && mi <= 59
        && s  >= 0    && s  <= 59) {
      clk_year    = y;  clk_month   = mo; clk_day     = d;
      clk_hours   = h;  clk_minutes = mi; clk_seconds = s;
      clk_prevMillis = millis();
      Serial.printf("[TIME] Clock set to: %s\n", getTimestamp().c_str());
    } else {
      Serial.println("[TIME] Bad format. Use: settime YYYY/MM/DD HH:MM:SS");
      Serial.println("[TIME] Example:        settime 2026/04/19 14:30:00");
    }

  } else if (input.length() > 0) {
    Serial.println("Commands:");
    Serial.println("  l                            - dump log.csv");
    Serial.println("  q                            - dump queue.csv");
    Serial.println("  s                            - storage stats");
    Serial.println("  x                            - wipe log.csv and queue.csv");
    Serial.println("  t                            - show current clock/epoch");
    Serial.println("  p                            - dry-run payload (show encoding)");
    Serial.println("  settime YYYY/MM/DD HH:MM:SS  - set clock");
  }
}

// ================= [DB] Filesystem Init =================
void fsInit() {
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS mount FAILED"); return;
  }
  Serial.println("[FS] LittleFS mounted OK");
  if (!LittleFS.exists(LOG_FILE)) {
    File f = LittleFS.open(LOG_FILE, "w");
    if (f) {
      f.println("timestamp,moist_pct,temp_c,hum_pct,pres_hpa,dist1_m,dist2_m,dist_m");
      f.close();
      Serial.println("[FS] Created log.csv");
    }
  }
  queueCount = countQueueRows();
  size_t total = LittleFS.totalBytes();
  size_t used  = LittleFS.usedBytes();
  Serial.printf("[FS] Used: %u KB / %u KB (%.1f%%)\n",
    used/1024, total/1024, (used*100.0f)/total);
  Serial.printf("[FS] Queue: %d unsent rows (capacity: %d)\n", queueCount, MAX_QUEUE_ROWS);
  if (LittleFS.exists(LOG_FILE)) {
    File f = LittleFS.open(LOG_FILE, "r");
    if (f) {
      size_t logSz = f.size(); f.close();
      Serial.printf("[FS] log.csv: %u KB (%.1f%% of rotation threshold)\n",
        logSz/1024, (logSz*100.0f)/MAX_LOG_SIZE);
    }
  }
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(SOIL_PIN, INPUT);

  Wire.begin(5, 4);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found, continuing...");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  if (!bme.begin(0x77)) {
    Serial.println("BME280 not found!");
  } else {
    Serial.println("BME280 OK");
  }

  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);

  clk_prevMillis = millis();
  Serial.printf("[TIME] Clock seeded: %s\n", getTimestamp().c_str());

  fsInit();

  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(1000);
  loraInit();

  Serial.println("Moist,Temp,Hum,Pres,Dist1,Dist2,Final | [S=sampled, TX=sent]");
  Serial.println("[SYS] Commands: l=log  q=queue  s=storage  x=wipe  t=time  p=payload  settime YYYY/MM/DD HH:MM:SS");
}

// ================= Loop =================
void loop() {
  tickClock();

  static unsigned long lastSampleMs = 0;
  static unsigned long lastSendMs   = 0;
  static float moisturePercent      = 0;

  handleSerialCommand();

  // ---- SAMPLE every 10 seconds ----
  if (millis() - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = millis();

    float temp = bme.readTemperature();
    float hum  = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F;

    float d1 = readUltrasonic(TRIG1_PIN, ECHO1_PIN);
    delay(70);
    float d2 = readUltrasonic(TRIG2_PIN, ECHO2_PIN);

    dist1Buf[smoothIdx] = d1;
    dist2Buf[smoothIdx] = d2;
    smoothIdx = (smoothIdx + 1) % SMOOTH_SIZE;

    float d1Smooth  = movingAverage(dist1Buf);
    float d2Smooth  = movingAverage(dist2Buf);
    float distFinal = combineDistances(d1Smooth, d2Smooth);

    int rawValue = analogRead(SOIL_PIN);
    moisturePercent = (float)constrain(
      map(rawValue, SOIL_DRY, SOIL_WET, 0, 100), 0, 100);

    samples.push(temp, hum, pres, d1Smooth, d2Smooth, distFinal, moisturePercent);

    Serial.printf("[S][%s] moist:%.1f%% %.2f,%.2f,%.2f,%.2f,%.2f,%.2f (buf:%d/6)\n",
      getTimestamp().c_str(),
      moisturePercent, temp, hum, pres, d1Smooth, d2Smooth, distFinal, samples.count);

    static unsigned long lastOledSwitch = 0;
    if (millis() - lastOledSwitch >= 10000) {
      displayPage = !displayPage;
      lastOledSwitch = millis();
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("%02d:%02d:%02d", clk_hours, clk_minutes, clk_seconds);
    display.setCursor(80, 0);
    display.print(loraReady ? "LoRa:OK" : "LoRa:--");
    display.setCursor(0, 10);
    display.printf("Buf:%d/6 Q:%d", samples.count, queueCount);
    display.setCursor(0, 20);
    if (!displayPage) {
      display.printf("Temp:%.1fC\n",  temp);
      display.printf("Hum: %.1f%%\n", hum);
      display.printf("Pres:%.1fhPa",  pres);
    } else {
      display.printf("D1: %.2fm\n",   d1Smooth);
      display.printf("D2: %.2fm\n",   d2Smooth);
      display.printf("Moist:%.0f%%",  moisturePercent);
    }
    display.display();
  }

  // ---- SEND averaged batch every 60 seconds ----
  if (millis() - lastSendMs >= SEND_INTERVAL_MS && samples.count > 0) {
    lastSendMs = millis();

    float avgTemp  = samples.avg(samples.temp);
    float avgHum   = samples.avg(samples.hum);
    float avgPres  = samples.avg(samples.pres);
    float avgDist1 = samples.avg(samples.dist1);
    float avgDist2 = samples.avg(samples.dist2);
    float avgDist  = samples.avg(samples.dist);
    float avgMoist = samples.avg(samples.moist);

    Serial.printf("[TX][%s] Sending avg of %d samples: moist:%.1f%% %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      getTimestamp().c_str(),
      samples.count, avgMoist, avgTemp, avgHum, avgPres, avgDist1, avgDist2, avgDist);

    String csvRow = buildCsvRow(avgMoist, avgTemp, avgHum, avgPres, avgDist1, avgDist2, avgDist);
    appendLog(csvRow);

    bool sentFromQueue = false;
    if (queueCount > 0) {
      String oldRow = peekQueueRow();
      if (oldRow.length() > 0) {
        // Column order: timestamp(skip), moist(0), temp(1), hum(2), pres(3), d1(4), d2(5), dist(6)
        float v[7] = {0};
        int fieldIdx = 0, start = 0, sensorFieldIdx = 0;
        for (int i = 0; i <= (int)oldRow.length(); i++) {
          if (i == (int)oldRow.length() || oldRow[i] == ',') {
            if (fieldIdx == 0) {
              fieldIdx++; start = i + 1; continue;  // skip timestamp
            }
            String field = oldRow.substring(start, i);
            if (sensorFieldIdx < 7) v[sensorFieldIdx] = field.toFloat();
            sensorFieldIdx++;
            fieldIdx++; start = i + 1;
          }
        }
        // v[0]=moist, v[1]=temp, v[2]=hum, v[3]=pres, v[4]=d1, v[5]=d2, v[6]=dist
        for (int i = 0; i < 7; i++) if (isnan(v[i])) v[i] = 0;
        if (v[4] < 0) v[4] = 0;
        if (v[5] < 0) v[5] = 0;
        if (v[6] < 0) v[6] = 0;

        uint32_t epoch = getEpochSeconds();
        char payload[37];
        snprintf(payload, sizeof(payload), "%08lX%04X%04X%04X%04X%04X%04X%04X",
          (unsigned long)epoch,
          (uint16_t)(v[1] * 10),   // temp
          (uint16_t)(v[2] * 10),   // hum
          (uint16_t)(v[3]),         // pres
          (uint16_t)(v[4] * 100),  // dist1
          (uint16_t)(v[5] * 100),  // dist2
          (uint16_t)(v[6] * 100),  // dist
          (uint16_t)(v[0] * 10));  // moist

        Serial.printf("[TX] Queue drain (%d rows) - replaying: %s\n", queueCount, payload);
        String cmd = "AT+DTRX=0,1,18,"; cmd += String(payload);
        // loraAT will drain RX window automatically after DTRX
        String response = loraAT(cmd, 8000);
        if (response.indexOf("OK+SENT") >= 0 || response.indexOf("OK+SEND") >= 0) {
          Serial.println("[LoRa] OK Queued uplink sent: " + String(payload));
          popQueueRow();
          appendQueue(csvRow);
          Serial.printf("[TX] Replay OK - fresh row queued at back (%d total)\n", queueCount);
          sentFromQueue = true;
        } else {
          Serial.println("[TX] Queue replay failed - queuing fresh row too");
          appendQueue(csvRow);
          sentFromQueue = true;
        }
      }
    }

    if (!sentFromQueue) {
      loraSend(avgTemp, avgHum, avgPres, avgDist1, avgDist2, avgDist, avgMoist);
    }
  }
}
