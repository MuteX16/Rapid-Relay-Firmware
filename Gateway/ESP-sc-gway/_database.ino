// =============================================================================
// _database.ino  —  Local SPIFFS storage for the ESP-sc-gway gateway
//
// Mirrors the [DB] pattern from the node firmware (DetectionZone_V1_with_TIME).
// Stores a decoded CSV log of every received payload, and a retry queue for
// packets that could not be forwarded to ChirpStack when it was offline.
//
// Files on SPIFFS:
//   /gwlog.csv       — permanent log, one row per received payload
//   /gwlog.old.csv   — previous log (kept after rotation)
//   /gwqueue.csv     — unsent UDP packets awaiting retry
//   /gwqueue_tmp.csv — temp file used during queue pop
//
// CSV columns — IDENTICAL to node log.csv (DetectionZone_V1_with_TIME):
//   timestamp,epoch_s,moist_pct,temp_c,hum_pct,pres_hpa,dist1_m,dist2_m,dist_m
//
// Payload layout (18 bytes, epoch-first) — matches loraSend() in node firmware:
//   [0-3]   epoch_s  uint32 BE  seconds since 2026-01-01 00:00:00 PHT
//   [4-5]   soil     uint16 BE  soil moisture % x10
//   [6-7]   temp     uint16 BE  temperature C x10
//   [8-9]   hum      uint16 BE  humidity % x10
//   [10-11] pres     uint16 BE  pressure hPa x1
//   [12-13] dist1    uint16 BE  ultrasonic 1 distance m x100
//   [14-15] dist2    uint16 BE  ultrasonic 2 distance m x100
//   [16-17] dist     uint16 BE  combined distance m x100
//
// Serial commands (same style as node):
//   l  - dump gwlog.csv
//   q  - dump gwqueue.csv
//   s  - storage stats + ChirpStack probe state
//   x  - wipe gwlog.csv and gwqueue.csv
// =============================================================================

#include <WiFiClient.h>

// ===================== [DB] Storage Config ===================================
#define GW_LOG_FILE       "/gwlog.csv"
#define GW_LOG_OLD_FILE   "/gwlog.old.csv"
#define GW_QUEUE_FILE     "/gwqueue.csv"
#define GW_QUEUE_TMP_FILE "/gwqueue_tmp.csv"

// ~1.68 MB rotation threshold — same as node
#define GW_MAX_LOG_SIZE   1761280UL

// Each queue entry holds the raw UDP buff_up bytes as a hex string.
// Max UDP packet is TX_BUFF_SIZE (1024 bytes) -> hex string = 2048 chars + newline.
// Cap at 200 entries to avoid filling SPIFFS.
#define GW_MAX_QUEUE_ROWS 200

int gwQueueCount = 0;   // in-memory count, synced from SPIFFS at boot

// =============================================================================
// [DB] ChirpStack reachability probe
//
// Problem: UDP is fire-and-forget. sendUdp() always returns true even when
// ChirpStack is completely offline (power failure, Pi still booting, service
// crashed, etc.). Without an active probe, packets get silently discarded
// instead of queued — so we lose data during every ChirpStack outage.
//
// Solution: Before any sendUdp() or queue retry, open a TCP connection to
// ChirpStack's HTTP port (8080). A successful TCP connect means the process is
// running and accepting connections. A failure means it is down — queue instead.
//
// Probe cache:
//   Successful probe results are cached for CS_PROBE_CACHE_MS (default 10s).
//   This prevents a 800ms TCP penalty on every received LoRa packet when
//   ChirpStack is healthy. Failed results are NEVER cached so that recovery
//   after a power-cycle is detected on the very next packet.
//
// Tuning:
//   CS_PROBE_PORT     — ChirpStack HTTP port (8080 by default in ChirpStack v4)
//   CS_PROBE_TIMEOUT  — TCP connect timeout in ms. 800ms is safe on LAN.
//                       Lower to 400ms if nodes send frequently and you want
//                       less gateway loop latency.
//   CS_PROBE_CACHE_MS — How long a successful result is reused. 10 000ms means
//                       at most one real TCP probe per 10s when things are up.
// =============================================================================
#define CS_PROBE_PORT       8080
#define CS_PROBE_TIMEOUT    200
#define CS_PROBE_CACHE_MS   30000UL

static bool     csLastResult   = false;
static uint32_t csLastProbeMs  = 0;
static bool     csCacheValid   = false;

// =============================================================================
// [DB] Offline retry backoff
//
// When ChirpStack goes offline the gateway queues packets.  Once the queue is
// non-empty we start probing on an exponential backoff schedule independently
// of _PULL_INTERVAL so recovery is fast without hammering the Pi.
//
// Schedule (probe intervals):
//   attempt 1 →  5 s
//   attempt 2 → 10 s
//   attempt 3 → 20 s
//   attempt 4 → 40 s
//   attempt 5+ → 60 s  (cap)
//
// On recovery up to GW_RETRY_BURST_MAX queued packets are forwarded in one
// tickGwRetry() call.  The backoff resets to 0 once the queue is empty.
// =============================================================================
#define GW_RETRY_BURST_MAX   10          // max packets drained per recovery burst
#define GW_RETRY_BASE_MS     5000UL      // first retry interval
#define GW_RETRY_MAX_MS      60000UL     // cap

static uint32_t gwRetryNextMs   = 0;    // millis() timestamp of next probe attempt
static uint8_t  gwRetryAttempt  = 0;    // how many consecutive failed attempts

static uint32_t gwRetryIntervalFor(uint8_t attempt) {
  uint32_t iv = GW_RETRY_BASE_MS;
  for (uint8_t i = 0; i < attempt; i++) {
    iv *= 2;
    if (iv >= GW_RETRY_MAX_MS) return GW_RETRY_MAX_MS;
  }
  return iv;
}

bool isChirpStackReachable() {
  uint32_t now_ms = millis();

  // Return cached result only when the last probe was a success AND is still fresh.
  // Failed results are never cached — we want to detect recovery immediately.
  if (csCacheValid && csLastResult && (now_ms - csLastProbeMs) < CS_PROBE_CACHE_MS) {
#if _MONITOR>=1
    if (debug>=2) { mPrint("[GWDB] CS probe: cached ONLINE"); }
#endif
    return true;
  }

  WiFiClient client;
  client.setTimeout(CS_PROBE_TIMEOUT);
  bool ok = client.connect(ttnServer, CS_PROBE_PORT);
  if (ok) client.stop();

  csLastResult  = ok;
  csLastProbeMs = now_ms;
  csCacheValid  = true;

#if _MONITOR>=1
  if (debug>=1) {
    mPrint(ok ? "[GWDB] CS probe: ONLINE" : "[GWDB] CS probe: OFFLINE - packet will be queued");
  }
#endif

  return ok;
}

// Call when sendUdp() fails despite a successful probe (e.g. network blip).
// Forces re-probe on the very next packet instead of trusting a stale cache.
void csProbeInvalidate() {
  csCacheValid = false;
#if _MONITOR>=1
  if (debug>=1) { mPrint("[GWDB] CS probe cache invalidated"); }
#endif
}

// =============================================================================
// [DB] Epoch -> "YYYY/MM/DD HH:MM:SS" string
//
// epoch_s = seconds since 2026-01-01 00:00:00 PHT (custom base used by node).
// Reconstructs the human-readable timestamp the node embedded in its log.csv
// so both files are directly comparable row-for-row.
// =============================================================================
static bool gwIsLeap(int y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}
static int gwDaysInMonth(int m, int y) {
  if (m == 2) return gwIsLeap(y) ? 29 : 28;
  if (m == 4 || m == 6 || m == 9 || m == 11) return 30;
  return 31;
}

String epochToTimestamp(uint32_t epoch_s) {
  uint32_t remaining = epoch_s;
  int year = 2026, month = 1, day = 1;

  while (true) {
    uint32_t diy = gwIsLeap(year) ? 366UL : 365UL;
    if (remaining < diy * 86400UL) break;
    remaining -= diy * 86400UL;
    year++;
  }
  while (true) {
    uint32_t dim = (uint32_t)gwDaysInMonth(month, year) * 86400UL;
    if (remaining < dim) break;
    remaining -= dim;
    month++;
  }
  day      += remaining / 86400UL;
  remaining %= 86400UL;
  int h  = remaining / 3600;
  int mi = (remaining % 3600) / 60;
  int s  = remaining % 60;

  char buf[22];
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
    year, month, day, h, mi, s);
  return String(buf);
}

// =============================================================================
// [DB] CSV row builder
// Output matches node log.csv exactly (sensor_node-v4):
//   timestamp,epoch_s,moist_pct,temp_c,hum_pct,pres_hpa,dist1_m,dist2_m,dist_m
// =============================================================================
String buildGwCsvRow(uint32_t epoch_s,
                     float moist, float temp, float hum,
                     float pres, float d1, float d2, float dist) {
  char buf[220];
  snprintf(buf, sizeof(buf),
    "%s,%lu,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    epochToTimestamp(epoch_s).c_str(),
    (unsigned long)epoch_s,
    moist, temp, hum, pres, d1, d2, dist);
  return String(buf);
}

// =============================================================================
// [DB] Decode raw FRMPayload bytes -> epoch + sensor floats
//
// 18-byte payload matching sensor_node-v4 loraSend():
//   [0-3]   epoch_s  uint32 BE   seconds since 2026-01-01 PHT
//   [4-5]   temp     uint16 BE / 10.0
//   [6-7]   hum      uint16 BE / 10.0
//   [8-9]   pres     uint16 BE (no divisor)
//   [10-11] dist1    uint16 BE / 100.0
//   [12-13] dist2    uint16 BE / 100.0
//   [14-15] dist     uint16 BE / 100.0
//   [16-17] moist    uint16 BE / 10.0
//
// Returns true if datal >= 18, false otherwise.
// =============================================================================
bool decodeGwPayload(uint8_t *data, uint8_t datal,
                     uint32_t &epoch_s,
                     float &moist, float &temp, float &hum,
                     float &pres, float &d1, float &d2, float &dist) {
  if (datal < 18) return false;

  epoch_s = ((uint32_t)data[0] << 24)
           | ((uint32_t)data[1] << 16)
           | ((uint32_t)data[2] <<  8)
           |  (uint32_t)data[3];

  temp  = (float)((data[4]  << 8) | data[5])  / 10.0f;
  hum   = (float)((data[6]  << 8) | data[7])  / 10.0f;
  pres  = (float)((data[8]  << 8) | data[9]);
  d1    = (float)((data[10] << 8) | data[11]) / 100.0f;
  d2    = (float)((data[12] << 8) | data[13]) / 100.0f;
  dist  = (float)((data[14] << 8) | data[15]) / 100.0f;
  moist = (float)((data[16] << 8) | data[17]) / 10.0f;
  return true;
}

// =============================================================================
// [DB] Log helpers
// =============================================================================
void appendGwLog(const String& row) {
  File f = SPIFFS.open(GW_LOG_FILE, "a");
  if (!f) { Serial.println("[GWDB] Cannot open gwlog.csv"); return; }
  f.seek(0, SeekEnd);
  size_t sz = f.position();
  f.close();

  if (sz >= GW_MAX_LOG_SIZE) {
    SPIFFS.remove(GW_LOG_OLD_FILE);
    SPIFFS.rename(GW_LOG_FILE, GW_LOG_OLD_FILE);
    Serial.println("[GWDB] gwlog.csv rotated -> gwlog.old.csv");
    File nf = SPIFFS.open(GW_LOG_FILE, "w");
    if (nf) {
      nf.println("timestamp,epoch_s,moist_pct,temp_c,hum_pct,pres_hpa,dist1_m,dist2_m,dist_m");
      nf.close();
    }
  }

  File af = SPIFFS.open(GW_LOG_FILE, "a");
  if (af) { af.print(row); af.close(); }
}

// =============================================================================
// [DB] Queue helpers — stores raw UDP packets (hex-encoded) for ChirpStack retry
// =============================================================================

String bytesToHex(uint8_t *buf, uint16_t len) {
  String out;
  out.reserve(len * 2 + 2);
  for (uint16_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) out += '0';
    out += String(buf[i], HEX);
  }
  return out;
}

uint16_t hexToBytes(const String& hex, uint8_t *buf, uint16_t maxLen) {
  uint16_t len = hex.length() / 2;
  if (len > maxLen) len = maxLen;
  for (uint16_t i = 0; i < len; i++) {
    buf[i] = (uint8_t) strtol(hex.substring(i * 2, i * 2 + 2).c_str(), nullptr, 16);
  }
  return len;
}

void appendGwQueue(uint8_t *buf, uint16_t len) {
  if (gwQueueCount >= GW_MAX_QUEUE_ROWS) {
    // Drop oldest entry to make room
    File src = SPIFFS.open(GW_QUEUE_FILE, "r");
    if (!src) { Serial.println("[GWDB] appendGwQueue: cannot open queue"); return; }
    src.readStringUntil('\n');
    File tmp = SPIFFS.open(GW_QUEUE_TMP_FILE, "w");
    if (!tmp) { src.close(); Serial.println("[GWDB] appendGwQueue: cannot open tmp"); return; }
    uint8_t rbuf[128];
    while (src.available()) {
      int n = src.read(rbuf, sizeof(rbuf));
      if (n > 0) tmp.write(rbuf, n);
    }
    src.close(); tmp.close();
    SPIFFS.remove(GW_QUEUE_FILE);
    SPIFFS.rename(GW_QUEUE_TMP_FILE, GW_QUEUE_FILE);
    if (gwQueueCount > 0) gwQueueCount--;
    Serial.println("[GWDB] Queue full - dropped oldest entry");
  }

  String hexRow = bytesToHex(buf, len) + "\n";
  File qf = SPIFFS.open(GW_QUEUE_FILE, "a");
  if (qf) { qf.print(hexRow); qf.close(); gwQueueCount++; }
  Serial.printf("[GWDB] Queued UDP packet (total: %d / %d)\n", gwQueueCount, GW_MAX_QUEUE_ROWS);
}

String peekGwQueueRow() {
  if (gwQueueCount == 0) return "";
  File qf = SPIFFS.open(GW_QUEUE_FILE, "r");
  if (!qf) return "";
  String line = qf.readStringUntil('\n');
  qf.close();
  line.trim();
  return line;
}

void popGwQueueRow() {
  File src = SPIFFS.open(GW_QUEUE_FILE, "r");
  if (!src) return;
  src.readStringUntil('\n');
  File tmp = SPIFFS.open(GW_QUEUE_TMP_FILE, "w");
  if (!tmp) { src.close(); Serial.println("[GWDB] popGwQueue: cannot open tmp"); return; }
  uint8_t buf[128];
  while (src.available()) {
    int n = src.read(buf, sizeof(buf));
    if (n > 0) tmp.write(buf, n);
  }
  src.close(); tmp.close();
  SPIFFS.remove(GW_QUEUE_FILE);
  SPIFFS.rename(GW_QUEUE_TMP_FILE, GW_QUEUE_FILE);
  if (gwQueueCount > 0) gwQueueCount--;
  Serial.printf("[GWDB] Popped queue entry (remaining: %d)\n", gwQueueCount);
}

int countGwQueueRows() {
  File qf = SPIFFS.open(GW_QUEUE_FILE, "r");
  if (!qf) return 0;
  int n = 0;
  while (qf.available()) { qf.readStringUntil('\n'); n++; }
  qf.close();
  return n;
}

// =============================================================================
// [DB] Try to forward one queued packet to ChirpStack.
//
// Called from:
//   - receivePacket()  — opportunistic: after every successful fresh-packet send
//   - loop()           — periodic: every _PULL_INTERVAL seconds, up to 3 per tick
//
// Guarded by isChirpStackReachable() so we never attempt sendUdp() while
// ChirpStack is still offline (Pi powered off / still booting / service down).
// The TCP probe adds at most CS_PROBE_TIMEOUT ms (800ms) when CS is down;
// when CS is up the result is cached for CS_PROBE_CACHE_MS (10s) so normal
// operation has negligible overhead.
// =============================================================================
void tryForwardGwQueued() {
  if (gwQueueCount == 0) return;

  // [DB] Probe ChirpStack before retry — UDP fire-and-forget cannot do this.
  if (!isChirpStackReachable()) {
#if _MONITOR>=1
    if (debug>=1) { mPrint("[GWDB] Retry skipped - ChirpStack still offline"); }
#endif
    return;
  }

  String hexRow = peekGwQueueRow();
  if (hexRow.length() == 0) return;

  static uint8_t retryBuf[TX_BUFF_SIZE];
  uint16_t retryLen = hexToBytes(hexRow, retryBuf, TX_BUFF_SIZE);
  if (retryLen == 0) {
    Serial.println("[GWDB] tryForward: bad hex row, discarding");
    popGwQueueRow();
    return;
  }

#ifdef _TTNSERVER
  if (sendUdp(ttnServer, _TTNPORT, retryBuf, retryLen)) {
    Serial.printf("[GWDB] Replayed queued packet to ChirpStack (%d remaining)\n", gwQueueCount - 1);
    popGwQueueRow();
  } else {
    // sendUdp failed even though probe passed — network blip or ChirpStack
    // just went down between probe and send. Invalidate cache so the next
    // call re-probes instead of trusting the now-stale result.
    csProbeInvalidate();
    Serial.println("[GWDB] Retry sendUdp failed after probe - cache invalidated");
  }
#endif
}

// =============================================================================
// [DB] tickGwRetry()
//
// Call from loop() on every iteration (it returns immediately when nothing
// needs to be done).  Drives the offline-retry backoff independently of
// _PULL_INTERVAL so the gateway recovers quickly when ChirpStack comes back.
//
// State transitions:
//   queue empty          → does nothing, resets backoff counter
//   queue non-empty
//     + not yet time     → returns immediately (non-blocking)
//     + time reached
//         probe FAIL     → schedules next attempt with doubled interval
//         probe OK       → drains up to GW_RETRY_BURST_MAX packets, resets
// =============================================================================
void tickGwRetry() {
  // Nothing queued — keep backoff reset so first failure starts fresh.
  if (gwQueueCount == 0) {
    gwRetryAttempt = 0;
    gwRetryNextMs  = 0;
    return;
  }

  uint32_t now_ms = millis();

  // Not time yet — return without blocking.
  if (gwRetryNextMs != 0 && now_ms < gwRetryNextMs) return;

  // Time to probe.
#if _MONITOR>=1
  if (debug>=1) {
    mPrint("[GWDB] tickGwRetry: probing CS (attempt " + String(gwRetryAttempt + 1) +
           ", queue=" + String(gwQueueCount) + ")");
  }
#endif

  if (!isChirpStackReachable()) {
    // Still offline — back off and wait longer.
    gwRetryAttempt++;
    uint32_t iv = gwRetryIntervalFor(gwRetryAttempt);
    gwRetryNextMs = now_ms + iv;
#if _MONITOR>=1
    if (debug>=0) {
      mPrint("[GWDB] tickGwRetry: CS still OFFLINE — next retry in " +
             String(iv / 1000) + "s (attempt " + String(gwRetryAttempt) + ")");
    }
#endif
    return;
  }

  // ChirpStack is back online — drain a burst of queued packets.
#if _MONITOR>=1
  if (debug>=0) {
    mPrint("[GWDB] tickGwRetry: CS ONLINE — draining up to " +
           String(GW_RETRY_BURST_MAX) + " queued packets");
  }
#endif

  int sent = 0;
  for (int i = 0; i < GW_RETRY_BURST_MAX && gwQueueCount > 0; i++) {
    String hexRow = peekGwQueueRow();
    if (hexRow.length() == 0) break;

    static uint8_t retryBuf[TX_BUFF_SIZE];
    uint16_t retryLen = hexToBytes(hexRow, retryBuf, TX_BUFF_SIZE);
    if (retryLen == 0) {
      Serial.println("[GWDB] tickGwRetry: bad hex row, discarding");
      popGwQueueRow();
      continue;
    }

#ifdef _TTNSERVER
    if (sendUdp(ttnServer, _TTNPORT, retryBuf, retryLen)) {
      popGwQueueRow();
      sent++;
    } else {
      // sendUdp failed mid-burst — CS may have just dropped again.
      csProbeInvalidate();
      gwRetryAttempt = 0;                             // restart backoff from short end
      gwRetryNextMs  = now_ms + GW_RETRY_BASE_MS;
#if _MONITOR>=1
      if (debug>=0) {
        mPrint("[GWDB] tickGwRetry: sendUdp failed mid-burst — pausing retry");
      }
#endif
      break;
    }
#endif
    yield();
  }

  if (sent > 0) {
#if _MONITOR>=1
    if (debug>=0) {
      mPrint("[GWDB] tickGwRetry: forwarded " + String(sent) +
             " packet(s), " + String(gwQueueCount) + " remaining");
    }
#endif
  }

  // If queue is now empty, reset backoff fully.
  if (gwQueueCount == 0) {
    gwRetryAttempt = 0;
    gwRetryNextMs  = 0;
  } else {
    // More packets remain — schedule the next burst quickly (base interval).
    gwRetryAttempt = 0;
    gwRetryNextMs  = now_ms + GW_RETRY_BASE_MS;
  }
}

// =============================================================================
// [DB] Serial monitor commands
// =============================================================================
void handleGwSerialCmd() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "l") {
    Serial.println("\n--- gwlog.csv ---");
    File f = SPIFFS.open(GW_LOG_FILE, "r");
    if (!f) { Serial.println("File not found"); }
    else { while (f.available()) Serial.write(f.read()); f.close(); }
    Serial.println("--- end ---\n");

  } else if (input == "q") {
    Serial.println("\n--- gwqueue.csv ---");
    File f = SPIFFS.open(GW_QUEUE_FILE, "r");
    if (!f) { Serial.println("File not found or empty"); }
    else { while (f.available()) Serial.write(f.read()); f.close(); }
    Serial.printf("--- end (gwQueueCount=%d) ---\n\n", gwQueueCount);

  } else if (input == "x") {
    SPIFFS.remove(GW_LOG_FILE);
    SPIFFS.remove(GW_QUEUE_FILE);
    gwQueueCount = 0;
    csProbeInvalidate();
    File f = SPIFFS.open(GW_LOG_FILE, "w");
    if (f) {
      f.println("timestamp,epoch_s,moist_pct,temp_c,hum_pct,pres_hpa,dist1_m,dist2_m,dist_m");
      f.close();
    }
    Serial.println("[GWDB] Files wiped. gwlog.csv recreated with header.");

  } else if (input == "s") {
    Serial.println("\n--- gateway storage ---");
    size_t spiffsTotal = SPIFFS.totalBytes();
    size_t spiffsUsed  = SPIFFS.usedBytes();
    Serial.printf("Used:          %u KB / %u KB (%.1f%%)\n",
      (unsigned)(spiffsUsed / 1024),
      (unsigned)(spiffsTotal / 1024),
      (spiffsTotal > 0) ? (spiffsUsed * 100.0f) / spiffsTotal : 0.0f);
    if (SPIFFS.exists(GW_LOG_FILE)) {
      File f = SPIFFS.open(GW_LOG_FILE, "r");
      if (f) { Serial.printf("gwlog.csv:     %u bytes\n", f.size()); f.close(); }
    }
    if (SPIFFS.exists(GW_LOG_OLD_FILE)) {
      File f = SPIFFS.open(GW_LOG_OLD_FILE, "r");
      if (f) { Serial.printf("gwlog.old.csv: %u bytes\n", f.size()); f.close(); }
    }
    if (SPIFFS.exists(GW_QUEUE_FILE)) {
      File f = SPIFFS.open(GW_QUEUE_FILE, "r");
      if (f) { Serial.printf("gwqueue.csv:   %u bytes (%d entries)\n", f.size(), gwQueueCount); f.close(); }
    } else {
      Serial.println("gwqueue.csv:   empty");
    }
    Serial.printf("CS probe:      last=%s  cache=%s\n",
      csLastResult ? "ONLINE" : "OFFLINE",
      csCacheValid ? "valid" : "invalid");
    Serial.println("--- end ---\n");

  } else if (input.length() > 0) {
    Serial.println("Gateway DB commands:");
    Serial.println("  l  - dump gwlog.csv");
    Serial.println("  q  - dump gwqueue.csv");
    Serial.println("  s  - storage stats + CS probe state");
    Serial.println("  x  - wipe gwlog.csv and gwqueue.csv");
  }
}

// =============================================================================
// [DB] Filesystem init — called once from setup() after SPIFFS.begin()
// =============================================================================
void fsGwInit() {
  if (!SPIFFS.exists(GW_LOG_FILE)) {
    File f = SPIFFS.open(GW_LOG_FILE, "w");
    if (f) {
      f.println("timestamp,epoch_s,moist_pct,temp_c,hum_pct,pres_hpa,dist1_m,dist2_m,dist_m");
      f.close();
      Serial.println("[GWDB] Created gwlog.csv");
    }
  }

  gwQueueCount = countGwQueueRows();

  size_t spiffsTotal = SPIFFS.totalBytes();
  size_t spiffsUsed  = SPIFFS.usedBytes();
  Serial.printf("[GWDB] SPIFFS used: %u KB / %u KB (%.1f%%)\n",
    (unsigned)(spiffsUsed / 1024),
    (unsigned)(spiffsTotal / 1024),
    (spiffsTotal > 0) ? (spiffsUsed * 100.0f) / spiffsTotal : 0.0f);
  Serial.printf("[GWDB] Queue: %d unsent packets (cap: %d)\n", gwQueueCount, GW_MAX_QUEUE_ROWS);

  if (SPIFFS.exists(GW_LOG_FILE)) {
    File f = SPIFFS.open(GW_LOG_FILE, "r");
    if (f) {
      size_t logSz = f.size(); f.close();
      Serial.printf("[GWDB] gwlog.csv: %u KB (%.1f%% of rotation threshold)\n",
        (unsigned)(logSz / 1024), (logSz * 100.0f) / GW_MAX_LOG_SIZE);
    }
  }

  Serial.println("[GWDB] Commands: l=log  q=queue  s=storage  x=wipe");

  // [DB] No boot probe here — WiFiClient cannot be used this early in setup()
  // before the UDP stack semaphores are initialized (causes xQueueSemaphoreTake
  // assert crash). The probe cache starts invalid, so the first received LoRa
  // packet or the first _PULL_INTERVAL tick will probe naturally.
  Serial.println("[GWDB] CS probe deferred until first packet (WiFi not ready yet)");
}
