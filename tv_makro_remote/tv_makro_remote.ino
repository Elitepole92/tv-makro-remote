// ===========================================================================
// TV-Makro-Fernbedienung fuer M5Stack StickS3
// ===========================================================================
//
// Was kann das?
//   - Tastendruecke einer beliebigen IR-Fernbedienung als MAKRO AUFNEHMEN
//     (inkl. der Pausen dazwischen) und spaeter per Knopfdruck abspielen.
//   - Makros sind nach TV-MODELLEN sortiert (z.B. "Samsung Halle", "LG Buero").
//   - Gespeichert werden die Rohsignale -> funktioniert mit jeder TV-Marke.
//   - Alles direkt am Stick bedienbar. Optional Web-Oberflaeche zum
//     Umbenennen/Sortieren/Loeschen (Stick macht eigenes WLAN auf).
//
// Arduino IDE (wie beim Delfino-Stick):
//   1. Boardverwalter: M5Stack-Boardpaket "M5Stack" installieren
//      (Boardverwalter-URL: https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json)
//   2. Bibliotheksverwalter: "M5Unified" installieren
//   3. Board "M5StickS3" waehlen, Port waehlen, Hochladen.
//
// Bedienung am Stick (hochkant):
//   Taste A (vorne)   kurz : Auswaehlen / Abspielen     lang : Optionen (neu aufnehmen, loeschen)
//   Taste B (Seite)   kurz : naechster Eintrag          lang : Menue (WLAN, Update, Ausschalten)
//   Power-Taste       kurz : ganz aus (bleibt aus, bis man Power wieder drueckt)
//
// Strom sparen: nach 10 s wird das Display dunkler, nach 30 s schlaeft der Stick
// tief (Display aus, Prozessor aus). Taste A oder B weckt ihn - der erste Druck
// weckt nur, er steht dann wieder beim zuletzt benutzten Makro.
//
// Updates ohne Laptop (holt sich der Stick selbst von GitHub):
//   1. Einmalig: am Stick B lang -> "WLAN / Browser" -> am iPhone ins WLAN "TV-Remote-xxxx"
//      -> im Browser 192.168.4.1 -> unten bei "Updates" den iPhone-Hotspot eintragen.
//   2. Spaeter: iPhone-Hotspot an (Einstellungen -> Persoenlicher Hotspot,
//      "Maximale Kompatibilitaet" an) -> am Stick B lang -> "Update".
//      Er laedt die neueste Version von GitHub und startet neu.
//   Makros und TV-Modelle bleiben dabei erhalten.
//
// Aufnehmen:
//   TV-Modell oeffnen -> "Neues Makro aufnehmen" -> Original-Fernbedienung aus
//   ca. 20-30 cm auf die OBERSEITE des Sticks richten und die Tasten in Ruhe
//   nacheinander druecken (z.B. Menue, runter, runter, OK, ...). Danach A = fertig.
//   Beim Abspielen werden die Tasten mit denselben Pausen wieder gesendet.
//   Tipp: zwischen Tasten im TV-Menue lieber etwas zu langsam als zu schnell druecken.
// ===========================================================================

#include <Arduino.h>
#include <M5Unified.h>
#include <esp_ota_ops.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_partition.h>
#include <esp_timer.h>
#include <esp_mac.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <vector>

#if __has_include("version.h")
#include "version.h"  // legt GitHub beim Bauen an
#endif
#ifndef FW_BUILD
#define FW_BUILD 0  // fortlaufende Build-Nummer (nur fuer den Update-Vergleich)
#endif
#ifndef FW_NAME
#define FW_NAME "dev"  // sichtbare Version, kommt aus der Datei VERSION
#endif

// ---------------------------------------------------------------------------
// Einstellungen
// ---------------------------------------------------------------------------
static constexpr int PIN_IR_TX = 46;  // IR-LED des StickS3
static constexpr int PIN_IR_RX = 42;  // IR-Empfaenger des StickS3
static constexpr uint32_t IR_CARRIER_HZ = 38000;

static constexpr const char *AP_PREFIX = "TV-Remote-";
static constexpr const char *AP_PASS = "fernbedienung";  // mind. 8 Zeichen

static constexpr size_t MAX_TVS = 20;
static constexpr size_t MAX_MACROS = 30;           // pro TV-Modell
static constexpr size_t MAX_STEPS = 400;           // IR-Signale pro Makro
static constexpr size_t MAX_TOTAL_BYTES = 120000;  // Gesamtspeicher fuer alle Makros
static constexpr uint32_t MAX_GAP_MS = 30000;      // laengere Pausen werden gekuerzt
static constexpr uint32_t DISPLAY_DIM_MS = 10000;  // Display dunkler nach 10 s
static constexpr uint32_t DISPLAY_OFF_MS = 30000;  // nach 30 s: Display aus + Tiefschlaf
static constexpr uint32_t AP_IDLE_MS = 300000;     // Stick-WLAN geht aus, wenn 5 min niemand verbunden ist
static constexpr uint8_t BRIGHT_ON = 110;          // Helligkeit (0-255): weniger = laenger Akku
static constexpr uint8_t BRIGHT_DIM = 20;
static constexpr int PIN_BTN_A = 11;               // Tasten des StickS3 (wecken aus dem Tiefschlaf)
static constexpr int PIN_BTN_B = 12;
static constexpr uint32_t CPU_MHZ = 80;            // reicht fuer IR + Anzeige, spart Strom
static constexpr const char *COPYRIGHT = "(c) 2026 Daniel Lassak";
// Hier holt sich der Stick Updates (GitHub-Release: firmware.bin + version.txt)
static constexpr const char *UPDATE_URL = "https://github.com/Elitepole92/tv-makro-remote/releases/latest/download/";

// ---------------------------------------------------------------------------
// Datenmodell
// ---------------------------------------------------------------------------
struct Step {
  uint32_t at;                // Startzeit relativ zum Makrobeginn (ms)
  std::vector<uint16_t> raw;  // Puls/Pause-Zeiten in us (beginnt mit Puls)
};

struct Macro {
  String name;
  std::vector<Step> steps;
};

struct Tv {
  String name;
  std::vector<Macro> macros;
};

struct FlashHeader {
  uint32_t magic;
  uint32_t seq;
  uint32_t len;
  uint32_t crc;
};

struct RxEvent {
  size_t n;
  int64_t timeUs;
  int buf;
};

struct RxFrame {
  std::vector<uint16_t> raw;
  int64_t startUs = 0;
};

struct Reader {
  const uint8_t *p;
  size_t n;
  size_t pos;
  bool ok;
};

enum class Screen : uint8_t { Tvs, Macros, Options, Confirm, Recording, Playing, Wifi, Update, Menu };

// Vorwaertsdeklarationen (Web <-> Bedienung)
namespace web {
void start();
void stop();
void loop();
String ssid();
bool isRunning();
}  // namespace web

// ===========================================================================
// Infrarot: Senden und Empfangen ueber das RMT-Peripheral
// ===========================================================================
namespace ir {

constexpr uint32_t RES_HZ = 1000000;  // 1 Tick = 1 us
constexpr size_t RX_SYMBOLS = 96;
constexpr size_t MAX_PENDING = 16;

rmt_channel_handle_t txChan = nullptr;
rmt_channel_handle_t rxChan = nullptr;
rmt_encoder_handle_t copyEnc = nullptr;
volatile bool rxEnabled = false;

// Doppelpuffer: waehrend ein Telegramm ausgewertet wird, empfaengt der andere Puffer schon weiter
rmt_symbol_word_t rxBuf[2][RX_SYMBOLS];
int armedBuf = 0;
QueueHandle_t rxQueue = nullptr;
SemaphoreHandle_t pendingLock = nullptr;
std::vector<RxFrame> pending;

static bool IRAM_ATTR onRxDone(rmt_channel_handle_t, const rmt_rx_done_event_data_t *ed, void *) {
  RxEvent ev;
  ev.n = ed->num_symbols;
  ev.timeUs = esp_timer_get_time();
  ev.buf = (ed->received_symbols == rxBuf[1]) ? 1 : 0;
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(rxQueue, &ev, &woken);
  return woken == pdTRUE;
}

static void arm(int buf) {
  rmt_receive_config_t rc = {};
  rc.signal_range_min_ns = 1250;      // kuerzere Stoerimpulse ignorieren
  rc.signal_range_max_ns = 12000000;  // 12 ms Ruhe = Ende eines IR-Telegramms
  rmt_receive(rxChan, rxBuf[buf], sizeof(rxBuf[buf]), &rc);
}

// RMT-Symbole -> abwechselnd Puls/Pause in us. Empfaenger ist active-low (0 = Licht).
static void convert(const rmt_symbol_word_t *sym, size_t n, int64_t endUs, RxFrame &f) {
  f.raw.clear();
  int lastMark = -1;
  uint32_t total = 0;
  for (size_t i = 0; i < n; i++) {
    for (int half = 0; half < 2; half++) {
      uint16_t dur = half ? sym[i].duration1 : sym[i].duration0;
      int level = half ? sym[i].level1 : sym[i].level0;
      if (dur == 0) continue;
      int mark = (level == 0) ? 1 : 0;
      if (f.raw.empty() && !mark) continue;  // fuehrende Pause verwerfen
      if (mark == lastMark) {
        uint32_t sum = (uint32_t)f.raw.back() + dur;
        f.raw.back() = sum > 0xFFFF ? 0xFFFF : sum;
      } else {
        f.raw.push_back(dur);
        lastMark = mark;
      }
      total += dur;
    }
  }
  if (!f.raw.empty() && f.raw.size() % 2 == 0) {
    total -= f.raw.back();
    f.raw.pop_back();  // mit einem Puls enden
  }
  f.startUs = endUs - 12000 - (int64_t)total;
}

// Eigene Task: sofort neu scharf schalten, damit auch schnelle Wiederholungen
// (z.B. Sony: 3 Telegramme im Abstand von ~45 ms) vollstaendig erfasst werden.
static void rxTask(void *) {
  RxEvent ev;
  RxFrame f;
  for (;;) {
    if (xQueueReceive(rxQueue, &ev, portMAX_DELAY) != pdTRUE) continue;
    if (!rxEnabled) continue;
    armedBuf = 1 - ev.buf;
    arm(armedBuf);
    size_t n = ev.n > RX_SYMBOLS ? RX_SYMBOLS : ev.n;
    convert(rxBuf[ev.buf], n, ev.timeUs, f);
    if (f.raw.size() < 3) continue;  // Stoerung
    xSemaphoreTake(pendingLock, portMAX_DELAY);
    if (pending.size() < MAX_PENDING) pending.push_back(f);
    xSemaphoreGive(pendingLock);
  }
}

bool begin() {
  rmt_tx_channel_config_t txc = {};
  txc.gpio_num = (gpio_num_t)PIN_IR_TX;
  txc.clk_src = RMT_CLK_SRC_DEFAULT;
  txc.resolution_hz = RES_HZ;
  txc.mem_block_symbols = 48;
  txc.trans_queue_depth = 4;
  if (rmt_new_tx_channel(&txc, &txChan) != ESP_OK) return false;

  rmt_carrier_config_t cc = {};
  cc.frequency_hz = IR_CARRIER_HZ;
  cc.duty_cycle = 0.33f;
  rmt_apply_carrier(txChan, &cc);

  rmt_copy_encoder_config_t ec = {};
  if (rmt_new_copy_encoder(&ec, &copyEnc) != ESP_OK) return false;
  rmt_enable(txChan);

  rmt_rx_channel_config_t rxc = {};
  rxc.gpio_num = (gpio_num_t)PIN_IR_RX;
  rxc.clk_src = RMT_CLK_SRC_DEFAULT;
  rxc.resolution_hz = RES_HZ;
  rxc.mem_block_symbols = RX_SYMBOLS;
  if (rmt_new_rx_channel(&rxc, &rxChan) != ESP_OK) {
    rxc.mem_block_symbols = 48;  // Fallback: kleinerer Hardware-Puffer
    if (rmt_new_rx_channel(&rxc, &rxChan) != ESP_OK) return false;
  }
  rxQueue = xQueueCreate(8, sizeof(RxEvent));
  pendingLock = xSemaphoreCreateMutex();
  if (!rxQueue || !pendingLock) return false;
  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = onRxDone;
  if (rmt_rx_register_event_callbacks(rxChan, &cbs, nullptr) != ESP_OK) return false;
  return xTaskCreate(rxTask, "ir_rx", 4096, nullptr, 10, nullptr) == pdPASS;
}

void startReceive() {
  if (!rxChan || rxEnabled) return;
  xSemaphoreTake(pendingLock, portMAX_DELAY);
  pending.clear();
  xSemaphoreGive(pendingLock);
  xQueueReset(rxQueue);
  rmt_enable(rxChan);
  rxEnabled = true;
  armedBuf = 0;
  arm(armedBuf);
}

void stopReceive() {
  if (!rxEnabled) return;
  rxEnabled = false;
  rmt_disable(rxChan);
}

// Holt das naechste empfangene Telegramm ab (Rohzeiten + Startzeit in us)
bool poll(RxFrame &out) {
  if (!pendingLock) return false;
  bool got = false;
  xSemaphoreTake(pendingLock, portMAX_DELAY);
  if (!pending.empty()) {
    out = std::move(pending.front());
    pending.erase(pending.begin());
    got = true;
  }
  xSemaphoreGive(pendingLock);
  return got;
}

bool send(const std::vector<uint16_t> &raw) {
  if (!txChan || raw.empty()) return false;
  std::vector<rmt_symbol_word_t> sym;
  sym.reserve(raw.size() / 2 + 1);
  for (size_t i = 0; i < raw.size(); i += 2) {
    uint32_t mark = raw[i];
    uint32_t space = (i + 1 < raw.size()) ? raw[i + 1] : 100;
    rmt_symbol_word_t s;
    s.level0 = 1;
    s.duration0 = mark < 1 ? 1 : (mark > 32767 ? 32767 : mark);
    s.level1 = 0;
    s.duration1 = space < 1 ? 1 : (space > 32767 ? 32767 : space);
    sym.push_back(s);
  }
  rmt_transmit_config_t tc = {};
  if (rmt_transmit(txChan, copyEnc, sym.data(), sym.size() * sizeof(rmt_symbol_word_t), &tc) != ESP_OK) return false;
  return rmt_tx_wait_all_done(txChan, 1000) == ESP_OK;
}

}  // namespace ir

// ===========================================================================
// Speicher: alle TV-Modelle + Makros als ein Datenblock in der Daten-Partition.
// Zwei Slots im Wechsel -> bei Stromausfall waehrend des Speicherns bleibt
// der vorherige Stand erhalten.
// ===========================================================================
namespace store {

std::vector<Tv> tvs;
uint8_t selTv = 0;
uint8_t selMacro = 0;
bool wifiOn = false;

const esp_partition_t *part = nullptr;
uint32_t slotSize = 0;
int curSlot = -1;
uint32_t curSeq = 0;
constexpr uint32_t MAGIC = 0x314D5654;  // "TVM1"
constexpr uint8_t FORMAT = 1;

uint32_t checksum(const uint8_t *d, size_t n) {
  uint32_t c = 0xFFFFFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320 & (0 - (c & 1)));
  }
  return ~c;
}

void put8(std::vector<uint8_t> &b, uint8_t v) { b.push_back(v); }
void put16(std::vector<uint8_t> &b, uint16_t v) {
  b.push_back(v & 0xFF);
  b.push_back(v >> 8);
}
void put32(std::vector<uint8_t> &b, uint32_t v) {
  put16(b, v & 0xFFFF);
  put16(b, v >> 16);
}
void putStr(std::vector<uint8_t> &b, const String &s) {
  size_t n = s.length() > 64 ? 64 : s.length();
  put8(b, n);
  for (size_t i = 0; i < n; i++) b.push_back((uint8_t)s[i]);
}

uint8_t get8(Reader &r) {
  if (r.pos >= r.n) {
    r.ok = false;
    return 0;
  }
  return r.p[r.pos++];
}
uint16_t get16(Reader &r) {
  uint16_t lo = get8(r);
  uint16_t hi = get8(r);
  return lo | (hi << 8);
}
uint32_t get32(Reader &r) {
  uint32_t lo = get16(r);
  uint32_t hi = get16(r);
  return lo | (hi << 16);
}
String getStr(Reader &r) {
  uint8_t n = get8(r);
  String s;
  if (r.pos + n > r.n) {
    r.ok = false;
    return s;
  }
  s.reserve(n);
  for (uint8_t i = 0; i < n; i++) s += (char)r.p[r.pos++];
  return s;
}

size_t stepBytes(const Step &s) { return 6 + 2 * s.raw.size(); }

// Wie viele Byte Makrodaten insgesamt gespeichert werden koennen
size_t capacity() {
  size_t c = MAX_TOTAL_BYTES;
  if (slotSize > 4096 && slotSize - 4096 < c) c = slotSize - 4096;
  return c;
}

size_t totalBytes() {
  size_t t = 0;
  for (auto &tv : tvs)
    for (auto &m : tv.macros)
      for (auto &s : m.steps) t += stepBytes(s);
  return t;
}

std::vector<uint8_t> serialize() {
  std::vector<uint8_t> b;
  b.reserve(totalBytes() + 256);
  put8(b, FORMAT);
  put8(b, selTv);
  put8(b, selMacro);
  put8(b, wifiOn ? 1 : 0);
  put16(b, tvs.size());
  for (auto &t : tvs) {
    putStr(b, t.name);
    put16(b, t.macros.size());
    for (auto &m : t.macros) {
      putStr(b, m.name);
      put16(b, m.steps.size());
      for (auto &s : m.steps) {
        put32(b, s.at);
        put16(b, s.raw.size());
        for (auto v : s.raw) put16(b, v);
      }
    }
  }
  return b;
}

bool deserialize(const uint8_t *p, size_t n) {
  Reader r = {p, n, 0, true};
  if (get8(r) != FORMAT) return false;
  uint8_t st = get8(r), sm = get8(r);
  bool w = get8(r) != 0;
  uint16_t nt = get16(r);
  if (!r.ok || nt > MAX_TVS) return false;
  std::vector<Tv> out(nt);
  for (auto &t : out) {
    t.name = getStr(r);
    uint16_t nm = get16(r);
    if (!r.ok || nm > MAX_MACROS) return false;
    t.macros.resize(nm);
    for (auto &m : t.macros) {
      m.name = getStr(r);
      uint16_t ns = get16(r);
      if (!r.ok || ns > MAX_STEPS) return false;
      m.steps.resize(ns);
      for (auto &s : m.steps) {
        s.at = get32(r);
        uint16_t nr = get16(r);
        if (!r.ok || r.pos + 2 * (size_t)nr > r.n) return false;
        s.raw.resize(nr);
        for (auto &v : s.raw) v = get16(r);
      }
    }
  }
  if (!r.ok) return false;
  tvs = std::move(out);
  selTv = st;
  selMacro = sm;
  wifiOn = w;
  return true;
}

bool readHeader(int slot, FlashHeader &h) {
  if (esp_partition_read(part, slot * slotSize, &h, sizeof(h)) != ESP_OK) return false;
  return h.magic == MAGIC && h.len > 0 && h.len <= slotSize - sizeof(FlashHeader);
}

bool load() {
  FlashHeader h[2];
  bool valid[2];
  for (int s = 0; s < 2; s++) valid[s] = readHeader(s, h[s]);
  int first = 0;
  if (!valid[0] || (valid[1] && (int32_t)(h[1].seq - h[0].seq) > 0)) first = 1;
  for (int i = 0; i < 2; i++) {
    int s = i == 0 ? first : 1 - first;
    if (!valid[s]) continue;
    std::vector<uint8_t> buf(h[s].len);
    if (esp_partition_read(part, s * slotSize + sizeof(FlashHeader), buf.data(), buf.size()) != ESP_OK) continue;
    if (checksum(buf.data(), buf.size()) != h[s].crc) continue;
    if (!deserialize(buf.data(), buf.size())) continue;
    curSlot = s;
    curSeq = h[s].seq;
    return true;
  }
  return false;
}

bool save() {
  if (!part) return false;
  std::vector<uint8_t> b = serialize();
  if (b.size() + sizeof(FlashHeader) > slotSize) return false;
  int target = (curSlot < 0) ? 0 : 1 - curSlot;
  FlashHeader h;
  h.magic = MAGIC;
  h.seq = curSeq + 1;
  h.len = b.size();
  h.crc = checksum(b.data(), b.size());
  uint32_t off = target * slotSize;
  uint32_t eraseLen = (sizeof(FlashHeader) + b.size() + 4095) & ~4095u;
  if (esp_partition_erase_range(part, off, eraseLen) != ESP_OK) return false;
  if (esp_partition_write(part, off + sizeof(FlashHeader), b.data(), b.size()) != ESP_OK) return false;
  if (esp_partition_write(part, off, &h, sizeof(h)) != ESP_OK) return false;  // Header zuletzt
  curSlot = target;
  curSeq = h.seq;
  return true;
}

void clampSelection() {
  if (tvs.empty()) {
    selTv = 0;
    selMacro = 0;
    return;
  }
  if (selTv >= tvs.size()) selTv = tvs.size() - 1;
  size_t nm = tvs[selTv].macros.size();
  if (selMacro >= nm) selMacro = nm ? nm - 1 : 0;
}

bool begin() {
  // Groesste Dateisystem-Partition nehmen (spiffs, fat oder littlefs - je nach Board)
  part = nullptr;
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, nullptr);
  while (it) {
    const esp_partition_t *p = esp_partition_get(it);
    int sub = (int)p->subtype;
    if (sub >= 0x81 && sub <= 0x83 && (!part || p->size > part->size)) part = p;  // 0x81 fat, 0x82 spiffs, 0x83 littlefs
    it = esp_partition_next(it);  // gibt den Iterator am Ende selbst frei
  }
  if (part) {
    slotSize = (part->size / 2) & ~0xFFFu;
    if (slotSize > 262144) slotSize = 262144;
    if (slotSize < 16384) part = nullptr;
  }
  if (!part || !load()) {
    tvs.clear();
    tvs.push_back({"TV 1", {}});
    selTv = selMacro = 0;
    wifiOn = false;
    if (part) save();
  }
  clampSelection();
  return part != nullptr;
}

}  // namespace store

// ===========================================================================
// Bedienung: Display, Tasten, Aufnahme, Wiedergabe
// ===========================================================================
namespace app {

M5Canvas canvas(&M5.Display);

Screen screen = Screen::Tvs;
size_t cursor = 0;

// Optionen-Menue
bool optForMacro = false;
size_t optTv = 0;
size_t optMacro = 0;

// Aufnahme
bool recording = false;
size_t recTv = 0;
int recMacro = -1;  // -1 = neues Makro
String recName;
std::vector<Step> recSteps;
size_t recBytes = 0;
int64_t recPrevStartUs = 0;
uint32_t recStartMs = 0;
uint32_t recLastSignalMs = 0;

// Wiedergabe
bool playing = false;
size_t playTv = 0;
size_t playMacro = 0;
size_t playIdx = 0;
uint32_t playT0 = 0;

String flashMsg;
uint32_t flashUntil = 0;
uint32_t lastInput = 0;
uint32_t lastDraw = 0;
bool displayOff = false;
bool dimmed = false;
bool waitRelease = false;  // nach dem Aufwachen: weckende Taste ignorieren, bis losgelassen
Screen menuBack = Screen::Tvs;  // von hier wurde das Menue geoeffnet
size_t menuBackCursor = 0;

// Update von GitHub
bool updating = false;
int updPct = -1;
String updMsg;
bool selDirty = false;
bool irOk = true;
bool storeOk = true;

constexpr uint16_t COL_ACC = 0x2C7F;
constexpr uint16_t COL_REC = 0xE8E4;
constexpr uint16_t COL_MUT = 0x9CF3;
constexpr uint16_t COL_OK = 0x2E8B;
constexpr uint16_t COL_WARN = 0xFC60;
constexpr uint16_t COL_SEL = 0x18E3;

void flash(const String &m, uint32_t ms = 2500) {
  flashMsg = m;
  flashUntil = millis() + ms;
}

void wake() {
  lastInput = millis();
  if (displayOff) {
    M5.Display.wakeup();
    displayOff = false;
    lastDraw = 0;  // sofort neu zeichnen
  }
  dimmed = false;
  M5.Display.setBrightness(BRIGHT_ON);
}

// Umlaute fuer die Display-Schrift umschreiben
String ascii(const String &s) {
  String o;
  o.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = s[i];
    if (c < 0x80) {
      o += (char)c;
      continue;
    }
    if (c == 0xC3 && i + 1 < s.length()) {
      uint8_t d = s[++i];
      switch (d) {
        case 0xA4: o += "ae"; break;
        case 0xB6: o += "oe"; break;
        case 0xBC: o += "ue"; break;
        case 0x84: o += "Ae"; break;
        case 0x96: o += "Oe"; break;
        case 0x9C: o += "Ue"; break;
        case 0x9F: o += "ss"; break;
        default: o += '?';
      }
      continue;
    }
    if ((c & 0xC0) == 0x80) continue;  // Folgebyte
    o += '?';
  }
  return o;
}

String fit(String s, int w) {
  s = ascii(s);
  if (canvas.textWidth(s) <= w) return s;
  while (s.length() > 1 && canvas.textWidth(s + "..") > w) s.remove(s.length() - 1);
  return s + "..";
}

bool busy() { return recording || playing; }

void doUpdate();

// ---------------------------------------------------------------- Aufnahme
bool startRecord(size_t tv, int macro, const String &name, String &err) {
  if (busy()) {
    err = "Es laeuft bereits eine Aufnahme/Wiedergabe";
    return false;
  }
  if (!irOk) {
    err = "IR-Empfaenger nicht bereit";
    return false;
  }
  if (tv >= store::tvs.size()) {
    err = "Unbekanntes TV-Modell";
    return false;
  }
  auto &t = store::tvs[tv];
  if (macro >= (int)t.macros.size()) {
    err = "Unbekanntes Makro";
    return false;
  }
  if (macro < 0 && t.macros.size() >= MAX_MACROS) {
    err = "Maximal " + String(MAX_MACROS) + " Makros pro TV-Modell";
    return false;
  }
  recording = true;
  recTv = tv;
  recMacro = macro;
  recName = name.length() ? name : String("Makro ") + String(t.macros.size() + 1);
  recSteps.clear();
  recBytes = 0;
  recStartMs = millis();
  recLastSignalMs = 0;
  ir::startReceive();
  screen = Screen::Recording;
  wake();
  return true;
}

void finishRecord(bool keep) {
  if (!recording) return;
  ir::stopReceive();
  recording = false;
  auto &t = store::tvs[recTv];
  if (keep && recSteps.empty()) {
    flash("Kein Signal empfangen");
    keep = false;
  }
  if (keep) {
    if (recMacro >= 0) {
      t.macros[recMacro].steps = std::move(recSteps);
      store::selMacro = recMacro;
    } else {
      Macro m;
      m.name = recName;
      m.steps = std::move(recSteps);
      t.macros.push_back(std::move(m));
      store::selMacro = t.macros.size() - 1;
    }
    store::selTv = recTv;
    flash(store::save() ? "Gespeichert" : "Speichern fehlgeschlagen!");
  } else if (millis() >= flashUntil) {
    flash("Aufnahme verworfen");
  }
  recSteps.clear();
  recSteps.shrink_to_fit();
  store::selTv = recTv;
  screen = Screen::Macros;
  cursor = store::selMacro;
}

void pollRecord() {
  RxFrame f;
  if (!ir::poll(f)) return;
  int64_t startUs = f.startUs;
  uint32_t at = 0;
  if (!recSteps.empty()) {
    int64_t gap = (startUs - recPrevStartUs) / 1000;
    if (gap < 0) gap = 0;
    if (gap > MAX_GAP_MS) gap = MAX_GAP_MS;
    at = recSteps.back().at + (uint32_t)gap;
  }
  recPrevStartUs = startUs;
  Step s;
  s.at = at;
  s.raw = std::move(f.raw);
  recBytes += store::stepBytes(s);
  recSteps.push_back(std::move(s));
  recLastSignalMs = millis();
  wake();

  size_t used = store::totalBytes();
  if (recMacro >= 0) {
    for (auto &st : store::tvs[recTv].macros[recMacro].steps) used -= store::stepBytes(st);
  }
  if (recSteps.size() >= MAX_STEPS || used + recBytes > store::capacity()) {
    finishRecord(true);
    flash("Speicher voll - Aufnahme beendet", 4000);
  }
}

// --------------------------------------------------------------- Wiedergabe
bool startPlay(size_t tv, size_t macro, String &err) {
  if (busy()) {
    err = "Es laeuft bereits eine Aufnahme/Wiedergabe";
    return false;
  }
  if (tv >= store::tvs.size() || macro >= store::tvs[tv].macros.size()) {
    err = "Unbekanntes Makro";
    return false;
  }
  if (store::tvs[tv].macros[macro].steps.empty()) {
    err = "Makro ist leer";
    return false;
  }
  if (store::selTv != tv || store::selMacro != macro) selDirty = true;
  store::selTv = tv;
  store::selMacro = macro;
  playing = true;
  playTv = tv;
  playMacro = macro;
  playIdx = 0;
  playT0 = millis();
  screen = Screen::Playing;
  wake();
  return true;
}

void stopPlay(const char *msg) {
  if (!playing) return;
  playing = false;
  screen = Screen::Macros;
  cursor = playMacro;
  flash(msg);
}

void loopPlay() {
  const auto &steps = store::tvs[playTv].macros[playMacro].steps;
  if (playIdx >= steps.size()) {
    stopPlay("Fertig");
    return;
  }
  if (millis() - playT0 >= steps[playIdx].at) {
    ir::send(steps[playIdx].raw);
    playIdx++;
  }
}

// ------------------------------------------------------------ Strom sparen
// IMU schlafen legen - nur wenn die M5Unified-Version das kann (sonst passiert nichts)
template <class T>
auto imuSleep(T &i, int) -> decltype(i.sleep(), void()) {
  i.sleep();
}
template <class T>
void imuSleep(T &, long) {}

// Tiefschlaf: Prozessor, Display und Funk aus. Taste A oder B startet ihn neu,
// er steht dann wieder beim zuletzt benutzten Makro (alles ist im Flash gespeichert).
void deepSleep() {
  if (selDirty) {
    selDirty = false;
    store::save();
  }
  web::stop();
  ir::stopReceive();
  M5.Display.setBrightness(0);
  M5.Display.sleep();
  imuSleep(M5.Imu, 0);
  M5.Power.setExtOutput(false);  // IR-LED-Versorgung aus
  // Ruhepegel der Tasten jetzt lesen (keine gedrueckt) und auf die Gegenrichtung wecken
  int idleA = digitalRead(PIN_BTN_A), idleB = digitalRead(PIN_BTN_B);
  uint64_t mask = (1ULL << PIN_BTN_A) | (idleA == idleB ? (1ULL << PIN_BTN_B) : 0);
  esp_sleep_enable_ext1_wakeup(mask, idleA ? ESP_EXT1_WAKEUP_ANY_LOW : ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

// Power-Taste: ganz aus. Einschalten wieder mit der Power-Taste.
// Haengt er am USB-Kabel, laesst sich der Stick nicht ganz abschalten -> Tiefschlaf.
void powerOff() {
  if (recording) finishRecord(false);
  if (playing) stopPlay("Aus");
  if (selDirty) {
    selDirty = false;
    store::save();
  }
  web::stop();
  M5.Display.setBrightness(0);
  M5.Display.sleep();
  M5.Power.powerOff();
  delay(500);
  deepSleep();
}

// ------------------------------------------------------------------ Listen
size_t itemCount() {
  switch (screen) {
    case Screen::Tvs: return store::tvs.size() + 1;
    case Screen::Macros: return store::tvs[store::selTv].macros.size() + 2;
    case Screen::Options: return optForMacro ? 4 : 2;
    case Screen::Confirm: return 2;
    case Screen::Menu: return 4;
    default: return 0;
  }
}

String itemText(size_t i) {
  switch (screen) {
    case Screen::Tvs: {
      size_t n = store::tvs.size();
      if (i < n) return store::tvs[i].name;
      return "+ Neues TV";
    }
    case Screen::Macros: {
      const auto &ms = store::tvs[store::selTv].macros;
      if (i < ms.size()) return ms[i].name;
      if (i == ms.size()) return "* Aufnehmen";
      return "< Zurueck";
    }
    case Screen::Options:
      if (optForMacro) {
        const char *o[] = {"Abspielen", "Neu aufnehmen", "Loeschen", "< Zurueck"};
        return o[i];
      } else {
        const char *o[] = {"Loeschen", "< Zurueck"};
        return o[i];
      }
    case Screen::Confirm: {
      const char *o[] = {"Nein", "Ja, loeschen"};
      return o[i];
    }
    case Screen::Menu: {
      if (i == 0) return web::isRunning() ? "WLAN: an" : "WLAN/Browser";
      const char *o[] = {"", "Update", "Ausschalten", "< Zurueck"};
      return o[i];
    }
    default: return "";
  }
}

String title() {
  switch (screen) {
    case Screen::Tvs: return "TV-Modelle";
    case Screen::Macros: return store::tvs[store::selTv].name;
    case Screen::Options:
    case Screen::Confirm:
      return optForMacro ? store::tvs[optTv].macros[optMacro].name : store::tvs[optTv].name;
    case Screen::Menu: return "Menue";
    default: return "";
  }
}

// Zwei Hinweiszeilen unten
void hints(const char *&l1, const char *&l2) {
  switch (screen) {
    case Screen::Tvs: l1 = "A Oeffnen  B Weiter"; l2 = "B lang: Menue"; break;
    case Screen::Macros: l1 = "A Start  B Weiter"; l2 = "A lang: Optionen"; break;
    case Screen::Confirm: l1 = "Wirklich loeschen?"; l2 = "A Auswahl  B Weiter"; break;
    case Screen::Menu: l1 = "A Auswahl  B Weiter"; l2 = "B lang: Zurueck"; break;
    default: l1 = "A Auswahl  B Weiter"; l2 = ""; break;
  }
}

void backToList() {
  screen = optForMacro ? Screen::Macros : Screen::Tvs;
  cursor = optForMacro ? optMacro : optTv;
}

void openMenu() {
  menuBack = screen;
  menuBackCursor = cursor;
  screen = Screen::Menu;
  cursor = 0;
}

void closeMenu() {
  screen = menuBack;
  cursor = menuBackCursor;
  if (screen == Screen::Macros && store::tvs.empty()) screen = Screen::Tvs;
  if (screen != Screen::Tvs && screen != Screen::Macros) screen = Screen::Tvs;
  if (cursor >= itemCount()) cursor = 0;
}

// Nach Aenderungen ueber das Web: Anzeige in einen gueltigen Zustand bringen
void afterExternalChange() {
  store::clampSelection();
  if (screen == Screen::Options || screen == Screen::Confirm) screen = Screen::Tvs;
  if (screen == Screen::Macros && store::tvs.empty()) screen = Screen::Tvs;
  if (menuBack == Screen::Macros && store::tvs.empty()) menuBack = Screen::Tvs;
  if (cursor >= itemCount()) cursor = 0;
}

void activate(size_t i) {
  String err;
  switch (screen) {
    case Screen::Tvs: {
      size_t n = store::tvs.size();
      if (i < n) {
        if (store::selTv != i) {
          store::selTv = i;
          store::selMacro = 0;
          selDirty = true;
        }
        screen = Screen::Macros;
        cursor = store::selMacro;
      } else {
        if (n >= MAX_TVS) {
          flash("Maximal " + String(MAX_TVS) + " TV-Modelle");
          return;
        }
        store::tvs.push_back({String("TV ") + String(n + 1), {}});
        store::save();
        cursor = n;
        flash("Angelegt. Name im Browser aenderbar", 3500);
      }
      break;
    }
    case Screen::Macros: {
      size_t n = store::tvs[store::selTv].macros.size();
      if (i < n) {
        if (!startPlay(store::selTv, i, err)) flash(err);
      } else if (i == n) {
        if (!startRecord(store::selTv, -1, "", err)) flash(err);
      } else {
        screen = Screen::Tvs;
        cursor = store::selTv;
      }
      break;
    }
    case Screen::Options:
      if (optForMacro) {
        if (i == 0) {
          if (!startPlay(optTv, optMacro, err)) flash(err);
        } else if (i == 1) {
          if (!startRecord(optTv, optMacro, store::tvs[optTv].macros[optMacro].name, err)) flash(err);
        } else if (i == 2) {
          screen = Screen::Confirm;
          cursor = 0;
        } else {
          backToList();
        }
      } else {
        if (i == 0) {
          screen = Screen::Confirm;
          cursor = 0;
        } else {
          backToList();
        }
      }
      break;
    case Screen::Confirm:
      if (i == 1) {
        if (optForMacro) {
          auto &ms = store::tvs[optTv].macros;
          ms.erase(ms.begin() + optMacro);
          if (optMacro > 0) optMacro--;
        } else {
          store::tvs.erase(store::tvs.begin() + optTv);
          if (optTv > 0) optTv--;
        }
        store::clampSelection();
        store::save();
        flash("Geloescht");
        if (!optForMacro || store::tvs.empty()) {
          screen = Screen::Tvs;
          cursor = optTv;
          return;
        }
      }
      backToList();
      break;
    case Screen::Menu:
      if (i == 0) {
        web::start();
        screen = Screen::Wifi;
      } else if (i == 1) {
        doUpdate();
      } else if (i == 2) {
        powerOff();
      } else {
        closeMenu();
      }
      break;
    default: break;
  }
}

void hold(size_t i) {
  if (screen == Screen::Tvs && i < store::tvs.size()) {
    optForMacro = false;
    optTv = i;
    screen = Screen::Options;
    cursor = 0;
  } else if (screen == Screen::Macros && i < store::tvs[store::selTv].macros.size()) {
    optForMacro = true;
    optTv = store::selTv;
    optMacro = i;
    screen = Screen::Options;
    cursor = 0;
  }
}

// ------------------------------------------------------------------ Zeichnen
// Hochkant: 135 x 240 Pixel
constexpr int ROW_Y = 26;  // erste Listenzeile
constexpr int ROW_H = 28;
constexpr int ROWS = 6;    // sichtbare Zeilen
constexpr int FOOT_Y = 204;

void header(const String &t, uint16_t col) {
  int w = canvas.width();
  canvas.fillRect(0, 0, w, 22, col);
  canvas.setTextColor(TFT_WHITE);
  canvas.setFont(&fonts::Font2);
  int bat = M5.Power.getBatteryLevel();
  int right = w - 3;
  if (bat >= 0) {
    canvas.setTextDatum(middle_right);
    String b = String(bat) + "%";
    canvas.drawString(b, right, 11);
    right -= canvas.textWidth(b) + 4;
  }
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextDatum(middle_left);
  canvas.drawString(fit(t, right - 4), 3, 11);
}

// Text auf mehrere Zeilen umbrechen (aktuelle Schrift), liefert die naechste freie y-Position
int wrapText(const String &text, int x, int y, int w, int lineH, int maxLines, bool center) {
  String t = ascii(text) + " ";
  String line, word;
  int lines = 0;
  canvas.setTextDatum(center ? top_center : top_left);
  int cx = center ? x + w / 2 : x;
  for (size_t i = 0; i < t.length() && lines < maxLines; i++) {
    char c = t[i];
    if (c != ' ') {
      word += c;
      continue;
    }
    String test = line.length() ? line + " " + word : word;
    if (canvas.textWidth(test) > w && line.length()) {
      canvas.drawString(line, cx, y);
      y += lineH;
      lines++;
      line = word;
    } else {
      line = test;
    }
    word = "";
  }
  if (line.length() && lines < maxLines) {
    canvas.drawString(fit(line, w), cx, y);
    y += lineH;
  }
  return y;
}

void footer(const char *l1, const char *l2, uint16_t col) {
  int w = canvas.width();
  canvas.setFont(&fonts::Font2);
  if (millis() < flashUntil) {
    canvas.setTextColor(COL_WARN);
    wrapText(flashMsg, 2, FOOT_Y, w - 4, 17, 2, true);
    return;
  }
  canvas.setTextColor(col);
  canvas.setTextDatum(top_center);
  canvas.drawString(fit(l1, w - 4), w / 2, FOOT_Y);
  canvas.drawString(fit(l2, w - 4), w / 2, FOOT_Y + 17);
}

void drawList() {
  int w = canvas.width();
  header(title(), screen == Screen::Confirm ? COL_REC : COL_ACC);
  size_t n = itemCount();
  if (cursor >= n) cursor = 0;
  int rows = screen == Screen::Menu ? 4 : ROWS;
  size_t first = cursor >= (size_t)rows ? cursor - rows + 1 : 0;
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextDatum(middle_left);
  for (int r = 0; r < rows && first + r < n; r++) {
    size_t i = first + r;
    int y = ROW_Y + r * ROW_H;
    if (i == cursor) {
      canvas.fillRoundRect(2, y + 1, w - 6, ROW_H - 3, 5, COL_SEL);
      canvas.setTextColor(TFT_WHITE);
    } else {
      canvas.setTextColor(COL_MUT);
    }
    canvas.drawString(fit(itemText(i), w - 16), 7, y + ROW_H / 2);
  }
  if (n > (size_t)rows) {  // Scrollbalken
    int h = rows * ROW_H;
    int bh = h * rows / n;
    int by = ROW_Y + (h - bh) * first / (n - rows);
    canvas.fillRect(w - 3, by, 2, bh, COL_MUT);
  }
  if (screen == Screen::Menu) {  // Copyright + Version
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(top_center);
    canvas.setTextColor(COL_MUT);
    canvas.drawString(COPYRIGHT, w / 2, 150);
    canvas.drawString(String("Version ") + FW_NAME, w / 2, 164);
  }
  const char *l1, *l2;
  hints(l1, l2);
  footer(l1, l2, COL_MUT);
}

void drawRecording() {
  int w = canvas.width();
  header("Aufnahme", COL_REC);
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextDatum(top_center);
  canvas.drawString(fit(recName, w - 8), w / 2, 30);
  bool fresh = recLastSignalMs && millis() - recLastSignalMs < 400;
  canvas.setFont(&fonts::FreeSansBold24pt7b);
  canvas.setTextColor(fresh ? COL_OK : TFT_WHITE);
  canvas.drawString(String(recSteps.size()), w / 2, 58);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COL_MUT);
  canvas.drawString(recSteps.size() == 1 ? "Signal" : "Signale", w / 2, 102);
  uint32_t secs = (millis() - recStartMs) / 1000;
  canvas.drawString(String(secs / 60) + ":" + (secs % 60 < 10 ? "0" : "") + String(secs % 60), w / 2, 120);
  canvas.setTextColor(TFT_WHITE);
  if (recSteps.empty()) wrapText("Original-FB auf die Oberseite richten und Tasten druecken", 4, 146, w - 8, 17, 3, true);
  else wrapText("Weitere Tasten druecken. Pausen werden mit aufgenommen.", 4, 146, w - 8, 17, 3, true);
  footer("A: Fertig", "B: Abbrechen", COL_MUT);
}

void drawPlaying() {
  int w = canvas.width();
  const auto &m = store::tvs[playTv].macros[playMacro];
  header("Abspielen", COL_OK);
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextColor(TFT_WHITE);
  int y = wrapText(m.name, 4, 32, w - 8, 20, 2, true);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(COL_MUT);
  canvas.setTextDatum(top_center);
  canvas.drawString(fit(store::tvs[playTv].name, w - 8), w / 2, y + 4);
  canvas.drawString("Signal " + String(playIdx) + " / " + String(m.steps.size()), w / 2, 110);
  int bw = w - 16;
  canvas.drawRect(8, 132, bw, 12, COL_MUT);
  canvas.fillRect(9, 133, (bw - 2) * playIdx / m.steps.size(), 10, COL_OK);
  uint32_t total = m.steps.back().at;
  uint32_t el = millis() - playT0;
  uint32_t left = el < total ? (total - el + 999) / 1000 : 0;
  canvas.drawString("noch " + String(left) + " s", w / 2, 152);
  footer("A / B: Stopp", "", COL_MUT);
}

void drawWifi() {
  int w = canvas.width();
  header("WLAN", COL_ACC);
  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(top_left);
  const char *lab[] = {"WLAN-Name:", "Passwort:", "Im Browser:"};
  String val[] = {web::ssid(), AP_PASS, "192.168.4.1"};
  for (int i = 0; i < 3; i++) {
    canvas.setTextColor(COL_MUT);
    canvas.drawString(lab[i], 6, 30 + i * 42);
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(fit(val[i], w - 12), 6, 48 + i * 42);
  }
  canvas.setTextColor(COL_MUT);
  wrapText("Umbenennen, Sortieren, Update-WLAN", 4, 160, w - 8, 17, 2, true);
  footer("A: Zurueck", "A lang: WLAN aus", COL_MUT);
}

void drawUpdate() {
  int w = canvas.width();
  header("Update", COL_WARN);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_WHITE);
  wrapText(updMsg, 4, 32, w - 8, 18, 3, true);
  canvas.setTextColor(COL_MUT);
  canvas.setTextDatum(top_center);
  canvas.drawString(String("Installiert: ") + FW_NAME, w / 2, 96);
  if (updPct >= 0) {
    canvas.drawString(String(updPct) + " %", w / 2, 122);
    int bw = w - 16;
    canvas.drawRect(8, 142, bw, 12, COL_MUT);
    canvas.fillRect(9, 143, (bw - 2) * updPct / 100, 10, COL_OK);
  }
  canvas.setTextColor(COL_WARN);
  canvas.drawString(updPct >= 0 ? "Nicht ausschalten!" : "B: Abbrechen", w / 2, FOOT_Y + 8);
}

void draw() {
  canvas.fillSprite(TFT_BLACK);
  switch (screen) {
    case Screen::Recording: drawRecording(); break;
    case Screen::Playing: drawPlaying(); break;
    case Screen::Wifi: drawWifi(); break;
    case Screen::Update: drawUpdate(); break;
    default: drawList(); break;
  }
  if (!irOk || !storeOk) {
    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(top_center);
    canvas.setTextColor(COL_REC);
    canvas.drawString(!irOk ? "IR-Fehler" : "Speicherfehler", canvas.width() / 2, 186);
  }
  canvas.pushSprite(0, 0);
}

// ------------------------------------------------------------ Update von GitHub
// Wie beim Delfino-Stick: mit dem gespeicherten WLAN (iPhone-Hotspot) verbinden,
// version.txt vom neuesten GitHub-Release lesen und bei neuerer Build-Nummer
// firmware.bin laden, flashen und neu starten.
void updShow(const String &msg, int pct) {
  updMsg = msg;
  updPct = pct;
  lastInput = millis();
  draw();
}

void doUpdate() {
  Preferences p;
  p.begin("upd", true);
  String ssid = p.getString("s", ""), pass = p.getString("p", "");
  p.end();
  if (!ssid.length()) {
    flash("Erst Update-WLAN im Browser eintragen", 5000);
    return;
  }
  bool hadAp = web::isRunning();
  web::stop();
  setCpuFrequencyMhz(240);  // fuer die verschluesselte Verbindung schneller
  updating = true;
  screen = Screen::Update;
  wake();
  updShow("Verbinde mit " + ascii(ssid), -1);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  bool cancel = false;
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000 && !cancel) {
    delay(100);
    M5.update();
    cancel = M5.BtnB.wasClicked();
  }

  String result;
  if (cancel) {
    result = "Abgebrochen";
  } else if (WiFi.status() != WL_CONNECTED) {
    result = "WLAN " + ascii(ssid) + " nicht gefunden";
  } else {
    updShow("Suche Update ...", -1);
    WiFiClientSecure cli;
    cli.setInsecure();
    int build = -1;
    String name;
    {
      HTTPClient http;
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
      http.setTimeout(15000);
      if (http.begin(cli, String(UPDATE_URL) + "version.txt")) {
        if (http.GET() == 200) {
          String s = http.getString();  // Zeile 1: Build-Nummer, Zeile 2: Version
          build = s.toInt();
          int nl = s.indexOf('\n');
          if (nl >= 0) name = s.substring(nl + 1);
          name.trim();
        }
        http.end();
      }
    }
    if (build <= 0) {
      result = "Kein Update gefunden";
    } else if (build <= FW_BUILD) {
      result = String("Schon aktuell (") + FW_NAME + ")";
    } else {
      updShow("Lade Version " + name, 0);
      if (selDirty) {
        selDirty = false;
        store::save();
      }
      httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
      httpUpdate.rebootOnUpdate(false);
      httpUpdate.onProgress([](int cur, int total) {
        int pct = total > 0 ? (int)((int64_t)cur * 100 / total) : 0;
        if (pct != updPct) updShow(updMsg, pct);
      });
      t_httpUpdate_return r = httpUpdate.update(cli, String(UPDATE_URL) + "firmware.bin");
      if (r == HTTP_UPDATE_OK) {
        updShow("Fertig - Neustart ...", 100);
        delay(1500);
        ESP.restart();
      }
      result = "Update fehlgeschlagen: " + httpUpdate.getLastErrorString();
    }
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  setCpuFrequencyMhz(CPU_MHZ);
  updating = false;
  updPct = -1;
  if (hadAp) web::start();
  screen = Screen::Menu;
  cursor = 1;
  flash(result, 6000);
  wake();
}

// ------------------------------------------------------------------ Ablauf
uint32_t bootMs = 0;

void begin() {
  esp_ota_mark_app_valid_cancel_rollback();  // diese Version laeuft: behalten
  setCpuFrequencyMhz(CPU_MHZ);
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);  // Funk aus, bis er gebraucht wird
  bootMs = millis();
  // Von Taste A/B aus dem Tiefschlaf geweckt: diesen Druck nicht als Bedienung werten
  waitRelease = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1;
  canvas.setColorDepth(16);
  canvas.createSprite(M5.Display.width(), M5.Display.height());
  storeOk = store::begin();
  store::wifiOn = false;  // Stick-WLAN startet nie von selbst (Strom)
  irOk = ir::begin();
  screen = Screen::Macros;  // direkt beim zuletzt benutzten TV-Modell starten
  if (store::tvs.empty()) screen = Screen::Tvs;
  cursor = store::selMacro;
  lastInput = millis();
  M5.Display.setBrightness(BRIGHT_ON);
  draw();
}

void loop() {
  M5.update();
  web::loop();

  bool aClick = M5.BtnA.wasClicked(), aHold = M5.BtnA.wasHold();
  bool bClick = M5.BtnB.wasClicked(), bHold = M5.BtnB.wasHold();
  bool pwr = M5.BtnPWR.wasClicked();

  if (waitRelease) {  // weckende Taste nach dem Tiefschlaf ignorieren
    if (!M5.BtnA.isPressed() && !M5.BtnB.isPressed()) waitRelease = false;
    aClick = aHold = bClick = bHold = false;
  }
  if (millis() - bootMs < 3000) pwr = false;  // Einschalt-Druck der Power-Taste nicht als "aus" werten

  if (pwr) powerOff();

  bool any = aClick || aHold || bClick || bHold;
  if (any && displayOff) {
    wake();  // Display war aus: erster Druck weckt nur
  } else if (any) {
    wake();
    switch (screen) {
      case Screen::Recording:
        if (aClick) finishRecord(true);
        if (bClick) finishRecord(false);
        break;
      case Screen::Playing:
        if (aClick || bClick) stopPlay("Gestoppt");
        break;
      case Screen::Wifi:
        if (aClick) {
          screen = Screen::Menu;
          cursor = 0;
        }
        if (aHold) {
          web::stop();
          screen = Screen::Menu;
          cursor = 0;
          flash("WLAN aus");
        }
        break;
      default: {
        size_t n = itemCount();
        if (bClick && n) cursor = (cursor + 1) % n;
        if (bHold) {
          if (screen == Screen::Menu) closeMenu();
          else if (screen == Screen::Tvs || screen == Screen::Macros) openMenu();
        }
        if (aClick) activate(cursor);
        if (aHold) hold(cursor);
        break;
      }
    }
  }

  if (recording) pollRecord();
  if (playing) loopPlay();

  // Zuletzt benutztes Makro merken (verzoegert, schont den Flash)
  if (selDirty && !busy() && millis() - lastInput > 10000) {
    selDirty = false;
    store::save();
  }

  // Strom sparen: 10 s dunkler, 30 s Tiefschlaf (mit Stick-WLAN an: nur Display aus)
  if (busy()) lastInput = millis();
  uint32_t idle = millis() - lastInput;
  if (!displayOff && !dimmed && idle > DISPLAY_DIM_MS) {
    M5.Display.setBrightness(BRIGHT_DIM);
    dimmed = true;
  }
  if (idle > DISPLAY_OFF_MS) {
    if (!web::isRunning() && !M5.BtnA.isPressed() && !M5.BtnB.isPressed()) deepSleep();
    if (!displayOff) {
      M5.Display.setBrightness(0);
      M5.Display.sleep();
      displayOff = true;
    }
  }

  if (!displayOff && millis() - lastDraw > (recording || playing ? 50u : 150u)) {
    lastDraw = millis();
    draw();
  }
  delay(recording ? 1 : (displayOff ? 30 : 10));  // Pausen: der Prozessor ruht dazwischen
}

}  // namespace app

// ===========================================================================
// Web-Oberflaeche (optional): Stick macht ein eigenes WLAN auf
// ===========================================================================
static const char INDEX_HTML[] PROGMEM =
    "<!doctype html>\n"
    "<html lang=\"de\"><head><meta charset=\"utf-8\">\n"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
    "<title>TV-Makros</title>\n"
    "<style>\n"
    ":root{--bg:#f3f4f6;--card:#fff;--fg:#111;--mut:#666;--line:#eee;--btn:#e5e7eb;--acc:#1f6feb;--warn:#fff7d6}\n"
    "@media(prefers-color-scheme:dark){:root{--bg:#111418;--card:#1c1f24;--fg:#e6e8eb;--mut:#9aa3ad;--line:#2b313a;--btn:#2f343c;--acc:#4c8dff;--warn:#3a3010}}\n"
    "body{font-family:system-ui,sans-serif;margin:0;background:var(--bg);color:var(--fg)}\n"
    "header{background:var(--acc);color:#fff;padding:12px 16px;font-weight:600}\n"
    "main{max-width:640px;margin:auto;padding:12px}\n"
    ".card{background:var(--card);border-radius:10px;padding:12px;margin-bottom:12px;box-shadow:0 1px 3px #0003}\n"
    ".row{display:flex;align-items:center;gap:6px;padding:7px 0;border-top:1px solid var(--line)}\n"
    ".grow{flex:1;min-width:0;overflow-wrap:anywhere}\n"
    ".mut{color:var(--mut);font-size:.85em}\n"
    "button{border:0;border-radius:6px;padding:8px 10px;background:var(--btn);color:var(--fg);font-size:.95em;cursor:pointer}\n"
    "button:disabled{opacity:.4}\n"
    ".p{background:var(--acc);color:#fff}.d{color:#d33}\n"
    "h2{font-size:1.1em;margin:0 0 4px;display:flex;align-items:center;gap:6px}\n"
    "#st{display:none;background:var(--warn)}\n"
    "</style></head><body>\n"
    "<header>TV-Makro-Fernbedienung</header>\n"
    "<main>\n"
    "<div class=\"card\" id=\"st\"></div>\n"
    "<div id=\"tvs\">Lade...</div>\n"
    "<button class=\"p\" onclick=\"addTv()\">+ TV-Modell</button>\n"
    "<div class=\"card\"><b>Updates</b>\n"
    "<p class=\"mut\">Installiert: Version <span id=\"ver\">?</span>. Der Stick holt sich Updates selbst von GitHub. Dafür hier einmal dein WLAN eintragen, z.B. den iPhone-Hotspot (am iPhone <i>Maximale Kompatibilität</i> einschalten). Danach am Stick <b>Update</b> wählen.</p>\n"
    "<p class=\"mut\" id=\"updnow\"></p>\n"
    "<input id=\"us\" placeholder=\"WLAN-Name (z.B. iPhone von ...)\" autocapitalize=\"none\" autocorrect=\"off\" style=\"width:100%;box-sizing:border-box;padding:9px;margin:4px 0;border-radius:6px;border:1px solid var(--line);background:var(--bg);color:var(--fg)\">\n"
    "<input id=\"up\" type=\"password\" placeholder=\"Passwort\" style=\"width:100%;box-sizing:border-box;padding:9px;margin:4px 0 8px;border-radius:6px;border:1px solid var(--line);background:var(--bg);color:var(--fg)\">\n"
    "<button class=\"p\" onclick=\"saveWifi()\">WLAN speichern</button></div>\n"
    "<p class=\"mut\">Aufnehmen: Original-Fernbedienung aus ca. 20-30 cm auf die Oberseite des StickS3 richten und die Tasten in Ruhe nacheinander drücken. Die Pausen werden mit aufgenommen und beim Abspielen genauso wiederholt.</p>\n"
    "</main>\n"
    "<script>\n"
    "let D=null,last='';\n"
    "const $=s=>document.querySelector(s);\n"
    "const esc=s=>String(s).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]));\n"
    "async function api(p,a){let ok=false;try{const r=await fetch('/api/'+p+'?'+new URLSearchParams(a||{}),{method:'POST'});ok=r.ok;if(!r.ok)alert(await r.text())}catch(e){alert('Keine Verbindung zum Stick')}last='';await load();return ok}\n"
    "const sec=ms=>(ms/1000).toFixed(1)+' s';\n"
    "function render(){\n"
    " const st=$('#st'),busy=D.rec.on||D.play.on;\n"
    " if(D.rec.on){st.style.display='block';st.innerHTML='<b>Aufnahme: '+esc(D.rec.name)+'</b><br>'+D.rec.n+' Signale empfangen<br><br><button class=\"p\" onclick=\"api(\\'recdone\\')\">Fertig &amp; speichern</button> <button onclick=\"api(\\'recabort\\')\">Abbrechen</button>'}\n"
    " else if(D.play.on){st.style.display='block';st.innerHTML='<b>Spielt ab: '+esc(D.play.name)+'</b><br>Signal '+D.play.step+' / '+D.play.total+'<br><br><button class=\"d\" onclick=\"api(\\'stop\\')\">Stopp</button>'}\n"
    " else st.style.display='none';\n"
    " $('#ver').textContent=D.ver;$('#updnow').textContent=D.updssid?'Gespeichertes WLAN: '+D.updssid:'Noch kein WLAN für Updates gespeichert.';if(!$('#us').value&&D.updssid&&document.activeElement!==$('#us'))$('#us').value=D.updssid;\n"
    " const dis=busy?' disabled':'';\n"
    " $('#tvs').innerHTML=D.tvs.length?D.tvs.map((t,i)=>'<div class=\"card\"><h2><span class=\"grow\">'+esc(t.n)+'</span><button onclick=\"ren('+i+',-1)\"'+dis+'>Umbenennen</button><button class=\"d\" onclick=\"del('+i+',-1)\"'+dis+'>Löschen</button></h2>'+\n"
    "  t.m.map((m,j)=>'<div class=\"row\"><button class=\"p\" onclick=\"api(\\'play\\',{tv:'+i+',m:'+j+'})\"'+dis+'>&#9654;</button><div class=\"grow\">'+esc(m.n)+'<br><span class=\"mut\">'+m.s+' Signale &middot; '+sec(m.d)+'</span></div>'+\n"
    "   (j?'<button title=\"nach oben\" onclick=\"api(\\'move\\',{tv:'+i+',m:'+j+'})\"'+dis+'>&uarr;</button>':'')+\n"
    "   '<button title=\"umbenennen\" onclick=\"ren('+i+','+j+')\"'+dis+'>&#9998;</button><button title=\"neu aufnehmen\" onclick=\"rec('+i+','+j+')\"'+dis+'>&#9679;</button><button class=\"d\" title=\"löschen\" onclick=\"del('+i+','+j+')\"'+dis+'>&#10005;</button></div>').join('')+\n"
    "  '<div class=\"row\"><button onclick=\"rec('+i+',-1)\"'+dis+'>&#9679; Neues Makro aufnehmen</button></div></div>').join(''):'<p class=\"mut\">Noch kein TV-Modell angelegt.</p>';\n"
    "}\n"
    "function saveWifi(){const s=$('#us').value.trim(),p=$('#up').value;if(!s)return alert('Bitte den WLAN-Namen eintragen.');\n"
    " const a={s:s,p:p};if(!p&&D.updssid===s&&confirm('Passwort leer lassen = bisheriges Passwort behalten?'))a.keep=1;\n"
    " api('updwifi',a).then(ok=>{if(!ok)return;$('#up').value='';alert('Gespeichert. Jetzt am Stick \"Update\" wählen, wenn der Hotspot an ist.')})}\n"
    "function ren(i,j){const o=j<0?D.tvs[i].n:D.tvs[i].m[j].n,n=prompt('Neuer Name:',o);if(n&&n.trim())api('rename',{tv:i,m:j,name:n.trim()})}\n"
    "function del(i,j){const n=j<0?'TV-Modell \"'+D.tvs[i].n+'\" mit allen Makros':'Makro \"'+D.tvs[i].m[j].n+'\"';if(confirm(n+' wirklich löschen?'))api('del',{tv:i,m:j})}\n"
    "function rec(i,j){let n;if(j<0){n=prompt('Name für das neue Makro:','Makro '+(D.tvs[i].m.length+1));if(!n||!n.trim())return}else{if(!confirm('Makro \"'+D.tvs[i].m[j].n+'\" neu aufnehmen (die alte Aufnahme wird ersetzt)?'))return;n=D.tvs[i].m[j].n}api('rec',{tv:i,m:j,name:n.trim()})}\n"
    "function addTv(){const n=prompt('Name des TV-Modells (z.B. Samsung UE55):');if(n&&n.trim())api('addtv',{name:n.trim()})}\n"
    "async function load(){try{const r=await fetch('/api/data'),t=await r.text();if(t!==last){last=t;D=JSON.parse(t);render()}}catch(e){}}\n"
    "load();setInterval(load,1000);\n"
    "</script></body></html>";

namespace web {

WebServer server(80);
bool running = false;
bool routesDone = false;
uint32_t lastClientMs = 0;

String ssid() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  char s[32];
  snprintf(s, sizeof(s), "%s%02X%02X", AP_PREFIX, mac[4], mac[5]);
  return s;
}

void jsonStr(String &o, const String &s) {
  o += '"';
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') {
      o += '\\';
      o += c;
    } else if ((uint8_t)c < 0x20) {
      o += ' ';
    } else {
      o += c;
    }
  }
  o += '"';
}

void ok() { server.send(200, "text/plain", "OK"); }
void fail(const String &m) { server.send(400, "text/plain; charset=utf-8", m); }

long argInt(const char *name, long def) { return server.hasArg(name) ? server.arg(name).toInt() : def; }

String argName() {
  String n = server.arg("name");
  n.trim();
  if (n.length() > 40) n = n.substring(0, 40);
  return n;
}

// WLAN fuer Updates (z.B. iPhone-Hotspot) speichern
void handleUpdWifi() {
  String s = server.arg("s");
  s.trim();
  if (!s.length() || s.length() > 32) return fail("WLAN-Name: 1-32 Zeichen");
  String pw = server.arg("p");
  bool keep = server.arg("keep") == "1";
  if (!keep && pw.length() && (pw.length() < 8 || pw.length() > 63)) return fail("Passwort: 8-63 Zeichen (oder leer bei offenem WLAN)");
  Preferences p;
  p.begin("upd", false);
  p.putString("s", s);
  if (!keep) p.putString("p", pw);
  p.end();
  ok();
}

String updSsid() {
  Preferences p;
  p.begin("upd", true);
  String s = p.getString("s", "");
  p.end();
  return s;
}

bool isRunning() { return running; }

bool notBusy() {
  if (app::busy()) {
    fail("Gerade laeuft eine Aufnahme oder Wiedergabe");
    return false;
  }
  return true;
}

bool argTv(size_t &tv) {
  long v = argInt("tv", -1);
  if (v < 0 || v >= (long)store::tvs.size()) {
    fail("Unbekanntes TV-Modell");
    return false;
  }
  tv = v;
  return true;
}

void handleData() {
  String o;
  o.reserve(1024);
  o += "{\"ver\":\"";
  o += FW_NAME;
  o += "\",\"updssid\":";
  jsonStr(o, updSsid());
  o += ",\"rec\":{\"on\":";
  o += app::recording ? "true" : "false";
  o += ",\"n\":" + String(app::recSteps.size()) + ",\"name\":";
  jsonStr(o, app::recName);
  o += "},\"play\":{\"on\":";
  o += app::playing ? "true" : "false";
  o += ",\"step\":" + String(app::playIdx);
  if (app::playing) {
    const auto &m = store::tvs[app::playTv].macros[app::playMacro];
    o += ",\"total\":" + String(m.steps.size()) + ",\"name\":";
    jsonStr(o, m.name);
  }
  o += "},\"tvs\":[";
  for (size_t i = 0; i < store::tvs.size(); i++) {
    const auto &t = store::tvs[i];
    if (i) o += ',';
    o += "{\"n\":";
    jsonStr(o, t.name);
    o += ",\"m\":[";
    for (size_t j = 0; j < t.macros.size(); j++) {
      const auto &m = t.macros[j];
      if (j) o += ',';
      o += "{\"n\":";
      jsonStr(o, m.name);
      o += ",\"s\":" + String(m.steps.size());
      o += ",\"d\":" + String(m.steps.empty() ? 0 : m.steps.back().at);
      o += '}';
    }
    o += "]}";
  }
  o += "]}";
  server.send(200, "application/json", o);
}

void handlePlay() {
  size_t tv;
  if (!argTv(tv)) return;
  String err;
  if (!app::startPlay(tv, argInt("m", -1), err)) return fail(err);
  ok();
}

void handleStop() {
  app::stopPlay("Gestoppt");
  ok();
}

void handleRec() {
  size_t tv;
  if (!argTv(tv)) return;
  String err;
  if (!app::startRecord(tv, argInt("m", -1), argName(), err)) return fail(err);
  ok();
}

void handleRecDone() {
  app::finishRecord(true);
  ok();
}

void handleRecAbort() {
  app::finishRecord(false);
  ok();
}

void handleRename() {
  if (!notBusy()) return;
  size_t tv;
  if (!argTv(tv)) return;
  String name = argName();
  if (!name.length()) return fail("Name fehlt");
  long m = argInt("m", -1);
  if (m < 0) {
    store::tvs[tv].name = name;
  } else if (m < (long)store::tvs[tv].macros.size()) {
    store::tvs[tv].macros[m].name = name;
  } else {
    return fail("Unbekanntes Makro");
  }
  if (!store::save()) return fail("Speichern fehlgeschlagen");
  ok();
}

void handleDelete() {
  if (!notBusy()) return;
  size_t tv;
  if (!argTv(tv)) return;
  long m = argInt("m", -1);
  if (m < 0) {
    store::tvs.erase(store::tvs.begin() + tv);
  } else if (m < (long)store::tvs[tv].macros.size()) {
    auto &ms = store::tvs[tv].macros;
    ms.erase(ms.begin() + m);
  } else {
    return fail("Unbekanntes Makro");
  }
  app::afterExternalChange();
  if (!store::save()) return fail("Speichern fehlgeschlagen");
  ok();
}

void handleAddTv() {
  if (!notBusy()) return;
  String name = argName();
  if (!name.length()) return fail("Name fehlt");
  if (store::tvs.size() >= MAX_TVS) return fail("Maximal " + String(MAX_TVS) + " TV-Modelle");
  store::tvs.push_back({name, {}});
  app::afterExternalChange();
  if (!store::save()) return fail("Speichern fehlgeschlagen");
  ok();
}

// Makro eine Position nach oben schieben
void handleMove() {
  if (!notBusy()) return;
  size_t tv;
  if (!argTv(tv)) return;
  long m = argInt("m", -1);
  auto &ms = store::tvs[tv].macros;
  if (m < 1 || m >= (long)ms.size()) return fail("Kann nicht verschoben werden");
  std::swap(ms[m], ms[m - 1]);
  app::afterExternalChange();
  if (!store::save()) return fail("Speichern fehlgeschlagen");
  ok();
}

void start() {
  if (running) return;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid().c_str(), AP_PASS);
  if (!routesDone) {
    routesDone = true;
    server.on("/", HTTP_GET, [] { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
    server.on("/api/data", HTTP_GET, handleData);
    server.on("/api/play", HTTP_POST, handlePlay);
    server.on("/api/stop", HTTP_POST, handleStop);
    server.on("/api/rec", HTTP_POST, handleRec);
    server.on("/api/recdone", HTTP_POST, handleRecDone);
    server.on("/api/recabort", HTTP_POST, handleRecAbort);
    server.on("/api/rename", HTTP_POST, handleRename);
    server.on("/api/del", HTTP_POST, handleDelete);
    server.on("/api/addtv", HTTP_POST, handleAddTv);
    server.on("/api/move", HTTP_POST, handleMove);
    server.on("/api/updwifi", HTTP_POST, handleUpdWifi);
    server.onNotFound([] { server.send(404, "text/plain", "Nicht gefunden"); });
  }
  server.begin();
  running = true;
  lastClientMs = millis();
}

void stop() {
  if (!running) return;
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  running = false;
}

void loop() {
  if (!running) return;
  server.handleClient();
  // Stromsparen: Stick-WLAN aus, wenn 5 min niemand verbunden war
  if (WiFi.softAPgetStationNum() > 0) lastClientMs = millis();
  else if (millis() - lastClientMs > AP_IDLE_MS) stop();

}

}  // namespace web

// ===========================================================================
// Arduino
// ===========================================================================
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Speaker.end();             // Verstaerker aus, sonst stoert er den IR-Empfang
  M5.Power.setExtOutput(true);  // Versorgung fuer die IR-LED
  M5.Mic.end();                 // Mikrofon wird nicht gebraucht
  M5.Display.setRotation(0);    // hochkant 135 x 240 (steht es auf dem Kopf: 2 eintragen)
  app::begin();
}

void loop() { app::loop(); }
