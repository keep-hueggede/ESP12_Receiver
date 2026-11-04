/*******************************************
 * ESP-ThreadReceiver - Thread-Collector fuer Sensorknoten
 *
 * ESP32-C6 (SuperMini): formt das Thread-Netz (Leader/FTD), nimmt die
 * Messwerte der Sensorknoten per CoAP entgegen und gibt sie als JSON-Zeilen
 * an den Raspberry Pi weiter (verdrahtet, UART0: GPIO16 TX / GPIO17 RX,
 * 115200 baud).
 *
 * Datenfluss:     Die Sensoren SENDEN (CoAP POST auf /ingest). Dieses Geraet
 *                 fragt nichts ab. Grund: die Sensoren laufen als Sleepy End
 *                 Devices mit abgeschaltetem Funkmodul und sind zwischen
 *                 ihren Sendezeitpunkten nicht erreichbar. Eine Abfrage
 *                 (Pull) wuerde in den Timeout laufen und den Stromspar-
 *                 effekt zunichte machen.
 *
 * Erweiterung:    Ein neuer Sensor braucht KEINE Aenderung an diesem Sketch.
 *                 Er kommissioniert sich mit dem PSKd (siehe JOINER_PSKD) und
 *                 traegt seinen Namen selbst im Feld "src" der Nutzlast.
 *
 * Rolle:          Thread-Leader, CoAP-Server, Commissioner
 *
 * Firmware-Update der Sensoren:
 *                 Der Pi laedt neue Sensor-Firmware von GitHub und schiebt
 *                 sie per UART hierher (Zeilen "FW BEGIN/DATA/END", siehe
 *                 handlePiCommand). Das Abbild liegt in der ungenutzten
 *                 SPIFFS-Partition, Version/Groesse/SHA-256 im NVS. Meldet
 *                 ein Sensor eine andere Version ("fw"), bekommt er in der
 *                 Antwort auf /ingest ein Angebot und holt das Abbild in
 *                 512-Byte-Bloecken ueber POST /fw ab. Pro Sensor und
 *                 Version hoechstens FW_MAX_OFFERS Angebote.
 *
 * Benoetigt:      Arduino-Core esp32 v3.x, Board "ESP32C6 Dev Module" bzw.
 *                 "MakerGO ESP32 C6 SuperMini". Die Debug-Konsole haengt fest
 *                 am USB-Serial/JTAG (siehe CONSOLE unten) und funktioniert
 *                 unabhaengig von der Menue-Einstellung "USB CDC On Boot".
 *******************************************/
#include <Arduino.h>
#include <OThread.h>
#include <OThreadCoAP.h>
#include <Preferences.h>
#include <map>
#include "esp_partition.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha256.h"

// --------------------------------------------------------------------------
// Debug-Konsole am USB-Port (der COM-Port, den der PC sieht)
//
// Der C6 SuperMini hat KEINEN USB-UART-Bridge-Chip - der COM-Port ist das
// eingebaute USB-Serial/JTAG-Peripheral (VID 303A / PID 1001). Der Arduino-
// Core bildet `Serial` aber nur dann darauf ab, wenn im Boardmenue
// "USB CDC On Boot: Enabled" gewaehlt ist. Default fuer alle C6-Boards ist
// *Disabled*, und dann gilt `#define Serial Serial0` -> UART0 auf GPIO16/17,
// also genau die Pins, die hier zum Pi gehen. Am COM-Port kommt dann nichts
// an ("keine Ausgabe in der seriellen Konsole").
//
// Deshalb wird die Konsole hier explizit an das USB-Serial/JTAG gebunden,
// unabhaengig von der Menue-Einstellung.
// --------------------------------------------------------------------------
#if ARDUINO_USB_CDC_ON_BOOT
#define CONSOLE Serial            // Serial ist bereits das USB-Serial/JTAG (HWCDC)
#else
static HWCDC USBConsole;          // USB-Serial/JTAG selbst instanziieren
#define CONSOLE USBConsole
#endif

// --------------------------------------------------------------------------
// Schnittstelle zum Raspberry Pi (UART0, 3-Leiter: TX, RX, GND)
// --------------------------------------------------------------------------
// SuperMini-Pinout (MakerGO): Pad "TX" = GPIO16 (TXD0), Pad "RX" = GPIO17
// (RXD0). Das sind zugleich die UART0-Standardpins des C6.
// Verdrahtung:
//   C6 Pad "TX" / GPIO16  -> Pi GPIO15 (RXD, pin 10)
//   C6 Pad "RX" / GPIO17  <- Pi GPIO14 (TXD, pin 8)
//   C6 GND                -> Pi GND (z. B. pin 6)
// Alternativ (empfohlen): C6 per USB am Pi, dort /dev/ttyACM0. Die JSON-Zeilen
// und "FW ..."-Befehle laufen dann ueber die Konsole; der Collector ignoriert
// alle Zeilen, die nicht mit '{' beginnen. Konsolenbefehle dann mit Enter.
#define PI_SERIAL    Serial0
#define PI_BAUD      115200
#define PI_TX_PIN    16     // Pad "TX", GPIO16 -> Pi RXD
#define PI_RX_PIN    17     // Pad "RX", GPIO17 <- Pi TXD

// --------------------------------------------------------------------------
// Thread-Netzkonfiguration (Operational Dataset)
//
// Dieses Geraet FORMT das Thread-Netz (Leader-Rolle). WICHTIG: initNew()
// erzeugt ein frisches Dataset (Timestamps + Mesh-Local-Prefix). Das wird in
// NVS persistiert und darf daher nur EINMAL beim ersten Start passieren,
// sonst aendert sich das Mesh-Local-Prefix bei jedem Reboot.
//
// Die Sensorknoten bekommen dieses Dataset NICHT hartcodiert, sondern per
// Commissioning ueber die Luft (siehe JOINER_PSKD). Damit existiert im
// gesamten Netz genau ein Dataset und genau ein Mesh-Local-Prefix.
// --------------------------------------------------------------------------
static const char     THREAD_NETWORK_NAME[] = "Wetter-Thread";
static const uint16_t THREAD_PAN_ID         = 0x2345;
static const uint8_t  THREAD_CHANNEL        = 15;
static const uint8_t  THREAD_EXT_PAN_ID[8]  = {0xDE,0xAD,0xBE,0xEF,0x00,0x00,0x01,0x02};
static const uint8_t  THREAD_NETWORK_KEY[16] = {0xA1,0xB2,0xC3,0xD4,0xE5,0xF6,0x07,0x18,
                                                0x29,0x3A,0x4B,0x5C,0x6D,0x7E,0x8F,0x90};

// --------------------------------------------------------------------------
// Commissioning
//
// Der PSKd ist das gemeinsame Geheimnis, mit dem sich ein neuer Sensor
// anmeldet. Erlaubt sind 6..32 Zeichen aus dem Base32-Thread-Alphabet
// (0-9, A-Y ohne I, O, Q, Z).
//
// Das Fenster wird beim Start geoeffnet und laeuft ab. Ein Druck auf 'c' in
// der Konsole oeffnet es erneut - so ist das Netz im Normalbetrieb zu.
// --------------------------------------------------------------------------
static const char JOINER_PSKD[] = "WETTER01";
#define COMMISSION_WINDOW_S  600   // Dauer des Anmeldefensters in Sekunden

// --------------------------------------------------------------------------
// CoAP-Ressource, auf der die Sensoren ihre Messwerte abliefern
// --------------------------------------------------------------------------
static const char *INGEST_PATH = "ingest";
static const char *FW_PATH     = "fw";

#define MAX_PAYLOAD  256   // groesste akzeptierte Nutzlast (JSON-Objekt)

// --------------------------------------------------------------------------
// Firmware-Puffer fuer die Sensoren
// --------------------------------------------------------------------------
#define FW_BLOCK        512   // bytes per CoAP response (OT_COAP_MAX_PAYLOAD)
#define FW_MAX_OFFERS   3     // per sensor and version, so a bad image cannot drain batteries
#define FW_SECTOR       4096

struct FwImage {
  String   version;           // "" = no complete image
  uint32_t size = 0;
  String   sha256;            // lowercase hex
};

static const esp_partition_t *g_fwPart = nullptr;
static FwImage  g_fw;              // complete image offered to the sensors
static FwImage  g_fwUpload;        // image currently coming from the Pi
static uint32_t g_fwWritten = 0;
static uint32_t g_fwErased = 0;
static std::map<String, uint8_t> g_fwOffers;   // src -> offers of g_fw.version
static SemaphoreHandle_t g_fwLock;             // g_fw is read in the OpenThread task
static SemaphoreHandle_t g_piLock;             // one writer per UART line
static Preferences g_prefs;

// --------------------------------------------------------------------------
// LEDs am SuperMini beim Start abschalten: RGB-LED (WS2812, GPIO8) und blaue
// LED (GPIO15). Die rote Power-LED haengt fest an 3V3 (nur per Hardware).
// --------------------------------------------------------------------------
#define LED_PIN      8
#define BLUE_LED_PIN 15

// Statistik
static uint32_t g_received = 0;   // angenommene Messungen
static uint32_t g_rejected = 0;   // verworfene Nutzlasten

// --------------------------------------------------------------------------
// JSON-Zeile an den Raspberry Pi weitergeben.
//
// Die Sensoren liefern bereits ein vollstaendiges JSON-Objekt inklusive dem
// Feld "src". Es wird unveraendert als eine Zeile durchgereicht - der Parser
// auf dem Pi sieht dasselbe Format wie bisher.
// --------------------------------------------------------------------------
// Lines come from the OpenThread task and from loop(): write each one in one piece.
// The Pi may be wired to the UART pins or plugged into USB, so both get every line.
static void sendToPi(const char *json) {
  xSemaphoreTake(g_piLock, portMAX_DELAY);
  PI_SERIAL.write((const uint8_t *)json, strlen(json));
  PI_SERIAL.write('\n');
  CONSOLE.write((const uint8_t *)json, strlen(json));
  CONSOLE.write('\n');
  xSemaphoreGive(g_piLock);
}

static void sendEventToPi(const char *event) {
  char line[160];
  snprintf(line, sizeof(line), "{\"sys\":\"%s\"}", event);
  sendToPi(line);
}

// --------------------------------------------------------------------------
// Firmware-Puffer: Abbild im Flash, Metadaten im NVS
// --------------------------------------------------------------------------
static void loadFirmwareMeta() {
  g_fwPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, nullptr);
  g_prefs.begin("fwstore", false);
  g_fw.version = g_prefs.getString("ver", "");
  g_fw.size = g_prefs.getUInt("size", 0);
  g_fw.sha256 = g_prefs.getString("sha", "");
  if (!g_fwPart) {
    g_fw = FwImage();
    CONSOLE.println("Firmware-Puffer: keine SPIFFS-Partition - Sensor-Updates deaktiviert.");
  } else if (g_fw.version.length()) {
    CONSOLE.printf("Firmware-Puffer: Sensor-Firmware %s (%lu Bytes)\n", g_fw.version.c_str(), (unsigned long)g_fw.size);
  }
}

static void setStoredFirmware(const FwImage &img) {
  xSemaphoreTake(g_fwLock, portMAX_DELAY);
  g_fw = img;
  g_fwOffers.clear();
  xSemaphoreGive(g_fwLock);
  g_prefs.putString("ver", img.version);
  g_prefs.putUInt("size", img.size);
  g_prefs.putString("sha", img.sha256);
}

static bool partitionSha256(uint32_t size, char *hexOut) {
  static uint8_t buf[FW_SECTOR];
  uint8_t digest[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  for (uint32_t off = 0; off < size; off += sizeof(buf)) {
    uint32_t n = min((uint32_t)sizeof(buf), size - off);
    if (esp_partition_read(g_fwPart, off, buf, n) != ESP_OK) {
      mbedtls_sha256_free(&ctx);
      return false;
    }
    mbedtls_sha256_update(&ctx, buf, n);
  }
  mbedtls_sha256_finish(&ctx, digest);
  mbedtls_sha256_free(&ctx);
  for (int i = 0; i < 32; i++) sprintf(hexOut + 2 * i, "%02x", digest[i]);
  return true;
}

static void fwReply(const char *fmt, ...) {
  char line[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  sendToPi(line);
}

// --------------------------------------------------------------------------
// Befehle vom Pi (eine Zeile je Befehl, Antwort als JSON-Zeile {"fw":...}):
//   FW STATUS                            -> status: gespeicherte Version
//   FW BEGIN <version> <size> <sha256>   -> ready  (altes Abbild wird ungueltig)
//   FW DATA <offset> <base64>            -> ack mit dem naechsten erwarteten Offset
//   FW END                               -> stored (SHA-256 geprueft) oder error
// --------------------------------------------------------------------------
static void handlePiCommand(char *line) {
  if (strncmp(line, "FW ", 3) != 0) {
    CONSOLE.printf("PI   %s\n", line);
    return;
  }
  char *cmd = line + 3;

  if (strcmp(cmd, "STATUS") == 0) {
    xSemaphoreTake(g_fwLock, portMAX_DELAY);
    FwImage img = g_fw;
    xSemaphoreGive(g_fwLock);
    fwReply("{\"fw\":\"status\",\"version\":\"%s\",\"size\":%lu,\"sha256\":\"%s\",\"capacity\":%lu}",
            img.version.c_str(), (unsigned long)img.size, img.sha256.c_str(),
            (unsigned long)(g_fwPart ? g_fwPart->size : 0));

  } else if (strncmp(cmd, "BEGIN ", 6) == 0) {
    char ver[32], sha[65];
    unsigned long size = 0;
    if (sscanf(cmd + 6, "%31s %lu %64s", ver, &size, sha) != 3 || strlen(sha) != 64) {
      fwReply("{\"fw\":\"error\",\"msg\":\"bad BEGIN\"}");
    } else if (!g_fwPart || size == 0 || size > g_fwPart->size) {
      fwReply("{\"fw\":\"error\",\"msg\":\"image does not fit\"}");
    } else {
      setStoredFirmware(FwImage());   // stop offering the old image before overwriting it
      g_fwUpload.version = ver;
      g_fwUpload.size = size;
      g_fwUpload.sha256 = sha;
      g_fwWritten = 0;
      g_fwErased = 0;
      CONSOLE.printf("Firmware-Upload %s (%lu Bytes) startet\n", ver, size);
      fwReply("{\"fw\":\"ready\"}");
    }

  } else if (strncmp(cmd, "DATA ", 5) == 0) {
    static uint8_t data[768];
    char *space = strchr(cmd + 5, ' ');
    if (!g_fwUpload.size || !space) {
      fwReply("{\"fw\":\"error\",\"msg\":\"no upload\"}");
      return;
    }
    *space = '\0';
    uint32_t off = strtoul(cmd + 5, nullptr, 10);
    size_t len = 0;
    if (mbedtls_base64_decode(data, sizeof(data), &len, (const uint8_t *)(space + 1), strlen(space + 1)) != 0) {
      fwReply("{\"fw\":\"error\",\"msg\":\"bad base64\"}");
      return;
    }
    if (off + len <= g_fwWritten) {
      // repeated chunk (the Pi missed our ack): already written
    } else if (off != g_fwWritten || off + len > g_fwUpload.size) {
      fwReply("{\"fw\":\"error\",\"msg\":\"unexpected offset\",\"expected\":%lu}", (unsigned long)g_fwWritten);
      return;
    } else {
      while (g_fwErased < off + len) {
        if (esp_partition_erase_range(g_fwPart, g_fwErased, FW_SECTOR) != ESP_OK) {
          fwReply("{\"fw\":\"error\",\"msg\":\"erase failed\"}");
          return;
        }
        g_fwErased += FW_SECTOR;
      }
      if (esp_partition_write(g_fwPart, off, data, len) != ESP_OK) {
        fwReply("{\"fw\":\"error\",\"msg\":\"write failed\"}");
        return;
      }
      g_fwWritten = off + len;
    }
    fwReply("{\"fw\":\"ack\",\"off\":%lu}", (unsigned long)g_fwWritten);

  } else if (strcmp(cmd, "END") == 0) {
    char hex[65];
    if (!g_fwUpload.size || g_fwWritten != g_fwUpload.size) {
      fwReply("{\"fw\":\"error\",\"msg\":\"incomplete\",\"written\":%lu}", (unsigned long)g_fwWritten);
    } else if (!partitionSha256(g_fwUpload.size, hex) || g_fwUpload.sha256 != hex) {
      fwReply("{\"fw\":\"error\",\"msg\":\"checksum mismatch\"}");
    } else {
      setStoredFirmware(g_fwUpload);
      CONSOLE.printf("Sensor-Firmware %s gespeichert, wird den Sensoren angeboten\n", g_fwUpload.version.c_str());
      fwReply("{\"fw\":\"stored\",\"version\":\"%s\"}", g_fwUpload.version.c_str());
    }
    g_fwUpload = FwImage();

  } else {
    fwReply("{\"fw\":\"error\",\"msg\":\"unknown command\"}");
  }
}

// Value of a "key":"text" pair in a flat JSON object ("" if missing)
static String jsonText(const String &json, const char *key) {
  String pattern = String("\"") + key + "\":\"";
  int start = json.indexOf(pattern);
  if (start < 0) return "";
  start += pattern.length();
  int end = json.indexOf('"', start);
  return end < 0 ? "" : json.substring(start, end);
}

// Offer the stored image to a sensor that reports another version ("dev" builds opt out)
static bool firmwareOffer(const String &payload, char *out, size_t outLen) {
  String fw = jsonText(payload, "fw");
  String src = jsonText(payload, "src");
  if (!fw.length() || fw == "dev" || !src.length()) return false;

  bool offer = false;
  uint8_t count = 0;
  String version;
  xSemaphoreTake(g_fwLock, portMAX_DELAY);
  if (g_fw.version.length() && fw != g_fw.version) {
    uint8_t &n = g_fwOffers[src];
    if (n < FW_MAX_OFFERS) {
      count = ++n;
      offer = true;
      version = g_fw.version;
      snprintf(out, outLen, "{\"ota\":\"%s\",\"size\":%lu,\"sha256\":\"%s\"}",
               g_fw.version.c_str(), (unsigned long)g_fw.size, g_fw.sha256.c_str());
    }
  }
  xSemaphoreGive(g_fwLock);

  if (offer) {
    char event[120];
    snprintf(event, sizeof(event), "fw offer %s %s -> %s (%u/%u)", src.c_str(), fw.c_str(), version.c_str(),
             count, FW_MAX_OFFERS);
    CONSOLE.printf("FW   Angebot an %s: %s -> %s (%u/%u)\n", src.c_str(), fw.c_str(), version.c_str(), count,
                   FW_MAX_OFFERS);
    sendEventToPi(event);
  }
  return offer;
}

// --------------------------------------------------------------------------
// CoAP-Handler: ein Sensor holt einen Block des Abbilds (POST /fw, Nutzlast:
// Offset als Dezimalzahl). Antwort: bis zu FW_BLOCK Bytes ab diesem Offset.
// --------------------------------------------------------------------------
static void handleFw(OThreadCoAPRequest &req, OThreadCoAPResponse &resp, void *ctx) {
  static uint8_t block[FW_BLOCK];
  uint32_t off = (uint32_t)strtoul(req.payloadString().c_str(), nullptr, 10);

  xSemaphoreTake(g_fwLock, portMAX_DELAY);
  uint32_t size = g_fw.size;
  size_t n = 0;
  bool ok = size > 0 && off < size;
  if (ok) {
    n = min((uint32_t)FW_BLOCK, size - off);
    ok = esp_partition_read(g_fwPart, off, block, n) == ESP_OK;
  }
  xSemaphoreGive(g_fwLock);

  if (!ok) {
    resp.setCode(OT_COAP_RESP_NOT_FOUND);
  } else {
    resp.setContentFormat(OT_COAP_FORMAT_OCTET_STREAM);
    resp.setPayload(block, n);
  }
  resp.send();
}

// --------------------------------------------------------------------------
// CoAP-Handler: ein Sensor liefert Messwerte ab (POST /ingest)
//
// Die Nutzlast muss ein einzelnes JSON-Objekt sein. Mehr wird hier bewusst
// nicht geprueft: der Knoten ist bereits per Thread-Netzwerkschluessel
// authentifiziert, und die Auswertung passiert auf dem Pi.
// --------------------------------------------------------------------------
static void handleIngest(OThreadCoAPRequest &req, OThreadCoAPResponse &resp, void *ctx) {
  String payload = req.payloadString();

  bool ok = payload.length() >= 2 && payload.length() <= MAX_PAYLOAD && payload[0] == '{' && payload[payload.length() - 1] == '}';

  if (ok) {
    g_received++;
    sendToPi(payload.c_str());
    CONSOLE.printf("RX   %-40s %s\n", req.remoteIP().toString().c_str(), payload.c_str());
  } else {
    g_rejected++;
    CONSOLE.printf("DROP %-40s (%u Bytes, kein JSON-Objekt)\n", req.remoteIP().toString().c_str(), (unsigned)payload.length());
  }

  // Immer quittieren: der Sensor wertet das ACK aus, um zu erkennen, ob seine
  // Messung angekommen ist. Die Antwort traegt ggf. ein Firmware-Angebot.
  char offer[160];
  if (ok && firmwareOffer(payload, offer, sizeof(offer))) {
    resp.setContentFormat(OT_COAP_FORMAT_JSON);
    resp.setPayload(offer);
  }
  resp.send();
}

// --------------------------------------------------------------------------
// Thread-Netz formen (Dataset-basiert, persistiert in NVS)
// --------------------------------------------------------------------------
static bool joinNetwork() {
  OThread.begin(false);

  if (!OThread.hasActiveDataset()) {
    DataSet ds;
    ds.initNew();
    ds.setNetworkName(THREAD_NETWORK_NAME);
    ds.setPanId(THREAD_PAN_ID);
    ds.setChannel(THREAD_CHANNEL);
    ds.setExtendedPanId(THREAD_EXT_PAN_ID);
    ds.setNetworkKey(THREAD_NETWORK_KEY);
    OThread.commitDataSet(ds);
    CONSOLE.println("Neues Dataset angelegt (initNew + Netzparameter).");
  } else {
    CONSOLE.println("Wiederverwende gespeichertes Dataset aus NVS.");
  }

  OThread.networkInterfaceUp();
  OThread.start();

  CONSOLE.print("Warte auf Thread-Verbindung");
  uint32_t start = millis();
  while (OThread.otGetDeviceRole() < OT_ROLE_CHILD) {
    if (millis() - start > 60000) {
      CONSOLE.println("\nTimeout - keine Thread-Verbindung.");
      return false;
    }
    CONSOLE.print('.');
    delay(200);
  }
  CONSOLE.println();

  CONSOLE.printf("Verbunden als %s\n", OThread.otGetStringDeviceRole());
  OThread.otPrintNetworkInformation(CONSOLE);
  CONSOLE.printf("Mesh-Local EID: %s\n", OThread.getMeshLocalEid().toString().c_str());
  return true;
}

// --------------------------------------------------------------------------
// Anmeldefenster fuer neue Sensoren oeffnen.
//
// startCommissioner() petitioniert das Netz um die Commissioner-Rolle; das
// geht nur, wenn dieses Geraet attached ist. addJoiner() traegt anschliessend
// den PSKd ein, mit dem sich JEDER Sensor anmelden darf (eui64 = nullptr).
// --------------------------------------------------------------------------
static bool openCommissioning() {
#if CONFIG_OPENTHREAD_COMMISSIONER
  if (OThread.otGetDeviceRole() < OT_ROLE_CHILD) {
    CONSOLE.println("Commissioning nicht moeglich: Geraet ist nicht attached.");
    return false;
  }

  otError err = OThread.startCommissioner();
  if (err != OT_ERROR_NONE) {
    CONSOLE.printf("startCommissioner fehlgeschlagen: %d\n", (int)err);
    return false;
  }

  err = OThread.addJoiner(JOINER_PSKD, COMMISSION_WINDOW_S, nullptr);
  if (err != OT_ERROR_NONE) {
    CONSOLE.printf("addJoiner fehlgeschlagen: %d\n", (int)err);
    OThread.stopCommissioner();
    return false;
  }

  CONSOLE.printf("Anmeldefenster offen fuer %d s (PSKd '%s'). Neue Sensoren koennen jetzt beitreten.\n", COMMISSION_WINDOW_S, JOINER_PSKD);
  sendEventToPi("commissioning window open");
  return true;
#else
  CONSOLE.println("Commissioner ist in diesem Core-Build nicht aktiviert.");
  return false;
#endif
}

// Collects bytes into lines; returns the finished, non-empty line or nullptr
struct LineReader {
  char buf[1100];
  size_t len = 0;
  char *feed(int c) {
    if (c == '\n' || len == sizeof(buf) - 1) {
      buf[len] = '\0';
      bool done = len > 0;
      len = 0;
      return done ? buf : nullptr;
    }
    if (c != '\r') buf[len++] = (char)c;
    return nullptr;
  }
};

// --------------------------------------------------------------------------
void setup() {
  CONSOLE.setRxBufferSize(4096);   // firmware blocks from the Pi when it is plugged into USB
  CONSOLE.begin(115200);
  // Auf den Host warten, aber nicht ewig: ohne offenen Monitor laeuft es weiter.
  while (!CONSOLE && millis() < 5000) {
    delay(10);
  }
  delay(100);  // sonst geht die erste Zeile beim USB-Enumerieren verloren
  CONSOLE.println("=== ESP-ThreadReceiver: Thread-Collector -> Pi (UART0) ===");

  // WS2812 keeps its last colour; only a black frame switches it off
  rgbLedWrite(LED_PIN, 0, 0, 0);
  // High impedance switches the blue LED off whether it is wired active-high or active-low
  pinMode(BLUE_LED_PIN, INPUT);

  g_fwLock = xSemaphoreCreateMutex();
  g_piLock = xSemaphoreCreateMutex();
  loadFirmwareMeta();

  // UART zum Raspberry Pi (JSON-Zeilen, 115200 baud). Groesserer RX-Puffer fuer Firmware-Bloecke.
  PI_SERIAL.setRxBufferSize(4096);
  PI_SERIAL.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  delay(100);
  sendEventToPi("ESP-ThreadReceiver boot");

  if (!joinNetwork()) {
    CONSOLE.println("Thread-Start fehlgeschlagen. Bitte Dataset-Werte pruefen.");
  }

  // CoAP-Server: hier liefern die Sensoren ab.
  OThreadCoAPServer.on(INGEST_PATH, OT_COAP_METHOD_POST, handleIngest, nullptr);
  OThreadCoAPServer.on(FW_PATH, OT_COAP_METHOD_POST, handleFw, nullptr);
  OThreadCoAPServer.begin();
  CONSOLE.printf("CoAP-Server bereit: POST /%s, POST /%s\n", INGEST_PATH, FW_PATH);

  // Beim Start ein Anmeldefenster oeffnen, damit neue Sensoren ohne
  // Zusatzhandgriff dazukommen koennen.
  openCommissioning();

  CONSOLE.println("Bereit. 'c' + Enter oeffnet ein neues Anmeldefenster, 's' + Enter zeigt den Status.");
}

// --------------------------------------------------------------------------
void loop() {
  static const char *lastRole = "";
  static uint32_t lastStatus = 0;

  // Rollenwechsel melden (z. B. Leader -> Router nach einem Merge)
  const char *role = OThread.otGetStringDeviceRole();
  if (strcmp(role, lastRole) != 0) {
    lastRole = role;
    CONSOLE.printf("Rolle: %s\n", role);
  }

  // USB: Pi protocol ("FW ...") and console commands 'c' / 's', one per line
  static LineReader usb;
  while (CONSOLE.available()) {
    char *line = usb.feed(CONSOLE.read());
    if (!line) continue;
    if (strncmp(line, "FW ", 3) == 0) {
      handlePiCommand(line);
    } else if (strcasecmp(line, "c") == 0) {
      openCommissioning();
    } else if (strcasecmp(line, "s") == 0) {
      CONSOLE.printf("Status: Rolle=%s RLOC16=0x%04X empfangen=%lu verworfen=%lu Laufzeit=%lus\n",
                     role, OThread.getRloc16(), (unsigned long)g_received, (unsigned long)g_rejected, (unsigned long)(millis() / 1000));
    }
  }

  // Zeilen vom Pi: Firmware-Befehle ("FW ..."), alles andere nur auf der
  // Konsole zeigen (Verdrahtungs-Check: auf dem Pi z. B. `echo ping > /dev/serial0`).
  static LineReader uart;
  while (PI_SERIAL.available()) {
    char *line = uart.feed(PI_SERIAL.read());
    if (line) handlePiCommand(line);
  }

  // Lebenszeichen alle 60 s, damit im Log sichtbar bleibt, dass der Collector
  // laeuft - auch wenn gerade kein Sensor sendet.
  uint32_t now = millis();
  if (now - lastStatus >= 60000) {
    lastStatus = now;
    CONSOLE.printf("... %s, %lu Messung(en) empfangen\n", role, (unsigned long)g_received);
  }

  delay(20);
}
