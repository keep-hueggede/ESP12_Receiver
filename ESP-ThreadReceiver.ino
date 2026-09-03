/*******************************************
 * ESP-ThreadReceiver - Thread + CoAP Empfänger
 *
 * ESP32-C6 (SuperMini): tritt demselben Thread-Netzwerk bei wie die
 * ESP-BME-Wetterknoten und fragt deren CoAP-Ressource /weather/all
 * ab. Die Messwerte werden als JSON-Zeilen an den Raspberry Pi
 * übergeben (verdrahtet, UART0: GPIO16 TX / GPIO17 RX, 115200 baud).
 *
 * Übertragung:    Thread (802.15.4) + CoAP (UDP, 5683)
 * Rolle:          CoAP-Client / Thread-Router (FTD), kein OTBR nötig
 * Ausgabe:        JSON-Zeile je Node und Abfrage (siehe sendToPi)
 *
 * Erweiterung:    Weitere Thread-Sender werden einfach als neuer Eintrag
 *                 im Array `nodes[]` ergänzt (Name + IPv6-Adresse).
 *
 * Benötigt:       Arduino-Core esp32 v3.x, Board "ESP32C6 Dev Module"
 *                 bzw. "MakerGO ESP32 C6 SuperMini". Die Debug-Konsole
 *                 hängt fest am USB-Serial/JTAG (siehe CONSOLE unten) und
 *                 funktioniert unabhängig von der Menü-Einstellung
 *                 "USB CDC On Boot".
 *******************************************/
#include <Arduino.h>
#include <OThread.h>
#include <OThreadCoAP.h>

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
#define PI_SERIAL    Serial0
#define PI_BAUD      115200
#define PI_TX_PIN    16     // Pad "TX", GPIO16 -> Pi RXD
#define PI_RX_PIN    17     // Pad "RX", GPIO17 <- Pi TXD

// --------------------------------------------------------------------------
// Thread-Netzkonfiguration (Operational Dataset)
//
// Dieses Gerät FORMST das Thread-Netz (Leader-Rolle, wie der offizielle
// "LeaderNode" Example) und vergibt dabei frische Timestamps + Mesh-Local-
// Prefix via initNew(). Der ESP-BME-Knoten tritt mit NUR dem Network Key
// bei (Client-Pattern, siehe sensor_client Example).
//
// Beim ersten Start wird das Netz mit diesen festen Werten angelegt. Damit
// der Knoten sich zuordnet, müssen Network Key (und idealerweise Name/PAN/
// Kanal) in beiden Sketches identisch sein.
// --------------------------------------------------------------------------
static const char     THREAD_NETWORK_NAME[] = "Wetter-Thread";
static const uint16_t THREAD_PAN_ID         = 0x2345;
static const uint8_t  THREAD_CHANNEL        = 15;
static const uint8_t  THREAD_EXT_PAN_ID[8]  = {0xDE,0xAD,0xBE,0xEF,0x00,0x00,0x01,0x02};
static const uint8_t  THREAD_NETWORK_KEY[16] = {0xA1,0xB2,0xC3,0xD4,0xE5,0xF6,0x07,0x18,
                                                0x29,0x3A,0x4B,0x5C,0x6D,0x7E,0x8F,0x90};

// --------------------------------------------------------------------------
// Nodes (Thread-Sender), die abgefragt werden sollen.
//
// Jede Zeile = ein Gerät. Die IPv6-Adresse ist die Thread-EID (Mesh-Local
// EID) des jeweiligen Knotens - ausgegeben von dessen otPrintNetworkInformation
// ("MLEID" bzw. die fdXX:...-Adresse unter IPv6) bzw. `ot ipaddr`.
// Neues Gerät anlegen = neue Zeile im Array, fertig.
// --------------------------------------------------------------------------
struct NodeConfig {
  const char *name;  // Anzeigename, wird als "src" an den Pi gesendet
  const char *eid;   // IPv6-Adresse des Knotens (Thread-EID)
};

static NodeConfig nodes[] = {
  // Mesh-Local EID des H2 - stabil ueber Reboots und Rollenwechsel. Sie wird
  // vom ESP-BME-Sketch beim Start als "Mesh-Local EID: ..." ausgegeben.
  {"wetter1", "fdd5:f199:8258:b9ad:6e93:1619:4fe4:41d6"},  // ESP-BME (H2)

  // {"wetter2", "fdde:ad00:beef:0000:0000:0000:0000:0002"},  // weiterer Knoten
};
#define NODE_COUNT (sizeof(nodes) / sizeof(nodes[0]))

// --------------------------------------------------------------------------
// Abfrage-Timing
// --------------------------------------------------------------------------
#define POLL_INTERVAL_MS  5000   // Abstand zwischen zwei Abfrage-Runden
#define COAP_TIMEOUT_MS   3000   // Timeout je Node-Abfrage
#define MAX_FAILURES      3      // Fehlversuche, bis ein Node als offline markiert wird

static const char *WEATHER_PATH = "weather/all";

// --------------------------------------------------------------------------
// RGB-LED am SuperMini (GPIO8) beim Start abschalten
// --------------------------------------------------------------------------
#define LED_PIN 8

// --------------------------------------------------------------------------
// OpenThread / CoAP
// --------------------------------------------------------------------------
OThreadCoAPClient coap;

// Letzter bekannter Status je Node (für Offline-Erkennung)
static struct {
  uint32_t lastOkMs;   // millis() des letzten erfolgreichen Abrufs
  uint32_t failures;   // aufeinanderfolgende Fehlversuche
} nodeState[NODE_COUNT];

// --------------------------------------------------------------------------
// JSON-Zeile an den Raspberry Pi senden. Erwartet wird ein fertiges
// JSON-Objekt OHNE äußere Klammern, z. B.:
//   "temp":21.5,"pressure":1013.2,...
// --------------------------------------------------------------------------
static void sendToPi(const char *src, const char *body) {
  PI_SERIAL.print("{\"src\":\"");
  PI_SERIAL.print(src);
  PI_SERIAL.print("\",");
  PI_SERIAL.print(body);
  PI_SERIAL.println("}");
}

static void sendErrorToPi(const char *src, const char *err) {
  PI_SERIAL.print("{\"src\":\"");
  PI_SERIAL.print(src);
  PI_SERIAL.print("\",\"error\":\"");
  PI_SERIAL.print(err);
  PI_SERIAL.println("\"}");
}

// --------------------------------------------------------------------------
// Thread-Netz beitreten / formen (Dataset-basiert)
// --------------------------------------------------------------------------
static bool joinNetwork() {
  OThread.begin(false);

  // Dieses Gerät FORMST das Thread-Netz (Leader-Rolle). WICHTIG: initNew()
  // erzeugt ein frisches Dataset (Timestamps + Mesh-Local-Prefix). Das wird
  // in NVS persistiert und darf daher nur EINMAL beim ersten Start passieren,
  // sonst ändert sich das Mesh-Local-Prefix bei jedem Reboot und damit auch
  // die EID aller Knoten. Nach einem Reboot wird das gespeicherte Dataset
  // übernommen und das Netz identisch wiederhergestellt.
  if (!OThread.hasActiveDataset()) {
    DataSet ds;
    ds.initNew();
    ds.setNetworkName(THREAD_NETWORK_NAME);
    ds.setPanId(THREAD_PAN_ID);
    ds.setChannel(THREAD_CHANNEL);
    ds.setExtendedPanId(THREAD_EXT_PAN_ID);
    ds.setNetworkKey(THREAD_NETWORK_KEY);
    OThread.commitDataSet(ds);
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
  CONSOLE.printf("Mesh-Local EID (Leader): %s\n", OThread.getMeshLocalEid().toString().c_str());
  return true;
}

// --------------------------------------------------------------------------
// Einen Node per CoAP abfragen und Antwort als JSON-Body liefern.
// Rückgabe: 0 = Erfolg, <0 = CoAP-Fehler (siehe OT_COAP_ERROR_*)
// --------------------------------------------------------------------------
static int fetchNode(const NodeConfig *node, char *out, size_t outLen) {
  int code = coap.GET(node->eid, WEATHER_PATH);
  if (code <= 0) {
    return code;  // negativer OT_COAP_ERROR_*
  }
  String payload = coap.getString();
  if (payload.length() == 0) {
    return -2;  // leere Antwort
  }
  size_t n = payload.length();
  if (n > outLen - 1) n = outLen - 1;
  memcpy(out, payload.c_str(), n);
  out[n] = '\0';
  return 0;
}

// --------------------------------------------------------------------------
void setup() {
  CONSOLE.begin(115200);
  // Auf den Host warten, aber nicht ewig: ohne offenen Monitor laeuft es weiter.
  while (!CONSOLE && millis() < 5000) {
    delay(10);
  }
  delay(100);  // sonst geht die erste Zeile beim USB-Enumerieren verloren
  CONSOLE.println("=== ESP-ThreadReceiver: Thread + CoAP -> Pi (UART0) ===");

  // RGB-LED am SuperMini abschalten (GPIO8)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // UART zum Raspberry Pi (JSON-Zeilen, 115200 baud)
  PI_SERIAL.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  delay(100);
  PI_SERIAL.println("{\"sys\":\"ESP-ThreadReceiver boot\"}");

  // Thread-Netz beitreten / formen
  if (!joinNetwork()) {
    CONSOLE.println("Thread-Beitritt fehlgeschlagen. Bitte Dataset-Werte prüfen.");
  }

  coap.setTimeout(COAP_TIMEOUT_MS);

  // Kommittiertes Dataset zur Kontrolle ausgeben
  CONSOLE.printf("Dataset: Name=%s PAN=0x%04X Ch=%u ActiveDs=%s\n",
                OThread.getNetworkName().c_str(), (int)OThread.getPanId(), (int)OThread.getChannel(),
                OThread.hasActiveDataset() ? "ja" : "nein");
  const uint8_t *xp = OThread.getExtendedPanId();
  CONSOLE.printf("ExtPAN: %02X%02X%02X%02X%02X%02X%02X%02X\n",
                xp[0],xp[1],xp[2],xp[3],xp[4],xp[5],xp[6],xp[7]);
  CONSOLE.printf("EUI64 : %s\n", OThread.getEui64().c_str());

  CONSOLE.printf("Bereit. Polle %u Node(s) alle %d ms ueber UART0 (%d baud) an den Pi.\n",
                NODE_COUNT, (int)POLL_INTERVAL_MS, (int)PI_BAUD);
}

// --------------------------------------------------------------------------
void loop() {
  static uint32_t lastRound = 0;
  static uint32_t lastStatus = 0;
  static const char *lastRole = "";

  // Nur poller, wenn Thread aktiv ist
  if (OThread.otGetDeviceRole() >= OT_ROLE_CHILD) {
    uint32_t now = millis();
    if (now - lastRound >= POLL_INTERVAL_MS) {
      lastRound = now;

      char body[256];
      for (size_t i = 0; i < NODE_COUNT; i++) {
        int rc = fetchNode(&nodes[i], body, sizeof(body));
        if (rc == 0) {
          nodeState[i].lastOkMs = now;
          nodeState[i].failures = 0;
          sendToPi(nodes[i].name, body);
          CONSOLE.printf("OK   %-10s %s\n", nodes[i].name, body);
        } else {
          nodeState[i].failures++;
          const char *why = (rc < 0) ? "timeout" : "keine Antwort";
          sendErrorToPi(nodes[i].name, why);
          CONSOLE.printf("FAIL %-10s code=%d (%s)\n", nodes[i].name, rc, why);
        }
      }
    }
  } else {
    // Thread-Zustand im Hintergrund melden, solange noch keine Rolle besteht
    const char *role = OThread.otGetStringDeviceRole();
    uint32_t now = millis();
    if (strcmp(role, lastRole) != 0 || now - lastStatus > 10000) {
      lastStatus = now;
      lastRole = role;
      CONSOLE.printf("Rolle: %s\n", role);
    }
    delay(500);
  }

  delay(50);
}