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
 * Benötigt:       Arduino-Core esp32 v3.x, Board "ESP32C6 Dev Module",
 *                 CDCOnBoot = Enabled (Serial = USB-CDC am COM-Port).
 *******************************************/
#include <Arduino.h>
#include <OThread.h>
#include <OThreadCoAP.h>

// --------------------------------------------------------------------------
// Schnittstelle zum Raspberry Pi (UART0, 3-Leiter: TX, RX, GND)
// --------------------------------------------------------------------------
// ESP32-C6 SuperMini: UART0 TX = GPIO16, RX = GPIO17 (Pads "TX"/"RX").
// Verdrahtung:
//   C6 GPIO16 (TX)  -> Pi GPIO15 (RX, pin 10)
//   C6 GPIO17 (RX)  -> Pi GPIO14 (TX, pin 8)
//   C6 GND          -> Pi GND
#define PI_SERIAL    Serial0
#define PI_BAUD      115200
#define PI_TX_PIN    16     // GPIO16 -> Pi RX
#define PI_RX_PIN    17     // GPIO17 <- Pi TX

// --------------------------------------------------------------------------
// Thread-Netzkonfiguration (Operational Dataset)
//
// WICHTIG: Muss identisch mit den THREAD_*-Werten der ESP-BME-Knoten sein
// (bzw. dem Netz des Border Routers, falls vorhanden). Trage hier die Werte
// deines Thread-Netzwerks ein.
// --------------------------------------------------------------------------
static const char     THREAD_NETWORK_NAME[] = "OpenThread-XXXX";
static const uint16_t THREAD_PAN_ID         = 0x1234;
static const uint8_t  THREAD_CHANNEL        = 15;
static const uint8_t  THREAD_EXT_PAN_ID[8]  = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t  THREAD_NETWORK_KEY[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

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
  {"wetter1", "fdde:ad00:beef:0000:0000:0000:0000:0001"},  // TODO: EID des Knotens eintragen
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

  DataSet ds;
  ds.setNetworkName(THREAD_NETWORK_NAME);
  ds.setPanId(THREAD_PAN_ID);
  ds.setChannel(THREAD_CHANNEL);
  ds.setExtendedPanId(THREAD_EXT_PAN_ID);
  ds.setNetworkKey(THREAD_NETWORK_KEY);

  OThread.commitDataSet(ds);
  OThread.networkInterfaceUp();
  OThread.start();

  Serial.print("Warte auf Thread-Verbindung");
  uint32_t start = millis();
  while (OThread.otGetDeviceRole() < OT_ROLE_CHILD) {
    if (millis() - start > 45000) {
      Serial.println("\nTimeout - keine Thread-Verbindung.");
      return false;
    }
    Serial.print('.');
    delay(200);
  }
  Serial.println();

  Serial.printf("Verbunden als %s\n", OThread.otGetStringDeviceRole());
  OThread.otPrintNetworkInformation(Serial);
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
  Serial.begin(115200);
  delay(500);
  Serial.println("=== ESP-ThreadReceiver: Thread + CoAP -> Pi (UART0) ===");

  // RGB-LED am SuperMini abschalten (GPIO8)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // UART zum Raspberry Pi (JSON-Zeilen, 115200 baud)
  PI_SERIAL.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  delay(100);
  PI_SERIAL.println("{\"sys\":\"ESP-ThreadReceiver boot\"}");

  // Thread-Netz beitreten / formen
  if (!joinNetwork()) {
    Serial.println("Thread-Beitritt fehlgeschlagen. Bitte Dataset-Werte prüfen.");
  }

  coap.setTimeout(COAP_TIMEOUT_MS);

  Serial.printf("Bereit. Pollinger %u Node(s) alle %d ms über UART0 (%d baud) an den Pi.\n",
                NODE_COUNT, (int)POLL_INTERVAL_MS, (int)PI_BAUD);
}

// --------------------------------------------------------------------------
void loop() {
  static uint32_t lastRound = 0;

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
          Serial.printf("OK   %-10s %s\n", nodes[i].name, body);
        } else {
          nodeState[i].failures++;
          const char *why = (rc < 0) ? "timeout" : "keine Antwort";
          sendErrorToPi(nodes[i].name, why);
          Serial.printf("FAIL %-10s code=%d (%s)\n", nodes[i].name, rc, why);
        }
      }
    }
  } else {
    delay(500);
  }

  delay(50);
}