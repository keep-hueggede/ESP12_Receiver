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
 * Benoetigt:      Arduino-Core esp32 v3.x, Board "ESP32C6 Dev Module" bzw.
 *                 "MakerGO ESP32 C6 SuperMini". Die Debug-Konsole haengt fest
 *                 am USB-Serial/JTAG (siehe CONSOLE unten) und funktioniert
 *                 unabhaengig von der Menue-Einstellung "USB CDC On Boot".
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

#define MAX_PAYLOAD  256   // groesste akzeptierte Nutzlast (JSON-Objekt)

// --------------------------------------------------------------------------
// RGB-LED am SuperMini (GPIO8) beim Start abschalten
// --------------------------------------------------------------------------
#define LED_PIN 8

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
static void sendToPi(const char *json) {
  PI_SERIAL.println(json);
}

static void sendEventToPi(const char *event) {
  PI_SERIAL.print("{\"sys\":\"");
  PI_SERIAL.print(event);
  PI_SERIAL.println("\"}");
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
  // Messung angekommen ist. Ohne send() wuerde der Server zwar automatisch
  // antworten, aber wir wollen das hier explizit im Code sehen.
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

// --------------------------------------------------------------------------
void setup() {
  CONSOLE.begin(115200);
  // Auf den Host warten, aber nicht ewig: ohne offenen Monitor laeuft es weiter.
  while (!CONSOLE && millis() < 5000) {
    delay(10);
  }
  delay(100);  // sonst geht die erste Zeile beim USB-Enumerieren verloren
  CONSOLE.println("=== ESP-ThreadReceiver: Thread-Collector -> Pi (UART0) ===");

  // RGB-LED am SuperMini abschalten (GPIO8)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // UART zum Raspberry Pi (JSON-Zeilen, 115200 baud)
  PI_SERIAL.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  delay(100);
  sendEventToPi("ESP-ThreadReceiver boot");

  if (!joinNetwork()) {
    CONSOLE.println("Thread-Start fehlgeschlagen. Bitte Dataset-Werte pruefen.");
  }

  // CoAP-Server: hier liefern die Sensoren ab.
  OThreadCoAPServer.on(INGEST_PATH, OT_COAP_METHOD_POST, handleIngest, nullptr);
  OThreadCoAPServer.begin();
  CONSOLE.printf("CoAP-Server bereit: POST /%s\n", INGEST_PATH);

  // Beim Start ein Anmeldefenster oeffnen, damit neue Sensoren ohne
  // Zusatzhandgriff dazukommen koennen.
  openCommissioning();

  CONSOLE.println("Bereit. Taste 'c' oeffnet ein neues Anmeldefenster, 's' zeigt den Status.");
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

  // Konsolen-Kommandos
  while (CONSOLE.available()) {
    int c = CONSOLE.read();
    if (c == 'c' || c == 'C') {
      openCommissioning();
    } else if (c == 's' || c == 'S') {
      CONSOLE.printf("Status: Rolle=%s RLOC16=0x%04X empfangen=%lu verworfen=%lu Laufzeit=%lus\n",
                     role, OThread.getRloc16(), (unsigned long)g_received, (unsigned long)g_rejected, (unsigned long)(millis() / 1000));
    }
  }

  // Alles, was vom Pi kommt, auf der Konsole zeigen (Verdrahtungs-Check:
  // auf dem Pi z. B. `echo ping > /dev/serial0`).
  static char piLine[128];
  static size_t piLen = 0;
  while (PI_SERIAL.available()) {
    int c = PI_SERIAL.read();
    if (c == '\n' || piLen == sizeof(piLine) - 1) {
      piLine[piLen] = '\0';
      CONSOLE.printf("PI   %s\n", piLine);
      piLen = 0;
    } else if (c != '\r') {
      piLine[piLen++] = (char)c;
    }
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
