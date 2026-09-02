/*******************************************
 * ESP-ThreadReceiver-LoRa - LoRa Empfänger (Zentrale)
 *
 * ESP32-C6 (SuperMini) mit LoRa-Modul (SX127x, SPI): empfängt die
 * LoRa-Pakete der Waldknoten (ESP-BME-LoRa) und übergibt die
 * Messwerte als JSON-Zeilen an den Raspberry Pi
 * (verdrahtet, UART0: GPIO16 TX / GPIO17 RX, 115200 baud).
 *
 * Übertragung:    LoRa (SPI, SX127x) @ 868 MHz, SF12
 * Rolle:          LoRa-Empfänger, einmal dauerhaft empfangsbereit
 * Ausgabe:        JSON-Zeile je Node und Paket (siehe sendToPi)
 *
 * Erweiterung:    Weitere Waldknoten brauchen hier NICHTS geändert zu
 *                 werden - jeder Sender trägt seinen Namen im Paket.
 *
 * Benötigt:       Arduino-Core esp32 v3.x, Board "ESP32C6 Dev Module",
 *                 CDCOnBoot = Enabled, RadioLib-Bibliothek.
 *******************************************/
#include <Arduino.h>
#include <RadioLib.h>

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
// LoRa-Modul am SPI-Bus des ESP32-C6 (Verdrahtung anpassen!)
// --------------------------------------------------------------------------
#define LORA_SS   18       // NSS/CS  -> GPIO18 (Default SS des C6)
#define LORA_RST  10       // Reset   -> GPIO10 (frei)
#define LORA_DIO0 11       // DIO0    -> GPIO11 (frei, Interrupt für RX)
// SCK=GPIO21, MOSI=GPIO19, MISO=GPIO20 (Default-SPI des C6)

// --------------------------------------------------------------------------
// LoRa-Funkparameter (müssen auf Sender und Empfänger identisch sein!)
// --------------------------------------------------------------------------
#define LORA_FREQ_MHZ   868.1F   // EU-LoRa ISM-Band, Kanal 1
#define LORA_SF         12       // Spreading Factor 12 = maximale Reichweite
#define LORA_BW         125.0F   // Bandbreite 125 kHz
#define LORA_CR         8        // Coding Rate 4/8 (mehr Redundanz, bessere Link-Budget)
#define LORA_SYNC       0x12     // Sync-Word (Standard LoRa)
#define LORA_POWER      20       // Sendeleistung (auf Empfänger nur für ACK-spätere Nutzung)
#define LORA_PREAMBLE   8        // Preamble-Symbole
#define LORA_RX_TIMEOUT 0        // RadioLib: 0 = Endlos-Empfang

// --------------------------------------------------------------------------
// RGB-LED am SuperMini (GPIO8) beim Start abschalten
// --------------------------------------------------------------------------
#define LED_PIN 8

// --------------------------------------------------------------------------
// Funk
// --------------------------------------------------------------------------
SX1278 radio = new Module(LORA_SS, LORA_DIO0, LORA_RST, RADIOLIB_NC);

// --------------------------------------------------------------------------
// JSON-Zeile an den Raspberry Pi senden. body ist das fertige JSON-Objekt,
// z. B. {"temp":21.5,...}. src ist der Knotenname aus dem LoRa-Paket.
// --------------------------------------------------------------------------
static void sendToPi(const char *src, const char *body) {
  PI_SERIAL.print("{\"src\":\"");
  PI_SERIAL.print(src);
  PI_SERIAL.print("\",\"data\":");
  PI_SERIAL.print(body);
  PI_SERIAL.println("}");
  PI_SERIAL.flush();
}

static void sendSysToPi(const char *msg) {
  PI_SERIAL.print("{\"sys\":\"");
  PI_SERIAL.print(msg);
  PI_SERIAL.println("\"}");
  PI_SERIAL.flush();
}

// --------------------------------------------------------------------------
// Interrupt-Handler: meldet ankommende LoRa-Pakete
// --------------------------------------------------------------------------
static volatile bool rxFlag = false;

static void onRxPkt() {
  rxFlag = true;
}

// --------------------------------------------------------------------------
// LoRa-Empfänger initialisieren und in den Endlos-Empfang wechseln
// --------------------------------------------------------------------------
static bool initLora() {
  Serial.println("Initialisiere LoRa-Modul...");
  int state = radio.begin(LORA_FREQ_MHZ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SYNC, LORA_POWER, LORA_PREAMBLE);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa-Fehler bei begin(): %d. Verdrahtung prüfen!\n", state);
    return false;
  }

  // Empfangsbereit: DIO0-Interrupt beim Start eines gültigen Pakets
  radio.setPacketReceivedAction(onRxPkt);
  state = radio.startReceive(LORA_RX_TIMEOUT);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa-Fehler bei startReceive(): %d\n", state);
    return false;
  }

  Serial.printf("LoRa empfangsbereit: %.1f MHz, SF%d, BW%.0f kHz, CR 4/%d\n",
                LORA_FREQ_MHZ, (int)LORA_SF, LORA_BW, (int)LORA_CR);
  return true;
}

// --------------------------------------------------------------------------
// Ein empfangenes Paket auswerten und an den Pi weiterreichen.
// Paketformat vom Waldknoten: "<Knotenname> <JSON>"
// --------------------------------------------------------------------------
static void processPacket() {
  rxFlag = false;

  size_t len = radio.getPacketLength();
  if (len == 0 || len > 255) {
    Serial.println("Ungültige Paketlänge.");
    radio.startReceive(LORA_RX_TIMEOUT);
    return;
  }

  char buf[256];
  int state = radio.readData((uint8_t *)buf, sizeof(buf));
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Empfangsfehler: %d\n", state);
    radio.startReceive(LORA_RX_TIMEOUT);
    return;
  }

  buf[len] = '\0';  // readData liefert keinen Null-Terminator

  // Knotenname und JSON trennen (Leerzeichen als Trennzeichen)
  char *sep = strchr(buf, ' ');
  if (sep == NULL) {
    Serial.printf("Paket ohne Knotenname: %s\n", buf);
    radio.startReceive(LORA_RX_TIMEOUT);
    return;
  }
  *sep = '\0';
  const char *src = buf;
  const char *body = sep + 1;

  Serial.printf("OK   %-10s %s\n", src, body);
  sendToPi(src, body);

  // Direkt wieder in den Empfang
  radio.startReceive(LORA_RX_TIMEOUT);
}

// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== ESP-ThreadReceiver-LoRa: Zentrale ===");

  // RGB-LED am SuperMini abschalten (GPIO8)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // UART zum Raspberry Pi (JSON-Zeilen, 115200 baud)
  PI_SERIAL.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  delay(100);
  sendSysToPi("ESP-ThreadReceiver-LoRa boot");

  if (!initLora()) {
    sendSysToPi("LoRa init failed");
  }
}

// --------------------------------------------------------------------------
void loop() {
  if (rxFlag) {
    processPacket();
  }
  delay(10);
}