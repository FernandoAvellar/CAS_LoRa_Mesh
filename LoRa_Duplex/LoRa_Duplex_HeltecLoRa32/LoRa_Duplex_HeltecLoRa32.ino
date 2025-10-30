// LoRa32_bidirectional_counter.ino
#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "Wire.h"
#include "HT_SSD1306Wire.h"

#define RF_FREQUENCY        915000000
#define TX_OUTPUT_POWER     2
#define LORA_BANDWIDTH      2   // 500 kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE     1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define LORA_TIMEOUT_TRANSMISSION 3000
#define BUFFER_SIZE         64

// message types
#define MSG_TYPE_GPS_BIN    0x01
#define MSG_TYPE_COUNT_BIN  0x02
#define MSG_TYPE_TEXT       0x03

char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

bool lora_idle = true;
unsigned long lastSend = 0;
unsigned long lastDisplayUpdate = 0;
uint32_t txCounter = 0;

void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void VextON(void);

void setup() {
  memset(rxpacket, 0, sizeof(rxpacket));

  Serial.begin(115200);
  Serial.println("Iniciando...");
  delay(100);

  VextON();
  delay(100);
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Init LoRa ...");
  display.display();

  delay(100);

  Serial.println("Iniciando LoRa ...");
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Callbacks setup
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetPublicNetwork(true);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, LORA_TIMEOUT_TRANSMISSION);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                  0, LORA_FIX_LENGTH_PAYLOAD_ON,
                  0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  //Start in receive mode and allow the TX loop to run when time arrives
  lora_idle = true;
  Radio.Rx(0);
}

void loop() {

  //Update display periodically (Counter status + last RX stored in rxpacket)
  if (millis() - lastDisplayUpdate > 2000) 
  {
    lastDisplayUpdate = millis();

    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "LoRa Node (Cnt)");
    display.drawString(0, 24, String("TX CNT: ") + String(txCounter));
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 48, rxpacket);
    display.display();
  }

  //Transmit binary counter at every 7 seconds if idle
  if (millis() - lastSend > 7000 && lora_idle)
  {
    lastSend = millis();
    lora_idle = false; 
    txCounter++;

    uint8_t payload[8];
    payload[0] = MSG_TYPE_COUNT_BIN;
    memcpy(&payload[1], &txCounter, 4);
    uint8_t len = 5;

    Serial.printf("Enviando COUNT %lu\r\n", (unsigned long)txCounter);
    Radio.Send(payload, len);
  }

  Radio.IrqProcess();
}

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void OnTxDone(void) {
  Serial.println("TX done.");
  lora_idle = true;    // now radio is free for next TX
  Radio.Rx(0);         // re-arm continuous RX immediately
}

void OnTxTimeout(void) {
  Serial.println("TX Timeout");
  lora_idle = true;
  Radio.Rx(0);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) 
{
  //Store a human-readable summary into rxpacket for display
  Serial.println("Packet received...");
  lora_idle = false;
  memset(rxpacket, 0, sizeof(rxpacket));

  if (payload[0] == MSG_TYPE_GPS_BIN && size >= 11) 
  {
    int32_t lat_i=0, lon_i=0; 
    uint8_t sats=0, hdop10=0;
    memcpy(&lat_i, &payload[1], 4);
    memcpy(&lon_i, &payload[5], 4);
    sats = payload[9];
    hdop10 = payload[10];
    double lat = lat_i / 1e6;
    double lon = lon_i / 1e6;
    snprintf(rxpacket, BUFFER_SIZE, "LAT:%.5f LON:%.5f S:%d H:%.1f", lat, lon, sats, hdop10/10.0);
  } 
  else if (payload[0] == MSG_TYPE_COUNT_BIN && size >= 5) 
  {
    uint32_t cnt = 0;
    memcpy(&cnt, &payload[1], 4);
    snprintf(rxpacket, BUFFER_SIZE, "COUNT:%lu", (unsigned long)cnt);
  } 
  else
  {
    size_t text_len = (size-1 < BUFFER_SIZE-1) ? (size-1) : (BUFFER_SIZE-1);
    memcpy(rxpacket, &payload[1], text_len);
    rxpacket[text_len] = '\0';
  }

  Serial.printf("RX: %s | RSSI:%d SNR:%d\n", rxpacket, rssi, snr);

  lora_idle = true;
  Radio.Rx(0);
}
