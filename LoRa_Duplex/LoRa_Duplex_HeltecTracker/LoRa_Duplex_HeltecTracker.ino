// Tracker_GPS_bidirectional.ino
#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"
#include "HT_st7735.h"

TinyGPSPlus GPS;
HT_st7735 st7735;

#define VGNSS_CTRL 3

#define RF_FREQUENCY        915000000 // Hz
#define TX_OUTPUT_POWER     2         // dBm (reduz cobertura)
#define LORA_BANDWIDTH      2         // 0:125kHz, 1:250kHz, 2:500kHz (Melhora AirTime mas reduz cobertura)
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE     1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define LORA_TIMEOUT_TRANSMISSION 3000
#define BUFFER_SIZE         64

// binary message types
#define MSG_TYPE_GPS_BIN    0x01
#define MSG_TYPE_COUNT_BIN  0x02
#define MSG_TYPE_TEXT       0x03

char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

bool lora_idle = true;
unsigned long lastSend = 0;
unsigned long lastDisplayUpdate = 0;

void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

void setup() {
  memset(rxpacket, 0, sizeof(rxpacket));

  st7735.st7735_init();
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0, "Iniciando...");

  Serial.begin(115200);
  Serial.println("Iniciando...");

  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  Serial1.begin(115200, SERIAL_8N1, 33, 34);
  Serial.println("Iniciando GPS ...");
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

  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0, "Setup concluido.");
  Serial.println("Setup finalizado!");

  //Start in receive mode and allow the TX loop to run when time arrives
  lora_idle = true;
  Radio.Rx(0);
}

void loop() {
  //Read GPS stream
  while (Serial1.available() > 0) 
  {
    GPS.encode(Serial1.read());
  }

  //Update display periodically (status + last RX stored in rxpacket)
  if (millis() - lastDisplayUpdate > 2000) 
  {
    lastDisplayUpdate = millis();

    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, "Lora Node GPS");

    if (GPS.location.isValid() && GPS.satellites.value() > 3) 
    {
      String gpsStatus = String("GPS: OK ") + String(GPS.satellites.value()) + "sat";
      st7735.st7735_write_str(0, 24, gpsStatus.c_str());
    } else 
    {
      st7735.st7735_write_str(0, 24, "GPS: NO FIX");
    }
    
    st7735.st7735_write_str(0, 60, rxpacket);
  }

  // Transmit GPS binary payload every 5s if idle
  if (millis() - lastSend > 5000 && lora_idle) 
  {
    lastSend = millis();
    lora_idle = false;

    uint8_t payload[16];
    uint8_t len = 0;

    if (GPS.location.isValid() && GPS.satellites.value() > 3) 
    {
      // pack as: [1-byte type][4-bytes lat_i32][4-bytes lon_i32][1-byte sats][1-byte hdop_scaled]
      int32_t lat_i = (int32_t)round(GPS.location.lat() * 1e6);
      int32_t lon_i = (int32_t)round(GPS.location.lng() * 1e6);
      uint8_t sats = (uint8_t)min((uint32_t)255, GPS.satellites.value());
      uint8_t hdop10 = (uint8_t)min(255, (int)round((GPS.hdop.isValid()? (GPS.hdop.hdop()): 0.0) * 10.0));

      payload[0] = MSG_TYPE_GPS_BIN;
      memcpy(&payload[1], &lat_i, 4);
      memcpy(&payload[5], &lon_i, 4);
      payload[9] = sats;
      payload[10] = hdop10;
      len = 11;
    } 
    else 
    {
      const char *txt = "Sem fix GPS";
      payload[0] = MSG_TYPE_TEXT;
      strncpy((char*)&payload[1], txt, sizeof(payload)-2);
      len = 1 + strlen(txt);
    }

    Serial.printf("Enviando (len %d) ...\r\n", len);
    Radio.Send((uint8_t*)payload, len);
  }

  Radio.IrqProcess();
}

void OnTxDone(void) 
{
  Serial.println("TX done.");
  lora_idle = true;    // now radio is free for next TX
  Radio.Rx(0);         // re-arm continuous RX immediately
}

void OnTxTimeout(void) 
{
  Serial.println("TX timeout...");
  lora_idle = true;
  Radio.Rx(0);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) 
{
  //Store a human-readable summary into rxpacket for display
  Serial.println("Packet received...");
  lora_idle = false;
  memset(rxpacket, 0, sizeof(rxpacket));

  if (payload[0] == MSG_TYPE_COUNT_BIN && size >= 5) 
  {
    uint32_t cnt = 0;
    memcpy(&cnt, &payload[1], 4);
    snprintf(rxpacket, BUFFER_SIZE, "COUNT:%lu", (unsigned long)cnt);
  } 
  else if (payload[0] == MSG_TYPE_GPS_BIN && size >= 11) 
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
  else 
  {
    size_t text_len = (size-1 < BUFFER_SIZE-1) ? (size-1) : (BUFFER_SIZE-1);
    memcpy(rxpacket, &payload[1], text_len);
    rxpacket[text_len] = '\0';
  }

  Serial.printf("RX done rssi=%d snr=%d -> %s\r\n", rssi, snr, rxpacket);

  lora_idle = true;
  Radio.Rx(0);
}
