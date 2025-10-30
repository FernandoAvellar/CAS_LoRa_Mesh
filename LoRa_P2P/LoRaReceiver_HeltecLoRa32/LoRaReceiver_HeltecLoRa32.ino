#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <Wire.h>               
#include "HT_SSD1306Wire.h"

#define RF_FREQUENCY                                915000000
#define LORA_BANDWIDTH                              0  
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Tamanho máximo do payload

// --- LED onboard ---
#define LED_PIN 35

static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;

bool lora_idle = true;
unsigned long lastBlink = 0;
bool ledState = false;

// --- Callback ---
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Receptor LoRa + OLED ---");

  // Inicializa LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Inicializa display
  VextON();
  delay(100);
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 18, "Sem GPS fix");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 48, "Aguardando...");
  display.display();

  // Inicializa rádio
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);

  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
}


void loop()
{
  if(lora_idle)
  {
    lora_idle = false;
    Radio.Rx(0); //modo de recepção contínua
  }

  Radio.IrqProcess( );

  if (ledState && millis() - lastBlink > 200) 
  {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
  }
}

// --- Quando receber pacote ---
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {

  // copia o pacote recebido para string terminada em '\0'
  size = (size < BUFFER_SIZE - 1) ? size : BUFFER_SIZE - 1;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';

  Radio.Sleep();

  // Feedback LED
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
  lastBlink = millis();

  // Serial log
  Serial.printf("📡 Pacote recebido: %s\r\n", rxpacket);
  Serial.printf("RSSI: %d dBm | SNR: %d dB | Len: %d bytes\r\n", rssi, snr, size);
  Serial.println("-------------------------------");

  // --- Atualiza OLED ---
  display.clear();
  
  double latVal = 0.0, lonVal = 0.0;
  bool hasCoord = false;

  if (strstr(rxpacket, "LAT:") && strstr(rxpacket, "LON:")) 
  {
    if (sscanf(rxpacket, "LAT:%lf,LON:%lf", &latVal, &lonVal) == 2) 
    {
      hasCoord = true;
    }
  }

  if (hasCoord) 
  {
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    String rssiVal = "RSSI:" + String(rssi) + " dBm";
    display.drawString(0, 16, rssiVal);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 38, "Lat:" + String(latVal, 6));
    display.drawString(0, 50, "Long:" + String(lonVal, 6));
  } 
  else 
  {
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 18, "Sem GPS fix");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 48, "Aguardando...");
  }

  display.display();

  lora_idle = true;
}