#include "Arduino.h"
#include "HT_st7735.h"
#include "LoRaWan_APP.h"
#include "HT_TinyGPS++.h"

TinyGPSPlus GPS;
HT_st7735 st7735;

#define VGNSS_CTRL 3

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             2        // dBm
#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define BUFFER_SIZE                                 64 // Define the payload size here

char txpacket[BUFFER_SIZE];
bool lora_idle=true;
unsigned long lastSend = 0;
unsigned long lastDisplayUpdate = 0;

static RadioEvents_t RadioEvents;

void OnTxDone( void );
void OnTxTimeout( void );

void setup() 
{
  //inicia display
  st7735.st7735_init();
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0, "Iniciando...");

  // Serial de debug
  Serial.begin(115200);
  Serial.println("Iniciando setup...");

  // Inicializa serial do GPS
  pinMode(VGNSS_CTRL,OUTPUT);
  digitalWrite(VGNSS_CTRL,HIGH);
  Serial1.begin(115200,SERIAL_8N1,33,34);
  Serial.println("LoRa + GPS TX Start");
  Serial.println("Ambas seriais iniciadas...");

  delay(1000);

  // Inicializa LoRa
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0, "Setup concluido.");
  Serial.println("Setup finalizado!"); 
}


void loop() 
{
  // Atualiza dados do GPS continuamente
  while (Serial1.available() > 0) 
  {
    GPS.encode(Serial1.read());
  }

  if (millis() - lastDisplayUpdate > 5000) 
  {
    lastDisplayUpdate = millis();

    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, "LoRa TX");

    if (GPS.location.isValid()) 
    {
      String time_str = String(GPS.time.hour()) + ":" + String(GPS.time.minute()) + ":" + String(GPS.time.second());
      String latitude = "LAT: " + String(GPS.location.lat(), 6);
      String longitude = "LON: " + String(GPS.location.lng(), 6);

      st7735.st7735_write_str(0, 20, time_str);
      st7735.st7735_write_str(0, 40, latitude);
      st7735.st7735_write_str(0, 60, longitude);
    } 
    else 
    {
      st7735.st7735_write_str(0, 40, "Procurando sat..........");
    }
  }

  
  // Quando LoRa estiver livre, envia dados a cada 5 segundos
  if (millis() - lastSend > 5000 && lora_idle) {
    lora_idle = false;
    lastSend = millis();

    if (GPS.location.isValid()) 
    {
      double lat = GPS.location.lat();
      double lon = GPS.location.lng();
      int sat = GPS.satellites.value();
      double hdop = GPS.hdop.value() / 100.0;

      snprintf(txpacket, BUFFER_SIZE,
               "LAT:%.6f,LON:%.6f,SAT:%d,HDOP:%.1f",
               lat, lon, sat, hdop);
    } 
    else 
    {
      snprintf(txpacket, BUFFER_SIZE, "Sem fix GPS");
    }

    Serial.printf("\r\nEnviando pacote: %s\r\n", txpacket);
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
  }

  Radio.IrqProcess();

}

void OnTxDone( void )
{
	Serial.println("TX done.");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("Timeout......");
    lora_idle = true;
}