#include <Arduino_Extended.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RadioLib.h>
#include <lib_xcore>
#include "nova_pin_def.h"

// LoRa State
enum class LoRaState
{
  IDLE = 0,
  TRANSMITTING,
  RECEIVING
};

// LoRa Parameters
constexpr struct
{
  float center_freq = 921.500'000f; // MHz
  float bandwidth = 125.f;          // kHz
  uint8_t spreading_factor = 9;     // SF: 6 to 12
  uint8_t coding_rate = 8;          // CR: 5 to 8
  uint8_t sync_word = 0x12;         // Private SX1262
  int8_t power = 22;                // up to 22 dBm for SX1262
  uint16_t preamble_length = 16;
} params;

// LoRa Pins
constexpr uint32_t NSS_LORA = PA4;
constexpr uint32_t LORA_RXEN = PB13;
constexpr uint32_t LORA_TXEN = PB12;

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

// LoRa SX1262
SPISettings lora_spi_settings(6'000'000, MSBFIRST, SPI_MODE0);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

String tx_data;
int status_lora;
volatile bool rx_flag = false;
volatile bool tx_flag = false;
volatile LoRaState lora_state = LoRaState::IDLE;
uint32_t lora_tx_end_time;
float lora_rssi;

static bool state;

extern void set_rxflag();

void setup()
{
  Serial.begin();
  delay(1000);

  spi1.begin();
  // LoRa
  int lora_state = lora.begin(params.center_freq,
                              params.bandwidth,
                              params.spreading_factor,
                              params.coding_rate,
                              params.sync_word,
                              params.power,
                              params.preamble_length,
                              0,
                              false);
  state = state || lora.explicitHeader();
  state = state || lora.setCRC(true);
  state = state || lora.autoLDRO();

  lora.setPacketReceivedAction(set_rxflag);

  Serial.printf("SX1262 LoRa %s\n", state == RADIOLIB_ERR_NONE ? "SUCCESS" : "FAILED");
  if (state != RADIOLIB_ERR_NONE)
    Serial.printf("Initialization failed! Error: %d\n", lora_state);

  float freq;
  EEPROM.get<float>(0, freq);
  lora.setFrequency(freq);
  Serial.print("Set frequency to ");
  Serial.print(freq);
  Serial.println("MHz");
}

void loop()
{
  // Tx On Command
  if (Serial.available())
  {
    tx_data = "";
    while (Serial.available())
      tx_data += static_cast<char>(Serial.read());

    if (tx_data.substring(0, 5) == "freq ")
    {
      String freqStr = tx_data.substring(5);
      freqStr.trim();
      const float freq = freqStr.toFloat();
      lora.setFrequency(freq);
      EEPROM.put<float>(0, freq);
      Serial.print("Set frequency to ");
      Serial.print(freq);
      Serial.println("MHz");
    }
    else
    {
      lora_state = LoRaState::TRANSMITTING;
      lora.startTransmit(tx_data);
      lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(tx_data.length())) / 1000;
      Serial.println("[TRANSMITTING...]");
    }
  }

  // Set Tx Done
  if (millis() > lora_tx_end_time &&
      lora_state != LoRaState::RECEIVING)
  {
    tx_flag = true;
    lora_state = LoRaState::RECEIVING;
    lora.startReceive();
    Serial.println("[RECEIVING...]");
  }

  // Set Rx Done
  if (lora.getPacketLength() > 0 &&
      lora.getRSSI() != lora_rssi)
  {
    rx_flag = true;
  }

  // On Transmit
  if (tx_flag)
  {
    Serial.print("[TRANSMITTED] ");
    Serial.println(tx_data);
    tx_flag = false;
  }

  // On Receive
  if (rx_flag)
  {
    String s;
    lora.readData(s);
    lora_rssi = lora.getRSSI();
    Serial.print("RSSI: ");
    Serial.println(lora_rssi);
    Serial.print("[RECEIVED]    ");
    Serial.println(s);
    rx_flag = false;

    lora_state = LoRaState::RECEIVING;
    lora.startReceive();
    Serial.println("[RECEIVING...]");
  }
}

void set_rxflag()
{
    rx_flag = true;
}