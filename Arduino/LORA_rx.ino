#include <RadioLib.h>

// -----------------------------------------------------------------------------
// Configure Radio
// -----------------------------------------------------------------------------
STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] = {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE, {LOW, LOW}},
    {STM32WLx::MODE_RX, {HIGH, LOW}},
    {STM32WLx::MODE_TX_HP, {LOW, HIGH}},
    END_OF_MODE_TABLE,
};

// -----------------------------------------------------------------------------
// Constants & Config
// -----------------------------------------------------------------------------
#define RX_PACKET_SIZE 25

// -----------------------------------------------------------------------------
// Data Packet Structure
// -----------------------------------------------------------------------------
union DataPacket
{
  struct
  {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    uint32_t timeMs;
    int32_t pressure;
    int16_t temperature;
    uint8_t satellites;
    uint8_t status;
    uint8_t checksum;
  } fields;
  uint8_t buffer[RX_PACKET_SIZE];
};

DataPacket rxPacket;
volatile bool receivedFlag = false;

// -----------------------------------------------------------------------------
// ISR for Reception Complete
// -----------------------------------------------------------------------------
void setFlag(void)
{
  receivedFlag = true;
}

// -----------------------------------------------------------------------------
// Verify Packet Checksum
// -----------------------------------------------------------------------------
bool verifyChecksum(uint8_t *data, size_t len)
{
  if (len < 2)
    return false;

  uint8_t calculatedChecksum = 0;
  for (size_t i = 0; i < len - 1; i++)
  {
    calculatedChecksum ^= data[i];
  }
  return (calculatedChecksum == data[len - 1]);
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("[Setup] Starting initialisation..."));

  // Radio switch configuration
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  // Radio initialisation
  Serial.print(F("[Radio] Initialising ... "));
  int state = radio.begin(915.0, 125.0, 12, 5);
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (1)
    {
      delay(10);
    }
  }

  // Set TCXO voltage
  state = radio.setTCXO(1.7);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("TCXO failed, code "));
    Serial.println(state);
    while (1)
    {
      delay(10);
    }
  }

  // Additional radio configuration
  radio.setSyncWord(0x12);
  radio.setPreambleLength(8);

  // Print some settings info (optional)
  Serial.println(F("Settings:"));
  Serial.println(F("Frequency: 915.0 MHz"));
  Serial.println(F("Bandwidth: 125.0 kHz"));
  Serial.println(F("Spreading Factor: 12"));
  Serial.println(F("Coding Rate: 5"));

  // Set interrupt handler
  radio.setDio1Action(setFlag);

  // Start receiving
  Serial.print(F("[Radio] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (1)
    {
      delay(10);
    }
  }
  Serial.println(F("Setup complete!\n"));

  // Print CSV header row (one-time)
  Serial.println(F("sats,lat_deg,long_deg,alt_m,time_ms,pressure_Pa,temp_C,status_hex,checksum_hex,rssi_dbm,snr_db"));
}

// -----------------------------------------------------------------------------
// Main Loop
// -----------------------------------------------------------------------------
void loop()
{
  if (receivedFlag)
  {
    receivedFlag = false;

    // Clear buffer and read
    memset(rxPacket.buffer, 0, RX_PACKET_SIZE);
    int state = radio.readData(rxPacket.buffer, RX_PACKET_SIZE);

    if (state == RADIOLIB_ERR_NONE)
    {
      float rssi = radio.getRSSI();
      float snr = radio.getSNR();

      // Verify checksum
      if (verifyChecksum(rxPacket.buffer, RX_PACKET_SIZE))
      {
        // Print CSV row
        // sats,lat_deg,long_deg,alt_m,time_ms,pressure_hPa,temp_C,status_hex,checksum_hex,rssi_dbm,snr_db
        Serial.print(rxPacket.fields.satellites);
        Serial.print(",");
        Serial.print(rxPacket.fields.latitude / 10000000.0, 6);
        Serial.print(",");
        Serial.print(rxPacket.fields.longitude / 10000000.0, 6);
        Serial.print(",");
        Serial.print(rxPacket.fields.altitude);
        Serial.print(",");
        Serial.print(rxPacket.fields.timeMs);
        Serial.print(",");
        Serial.print(rxPacket.fields.pressure);
        Serial.print(",");
        Serial.print(rxPacket.fields.temperature / 100.0, 2);
        Serial.print(",0x");
        Serial.print(rxPacket.fields.status, HEX);
        Serial.print(",0x");
        Serial.print(rxPacket.fields.checksum, HEX);
        Serial.print(",");
        Serial.print(rssi, 1);
        Serial.print(",");
        Serial.println(snr, 1);
      }
      else
      {
        // If you want to see a failure case, optionally print something here
        Serial.println(F("Invalid checksum; packet discarded."));
      }
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
      Serial.println(F("CRC error!"));
    }
    else
    {
      Serial.print(F("Reception failed, code "));
      Serial.println(state);
    }

    // Restart reception
    radio.startReceive();
  }
}
