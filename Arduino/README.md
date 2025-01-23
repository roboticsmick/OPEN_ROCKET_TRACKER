### Programming via Arduino

Add the following board URL (under File / Preferences / Additional board URLs):

```txt
https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
```
Under menu Tools/Board/Boards manager, search for STM32 and install 'STM32 MCU based boards'

Select the board: Tools -> Board -> STM32 MCU Based Boards -> LoRa board
Select the board number: Tools -> Boards part number -> Lora-E5-mini

The following libraries need to be added to flash the LORA_tx program in Arduino.

#include <RadioLib.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MS5x.h>

Select the port connected to the to the STLINK debugger.

Flash the LORA_tx.ino to the transmitter, and the LORA_rx.ino to the base station reciever. 

### Debug Serial output

By default the LORA_tx program prints the sensor and transmit data to the serial monitor with a BAUD rate of 115200. Once you have the transmitter and reciever set up correctly set the debug to false (line 24 of LORA_tx.ino):

```cpp
// -------------------
// Constants & Config
// -------------------
#define TX_PACKET_SIZE 25
#define TX_INTERVAL    500       // Transmit interval in ms (2 Hz)
bool debugEnabled    = false;    // Set to false to disable debug prints via serial
```
