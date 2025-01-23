# ORCA OPEN ROCKET TRACKER 

Open source, low cost, long range (~10km) GPS tracker for rocketry. Uses a 4 layer board designed in Autodesk Fusion. Designed to fit in the smallest rockets at only 22mm x 40mm board size (plus antenna and battery you use). 

No radio licence required within Australia. A second board without the GPS or barometer installed can be used as a low base station to log position data via a serial input on a laptop. 

![image](https://github.com/user-attachments/assets/c7ade0d9-c0fa-4642-a546-5e5b196535c9)

CORE: WIO-E5 LoRa MODULE 
* CPU architecture: STM32WL Cortex M4 32 bit @ 48MHz
* CPU flash memory: 256KB 
* SRAM: 64KB
* LoRa radio: Semtech SX1262 (915 MHz)

GPS: UBlox MAX-M10Q
* Max Altitude: 80,000m
* Max G: ≤4
* Max Velocity: 500m/s
* Velocity Accuracy: 0.05m/s
* Heading Accuracy: 0.3 degrees

Altitude Pressure Sensor: TE Connectivity MS5611
* Operating Pressure: 1kPa ~ 120kPa
* Accuracy: ±0.15kPa
* Operating Temperature: -40°C ~ 85°C

Power systems:
3.3V battery with reverse polarity and ESD protection.
USB-C battery charging.

To do:
* Implement in Zephyr RTOS.
* Design low cost base station
  * SD card logger
  * GPS
  * Compass
  * LCD screen to output GPS coordinates and direction to rocket position
  * Bluetooth to connect with mobile phone to transmit GPS coordinates.
* Develop method for plotting GPS position on mobile maps.

## Programming the Wio-E5 STM32WLE5JC Module

### Connecting the STLINK debugger

Connect debug pins to (STLINK-V3MINIE)[https://www.st.com/resource/en/user_manual/um2910-stlinkv3minie-debuggerprogrammer-tiny-probe-for-stm32-microcontrollers-stmicroelectronics.pdf]

Use a OPENLINK connector to connect the STLINK to the Rocket tracker. The connection is the same as the Wio-E5 mini developer board. 

| Wio-E5 STM32WLE5JC | STLINK | 
|---|---|
| DIO | SWDIO / TMS / STDC14 PIN 4 / MIPI10 PIN 2 |
| CLK | SWCLK / CLK / STDC14 PIN 6 / MIPI10 PIN 4 |
| GND | GND / GND / STDC14 PIN 7 / MIPI10 PIN 5 |
| RST | RST / T_NRST / STDC14 PIN 12 / MIPI10 PIN 10 |
| 3V3 | VCC / T_VCC / STDC14 PIN 3 / MIPI10 PIN 1 |

Power the STLINK with a USB-C cable. 

Power the Wio-E5 STM32WLE5JC Module with another USB-C cable.

### Change read only protection to allow flashing with custom program

The first time you program a Wio-E5 STM32WLE5JC Module you need to remove Read-out Protection (RDP) first with STM32Cube Programmer.

1. Download STM32Cube Programmer and run
2. Connect the STLINK debugger 
3. Choose ST-LINK and set Reset Mode -> hardware reset -> Connect
4. Open the OB tab -> Change RDP to AA -> Apply

Once saved, this doesn't need to be done again.

## Power and Charging 

The board can be powered with a 3.3V battery. The 3.3V battery can be charged via a USB-C cable. 

### LED power status 
* A red LED will show when powered.
* An orange LED will show when charging.
* The orange LED will turn off when fully charged and connected via a USB-C cable.




