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

## To do

* ~~Implement in Zephyr RTOS.~~ ✓ In progress
* Implement base station receiver in Zephyr RTOS
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

## Zephyr RTOS Firmware

The Open Rocket Tracker uses [Zephyr RTOS](https://www.zephyrproject.org/) for both the rocket transmitter and base station receiver.

### Prerequisites

#### 1. Install Zephyr RTOS and SDK

Follow the official [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) to:

1. Install host dependencies
2. Install the Zephyr SDK
3. Create a workspace and initialize Zephyr with `west`

#### 2. Install Additional Tools

After completing the Zephyr setup, install build and debug tools:

```bash
sudo apt install ninja-build openocd stlink-tools
```

#### 3. Set Up ST-LINK udev Rules (Linux)

Required for non-root access to ST-LINK debuggers:

```bash
sudo tee /etc/udev/rules.d/99-stlink.rules << 'EOF'
# ST-LINK V2
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", GROUP="plugdev"
# ST-LINK V2-1
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", GROUP="plugdev"
# ST-LINK V3
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374d", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3753", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3754", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Add yourself to the plugdev group:

```bash
sudo usermod -aG plugdev $USER
# Log out and back in for group changes to take effect
```

### Environment Setup

Before building, activate the Zephyr environment:

```bash
source ~/zephyrproject/.venv/bin/activate
```

If `ZEPHYR_BASE` is not automatically set:

```bash
export ZEPHYR_BASE=~/zephyrproject/zephyr
```

### Building the Rocket Transmitter (STM32WL_LORA_tx)

```bash
cd zephyr/STM32WL_LORA_tx
west build -b open_rocket_tracker -p always -- -DBOARD_ROOT=$(pwd)
```

### Building the Base Station Receiver (STM32WL_LORA_rx)

```bash
cd zephyr/STM32WL_LORA_rx
west build -b open_rocket_tracker -p always -- -DBOARD_ROOT=$(pwd)
```

### Flashing

```bash
west flash
```

Or using OpenOCD directly:

```bash
openocd -f boards/open_rocket_tracker/support/openocd.cfg \
  -c "program build/zephyr/zephyr.elf verify reset exit"
```

### Debugging

```bash
west debug
```

See [zephyr/STM32WL_LORA_tx/README.md](zephyr/STM32WL_LORA_tx/README.md) for more details and troubleshooting.

## LoRa Configuration

Both the transmitter and receiver can be configured by editing their `prj.conf` files. **Settings must match on both TX and RX for communication to work.**

### Configuration Options

Edit `zephyr/STM32WL_LORA_tx/prj.conf` or `zephyr/STM32WL_LORA_rx/prj.conf`:

```conf
# LoRa Configuration
CONFIG_LORA_FREQUENCY=915000000      # Frequency in Hz (915 MHz for Australia/US)
CONFIG_LORA_BANDWIDTH=125            # Bandwidth: 125, 250, or 500 kHz
CONFIG_LORA_SPREADING_FACTOR=12      # SF6-SF12 (higher = longer range, slower)
CONFIG_LORA_TX_POWER=14              # TX power: 2-22 dBm
CONFIG_LORA_TX_INTERVAL_MS=2000      # TX only: interval between transmissions
```

### Spreading Factor Trade-offs

| SF   | Range    | Data Rate  | Min TX Interval | Use Case                    |
|------|----------|------------|-----------------|------------------------------|
| SF12 | Maximum  | ~250 bps   | ~1000 ms        | Long range, low update rate  |
| SF10 | Long     | ~980 bps   | ~500 ms         | Good range, moderate updates |
| SF8  | Medium   | ~3125 bps  | ~250 ms         | Balanced range/speed         |
| SF7  | Shorter  | ~5470 bps  | ~200 ms         | Fast updates (4-5 Hz)        |

### Example Configurations

**Maximum Range (default):**

```conf
CONFIG_LORA_SPREADING_FACTOR=12
CONFIG_LORA_TX_INTERVAL_MS=2000
```

**Fast Updates (4-5 Hz):**

```conf
CONFIG_LORA_SPREADING_FACTOR=7
CONFIG_LORA_TX_INTERVAL_MS=200
```

**Balanced:**

```conf
CONFIG_LORA_SPREADING_FACTOR=10
CONFIG_LORA_TX_INTERVAL_MS=500
```

After changing settings, rebuild and reflash both TX and RX:

```bash
cd zephyr/STM32WL_LORA_tx && west build -b open_rocket_tracker -p always -- -DBOARD_ROOT=$(pwd) && west flash
cd zephyr/STM32WL_LORA_rx && west build -b open_rocket_tracker -p always -- -DBOARD_ROOT=$(pwd) && west flash
```

## Telemetry Data Format

### LoRa Packet Structure (25 bytes)

| Field       | Type     | Size    | Units / Description                          |
|-------------|----------|---------|----------------------------------------------|
| latitude    | int32_t  | 4 bytes | Degrees × 10⁷ (e.g., -274678530 = -27.467853°) |
| longitude   | int32_t  | 4 bytes | Degrees × 10⁷ (e.g., 1530279210 = 153.027921°) |
| altitude    | int32_t  | 4 bytes | Meters (from GPS)                            |
| timeMs      | uint32_t | 4 bytes | HHMMSS format (e.g., 143052 = 14:30:52 UTC)  |
| pressure    | int32_t  | 4 bytes | Pascals (e.g., 101325 Pa = 1013.25 hPa)      |
| temperature | int16_t  | 2 bytes | Centi-degrees Celsius (e.g., 2350 = 23.50°C) |
| satellites  | uint8_t  | 1 byte  | Number of GPS satellites in view             |
| status      | uint8_t  | 1 byte  | Status flags                                 |
| checksum    | uint8_t  | 1 byte  | XOR checksum of all preceding bytes          |

### Serial Output (CSV)

The base station outputs CSV data at 115200 baud:

```text
sats,lat_deg,lon_deg,alt_m,time_ms,pressure_Pa,temp_C,status_hex,checksum_hex,rssi_dBm,snr_dB
```

Example:

```text
8,-27.467853,153.027921,45,143052,101325,23.50,0x01,0xA5,-85,7
```

| Column       | Type   | Description                                    |
|--------------|--------|------------------------------------------------|
| sats         | int    | Number of GPS satellites                       |
| lat_deg      | float  | Latitude in decimal degrees                    |
| lon_deg      | float  | Longitude in decimal degrees                   |
| alt_m        | int    | GPS altitude in meters                         |
| time_ms      | int    | UTC time as HHMMSS                             |
| pressure_Pa  | int    | Barometric pressure in Pascals                 |
| temp_C       | float  | Temperature in degrees Celsius                 |
| status_hex   | hex    | Status flags                                   |
| checksum_hex | hex    | Packet checksum                                |
| rssi_dBm     | int    | Received signal strength (dBm)                 |
| snr_dB       | int    | Signal-to-noise ratio (dB)                     |

### Calculating Barometric Altitude (AGL)

For Above Ground Level altitude, capture the pressure at launch (P₀) when starting your mission, then calculate:

```text
altitude_agl = 44330 × (1 - (P / P₀)^0.1903)
```

Where:

- P = current pressure (Pa)
- P₀ = launch pad pressure (Pa)
- Result is in meters above launch pad

This approach requires no internet connection and gives altitude relative to your launch site.

## Power and Charging 

The board can be powered with a 3.3V battery. The 3.3V battery can be charged via a USB-C cable. 

### LED power status 
* A red LED will show when powered.
* An orange LED will show when charging.
* The orange LED will turn off when fully charged and connected via a USB-C cable.

## Acknowledgments

- [@vinn-ie](https://github.com/vinn-ie) - Initial Zephyr board configuration and build setup, plus just a great human and electronics superstar :rocket:
