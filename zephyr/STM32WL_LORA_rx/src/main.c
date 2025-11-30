#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/sys/printk.h>
#include <string.h>

/* Configuration Constants */
#define RX_PACKET_SIZE 25

/* LoRa Configuration from Kconfig */
#define LORA_FREQUENCY      CONFIG_LORA_FREQUENCY
#define LORA_BANDWIDTH      CONFIG_LORA_BANDWIDTH
#define LORA_SPREADING_FACTOR CONFIG_LORA_SPREADING_FACTOR
#define LORA_TX_POWER       CONFIG_LORA_TX_POWER

/* Device pointer */
static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* Data packet structure - must match transmitter */
union DataPacket {
    struct {
        int32_t latitude;     /* degrees * 1e7 */
        int32_t longitude;    /* degrees * 1e7 */
        int32_t altitude;     /* meters */
        uint32_t timeMs;      /* HHMMSS or milliseconds */
        int32_t pressure;     /* Pascals */
        int16_t temperature;  /* centi-degrees Celsius */
        uint8_t satellites;   /* number of satellites */
        uint8_t status;       /* status flags */
        uint8_t checksum;     /* XOR checksum */
    } fields;
    uint8_t buffer[RX_PACKET_SIZE];
};

/* Map Kconfig bandwidth to Zephyr LoRa enum */
static enum lora_signal_bandwidth get_bandwidth(void)
{
    switch (LORA_BANDWIDTH) {
    case 125:
        return BW_125_KHZ;
    case 250:
        return BW_250_KHZ;
    case 500:
        return BW_500_KHZ;
    default:
        return BW_125_KHZ;
    }
}

/* Map Kconfig spreading factor to Zephyr LoRa enum */
static enum lora_datarate get_spreading_factor(void)
{
    switch (LORA_SPREADING_FACTOR) {
    case 6:
        return SF_6;
    case 7:
        return SF_7;
    case 8:
        return SF_8;
    case 9:
        return SF_9;
    case 10:
        return SF_10;
    case 11:
        return SF_11;
    case 12:
        return SF_12;
    default:
        return SF_12;
    }
}

/* Verify packet checksum */
static bool verify_checksum(const uint8_t *data, size_t len)
{
    if (len < 2) {
        return false;
    }

    uint8_t calculated = 0;
    for (size_t i = 0; i < len - 1; i++) {
        calculated ^= data[i];
    }
    return (calculated == data[len - 1]);
}

int main(void)
{
    union DataPacket rx_packet;
    int16_t rssi;
    int8_t snr;
    int ret;

    /* Check device readiness */
    if (!device_is_ready(lora_dev)) {
        printk("LoRa device not ready\n");
        return 1;
    }

    /* Configure LoRa radio for receive */
    struct lora_modem_config config = {
        .frequency = LORA_FREQUENCY,
        .bandwidth = get_bandwidth(),
        .datarate = get_spreading_factor(),
        .coding_rate = CR_4_5,
        .preamble_len = 8,
        .tx_power = LORA_TX_POWER,
        .tx = false,  /* RX mode */
        .public_network = false,
        .iq_inverted = false,
    };

    ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        printk("LoRa config failed: %d\n", ret);
        return 1;
    }

    /* Print startup info */
    printk("\n");
    printk("=== Open Rocket Tracker Base Station ===\n");
    printk("Frequency: %u Hz\n", LORA_FREQUENCY);
    printk("Bandwidth: %u kHz\n", LORA_BANDWIDTH);
    printk("Spreading Factor: SF%u\n", LORA_SPREADING_FACTOR);
    printk("Listening for packets...\n");
    printk("\n");

    /* Print CSV header */
    printk("sats,lat_deg,lon_deg,alt_m,time,pressure_Pa,temp_C,status,chksum,rssi_dBm,snr_dB\n");

    /* Main receive loop */
    while (1) {
        /* Clear buffer */
        memset(rx_packet.buffer, 0, RX_PACKET_SIZE);

        /* Blocking receive */
        ret = lora_recv(lora_dev, rx_packet.buffer, RX_PACKET_SIZE,
                        K_FOREVER, &rssi, &snr);

        if (ret < 0) {
            printk("LoRa receive error: %d\n", ret);
            continue;
        }

        if (ret != RX_PACKET_SIZE) {
            printk("Unexpected packet size: %d\n", ret);
            continue;
        }

        /* Verify checksum */
        if (!verify_checksum(rx_packet.buffer, RX_PACKET_SIZE)) {
            printk("Invalid checksum, packet discarded\n");
            continue;
        }

        /* Output CSV format matching Arduino receiver */
        /* sats,lat_deg,lon_deg,alt_m,time,pressure_Pa,temp_C,status,chksum,rssi_dBm,snr_dB */

        /* Calculate floating point values */
        int32_t lat_int = rx_packet.fields.latitude / 10000000;
        int32_t lat_frac = (rx_packet.fields.latitude % 10000000);
        if (lat_frac < 0) lat_frac = -lat_frac;

        int32_t lon_int = rx_packet.fields.longitude / 10000000;
        int32_t lon_frac = (rx_packet.fields.longitude % 10000000);
        if (lon_frac < 0) lon_frac = -lon_frac;

        int32_t temp_int = rx_packet.fields.temperature / 100;
        int32_t temp_frac = rx_packet.fields.temperature % 100;
        if (temp_frac < 0) temp_frac = -temp_frac;

        printk("%u,%d.%06d,%d.%06d,%d,%u,%d,%d.%02d,0x%02X,0x%02X,%d,%d\n",
               rx_packet.fields.satellites,
               lat_int, (int)(lat_frac / 10),  /* 6 decimal places */
               lon_int, (int)(lon_frac / 10),  /* 6 decimal places */
               rx_packet.fields.altitude,
               rx_packet.fields.timeMs,
               rx_packet.fields.pressure,
               temp_int, (int)temp_frac,
               rx_packet.fields.status,
               rx_packet.fields.checksum,
               rssi,
               snr);
    }

    return 0;
}
