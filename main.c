#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/sensor.h>   // <-- For MS5607
#include <zephyr/sys/printk.h>
#include <string.h>

/* Same packet size you used with RadioLib */
#define TX_PACKET_SIZE 25
#define TX_INTERVAL_MS 2000

/* LoRa device alias from your DTS */
static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

/* We'll find any MS5607 device that is "status = okay" in the DT. */
static const struct device *ms5607_dev = DEVICE_DT_GET_ANY(meas_ms5607);

/* Match your original struct layout */
union DataPacket {
    struct {
        int32_t latitude;     
        int32_t longitude;    
        int32_t altitude;     
        uint32_t timeMs;      
        int32_t pressure;     // Pressure in Pa
        int16_t temperature;  // Temperature in hundredths of a °C
        uint8_t satellites;   
        uint8_t status;       
        uint8_t checksum;     
    } fields;
    uint8_t buffer[TX_PACKET_SIZE];
};

int main(void)
{
    /**********************************
     * 1. Check LoRa device readiness
     **********************************/
    if (!device_is_ready(lora_dev)) {
        printk("LoRa device not ready\n");
        return 1;
    }

    /* Configure LoRa radio to match your original RadioLib settings */
    struct lora_modem_config config = {
        .frequency      = 922600000,  // 922.6 MHz
        .bandwidth      = BW_125_KHZ, // 125 kHz
        .datarate       = SF_12,      // spreading factor 12
        .coding_rate    = CR_4_5,     // coding rate 4/5
        .preamble_len   = 8,          // 8-symbol preamble
        .tx_power       = 13,         // 13 dBm
        .tx             = true,
        .public_network = false,
        .iq_inverted    = false,
    };

    int ret = lora_config(lora_dev, &config);
    if (ret < 0) {
        printk("LoRa config failed: %d\n", ret);
        return 2;
    }
    printk("LoRa configured.\n");


    /**********************************
     * 2. Check MS5607 sensor readiness
     **********************************/
    if (!device_is_ready(ms5607_dev)) {
        printk("MS5607 sensor not ready or not found\n");
        /* If sensor is optional, you can continue w/o it.
         * Otherwise, return an error code.
         */
        //return 3;
    } else {
        printk("Found MS5607 barometer\n");
    }


    /**********************************
     * 3. Begin main loop
     **********************************/
    printk("Sending data packets with MS5607 readings...\n");
    union DataPacket txPacket;

    while (1) {
        /* Zero the entire buffer first */
        memset(txPacket.buffer, 0, TX_PACKET_SIZE);

        /* Fill the fixed fields with dummy or optional real data */
        txPacket.fields.latitude    = 12345678;    // pretend lat * 1e7
        txPacket.fields.longitude   = -23456789;   // pretend lon * 1e7
        txPacket.fields.altitude    = 100000;      // pretend altitude in mm
        txPacket.fields.timeMs      = k_uptime_get_32(); 
        txPacket.fields.satellites  = 7;           
        txPacket.fields.status      = 0x01;        

        /**********************************
         * 4. Read MS5607 Pressure & Temp
         **********************************/
        if (device_is_ready(ms5607_dev)) {
            struct sensor_value press_val, temp_val;

            /* Trigger a new measurement */
            ret = sensor_sample_fetch(ms5607_dev);
            if (ret == 0) {
                /* Get pressure in hPa (SENSOR_CHAN_PRESS) */
                ret = sensor_channel_get(ms5607_dev, SENSOR_CHAN_PRESS, &press_val);
                if (ret == 0) {
                    /* Convert sensor_value to integer Pa 
                     * press_val.val1 = integer part in hPa
                     * press_val.val2 = fractional part in 1/10000 hPa
                     * 1 hPa = 100 Pa
                     */
                    int32_t press_hpa_int = press_val.val1;
                    int32_t press_hpa_frac = press_val.val2 / 10000; 
                    /* Combined pressure in Pa, e.g. 1013.25 hPa = 101325 Pa */
                    int32_t pressure_pa = (press_hpa_int * 100) + press_hpa_frac;
                    pressure_pa *= 100; // hPa to Pa
                    txPacket.fields.pressure = pressure_pa;
                }

                /* Get temperature in °C (SENSOR_CHAN_AMBIENT_TEMP) */
                ret = sensor_channel_get(ms5607_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val);
                if (ret == 0) {
                    /* temp_val.val1 = integer part of degrees
                     * temp_val.val2 = fractional part in 1/1000000
                     * Typically for MS5607: val2 is in increments of 10000
                     *
                     * We want hundredths of °C. 
                     */
                    int32_t t_int = temp_val.val1;                 // e.g. 23
                    int32_t t_frac = temp_val.val2 / 10000;        // e.g. 45
                    int32_t temperature_centi = t_int * 100 + t_frac;
                    txPacket.fields.temperature = (int16_t)temperature_centi;
                }
            } else {
                printk("MS5607 fetch error: %d\n", ret);
            }
        }

        /**********************************
         * 5. Compute packet checksum
         **********************************/
        uint8_t checksum = 0;
        for (size_t i = 0; i < (TX_PACKET_SIZE - 1); i++) {
            checksum ^= txPacket.buffer[i];
        }
        txPacket.fields.checksum = checksum;

        /**********************************
         * 6. Transmit the 25-byte packet
         **********************************/
        ret = lora_send(lora_dev, txPacket.buffer, TX_PACKET_SIZE);
        if (ret < 0) {
            printk("LoRa send failed: %d\n", ret);
        } else {
            printk("LoRa 25-byte packet sent (CHKSUM=0x%02X)\n", checksum);
        }

        /* Wait and repeat */
        k_msleep(TX_INTERVAL_MS);
    }

    return 0;
}
