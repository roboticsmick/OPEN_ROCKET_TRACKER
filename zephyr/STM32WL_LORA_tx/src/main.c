#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/sys/ring_buffer.h>

/* Configuration Constants */
#define TX_PACKET_SIZE 25
#define TX_INTERVAL_MS 2000
#define GPS_I2C_ADDR 0x42
#define GPS_BUFFER_SIZE 512

/* Thread Configuration */
#define BARO_STACK_SIZE 1024
#define TX_STACK_SIZE 1024
#define GNSS_STACK_SIZE 2048 
#define BARO_PRIORITY 1    
#define GNSS_PRIORITY 1    
#define TX_PRIORITY 2      

/* Thread Stacks */
K_THREAD_STACK_DEFINE(baro_stack, BARO_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_stack, TX_STACK_SIZE);
K_THREAD_STACK_DEFINE(gnss_stack, GNSS_STACK_SIZE);

/* Thread Data Objects */
static struct k_thread baro_thread_data;
static struct k_thread tx_thread_data;
static struct k_thread gnss_thread_data;

/* Synchronization */
K_SEM_DEFINE(data_ready_sem, 0, 1);
K_MUTEX_DEFINE(data_mutex);

/* Ring buffer configuration */
#define RING_BUF_SIZE 1024
static struct ring_buf gnss_ring_buf;
static uint8_t ring_buffer_data[RING_BUF_SIZE];

/* Device pointers */
static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
static const struct device *ms5607_dev = DEVICE_DT_GET_ANY(meas_ms5607);
static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));

/* Data packet structure */
union DataPacket {
    struct {
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
    uint8_t buffer[TX_PACKET_SIZE];
};

/* Shared data packet */
static union DataPacket shared_packet;

/* GNSS thread function */
static void gnss_thread(void *arg1, void *arg2, void *arg3)
{
    /* Make sure the I2C device is ready */
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready for GNSS\n");
        return;
    }

    /* Initialize ring buffer */
    ring_buf_init(&gnss_ring_buf, sizeof(ring_buffer_data), ring_buffer_data);

    uint8_t temp_byte;
    uint8_t parse_buffer[GPS_BUFFER_SIZE];
    
    while (1) {
        /* Read from I2C into ring buffer */
        while (ring_buf_space_get(&gnss_ring_buf) > 0) {
            int ret = i2c_read(i2c_dev, &temp_byte, 1, GPS_I2C_ADDR);
            if (ret < 0) {
                printk("GNSS I2C read error\n");
                break;
            }

            /* Accept ASCII range + CR/LF */
            if ((temp_byte >= 32 && temp_byte <= 126) || 
                temp_byte == '\r' || temp_byte == '\n') {
                uint32_t bytes_written = ring_buf_put(&gnss_ring_buf, 
                                                    &temp_byte, 1);
                if (bytes_written != 1) {
                    printk("Ring buffer full\n");
                    break;
                }
            } else {
                break; /* stop reading if it's not a typical NMEA char */
            }
        }

        /* Process data from ring buffer */
        uint32_t bytes_available = ring_buf_size_get(&gnss_ring_buf);
        if (bytes_available > 0) {
            /* Read data from ring buffer into parse buffer */
            uint32_t bytes_read = ring_buf_get(&gnss_ring_buf, 
                                             parse_buffer,
                                             MIN(bytes_available, GPS_BUFFER_SIZE));

            /* Look for GGA sentences in the buffer */
            for (size_t i = 0; i < bytes_read; i++) {
                /* Start of an NMEA sentence? */
                if (parse_buffer[i] == '$') {
                    /* Check if it's GGA ($GNGGA or $GPGGA) */
                    if ((i + 5 < bytes_read) &&
                       ((strncmp((char *)&parse_buffer[i], "$GNGGA", 6) == 0) ||
                        (strncmp((char *)&parse_buffer[i], "$GPGGA", 6) == 0))) {

                        /* Find the end '*' or run out of buffer */
                        size_t end_idx = i;
                        while ((end_idx < bytes_read) && 
                               (parse_buffer[end_idx] != '*')) {
                            end_idx++;
                        }

                        /* If we found '*', we at least have a full sentence */
                        if (end_idx < bytes_read) {
                            /* Temporarily make it a string by adding null terminator */
                            if (end_idx + 3 < bytes_read) {
                                parse_buffer[end_idx] = '\0';
                            }

                            /* Your existing NMEA parsing code remains the same */
                            char *sentence = (char *)&parse_buffer[i];
                            char *token;
                            int field_idx = 0;

                            /* Temporary local vars */
                            uint8_t satellites = 0;
                            int32_t altitude = 0;
                            int32_t lat_degs_fixed = 0;
                            int32_t lon_degs_fixed = 0;
                            int32_t gnss_time_hhmmss = 0;

                            token = strtok(sentence, ",");
                            while (token != NULL) {
                                field_idx++;
                                token = strtok(NULL, ",");

                                if (token == NULL) {
                                    break;
                                }

                                /* Your existing switch case for parsing remains the same */
                                switch (field_idx) {
                                    case 1: {
                                        /* hhmmss.sss (GNSS time) */
                                        /* We'll store as integer HHMMSS (ignore decimals) */
                                        /* e.g. "123519.00" -> 123519  */
                                        char hhmmss_str[7] = {0}; 
                                        strncpy(hhmmss_str, token, 6); 
                                        gnss_time_hhmmss = atoi(hhmmss_str);
                                        break;
                                    }
                                    case 2: {
                                        /* latitude in ddmm.mmmm */
                                        /* We'll parse to decimal degrees * 1e7 to store in int32 */
                                        /* example: "5133.8201" -> 51 + (33.8201/60) = 51.56367 deg */
                                        double raw_lat = atof(token);
                                        int deg = (int)(raw_lat / 100.0);
                                        double min = raw_lat - (deg * 100.0);
                                        double dec_deg = deg + (min / 60.0);
                                        /* Multiply by 1e7 for int32 storage (common approach) */
                                        lat_degs_fixed = (int32_t)(dec_deg * 1e7);
                                        break;
                                    }
                                    case 3: {
                                        /* 'N' or 'S' */
                                        if (*token == 'S') {
                                            lat_degs_fixed = -lat_degs_fixed;
                                        }
                                        break;
                                    }
                                    case 4: {
                                        /* longitude in dddmm.mmmm */
                                        double raw_lon = atof(token);
                                        int deg = (int)(raw_lon / 100.0);
                                        double min = raw_lon - (deg * 100.0);
                                        double dec_deg = deg + (min / 60.0);
                                        lon_degs_fixed = (int32_t)(dec_deg * 1e7);
                                        break;
                                    }
                                    case 5: {
                                        /* 'E' or 'W' */
                                        if (*token == 'W') {
                                            lon_degs_fixed = -lon_degs_fixed;
                                        }
                                        break;
                                    }
                                    case 7: {
                                        /* satellites in use */
                                        satellites = (uint8_t)atoi(token);
                                        break;
                                    }
                                    case 9: {
                                        /* altitude in meters */
                                        double alt_f = atof(token);
                                        altitude = (int32_t)alt_f;
                                        break;
                                    }
                                    default:
                                        /* We skip the rest */
                                        break;
                            }
                            }

                            /* Update shared_packet with new GNSS data */
                            k_mutex_lock(&data_mutex, K_FOREVER);
                            shared_packet.fields.latitude = lat_degs_fixed;
                            shared_packet.fields.longitude = lon_degs_fixed;
                            shared_packet.fields.altitude = altitude;
                            shared_packet.fields.timeMs = gnss_time_hhmmss;
                            shared_packet.fields.satellites = satellites;
                            shared_packet.fields.status = 1;
                            k_mutex_unlock(&data_mutex);

                            /* Signal new data */
                            k_sem_give(&data_ready_sem);
                            
                            /* Move i forward, we're done with this sentence */
                            i = end_idx;
                        }
                    }
                }
            }
        }

        /* Add monitoring for ring buffer usage */
        uint32_t space_available = ring_buf_space_get(&gnss_ring_buf);
        uint32_t usage_percent = ((RING_BUF_SIZE - space_available) * 100) / RING_BUF_SIZE;
        
        if (usage_percent > 80) {
            printk("Warning: GNSS ring buffer usage at %u%%\n", usage_percent);
        }

        /* Small delay before next read cycle */
        k_msleep(50);  
    }
}


/* Function to compute checksum */
static uint8_t compute_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/* Barometer thread function */
static void baro_thread(void *arg1, void *arg2, void *arg3) {
    while (1) {
        if (device_is_ready(ms5607_dev)) {
            struct sensor_value press_val, temp_val;
            int ret;

            /* Take mutex before updating shared data */
            k_mutex_lock(&data_mutex, K_FOREVER);

            ret = sensor_sample_fetch(ms5607_dev);
            if (ret == 0) {
                /* Get pressure */
                ret = sensor_channel_get(ms5607_dev, SENSOR_CHAN_PRESS, &press_val);
                if (ret == 0) {
                    int32_t press_hpa_int = press_val.val1;
                    int32_t press_hpa_frac = press_val.val2 / 10000;
                    int32_t pressure_pa = (press_hpa_int * 100 + press_hpa_frac) * 100;
                    shared_packet.fields.pressure = pressure_pa;
                }

                /* Get temperature */
                ret = sensor_channel_get(ms5607_dev, 
                                       SENSOR_CHAN_AMBIENT_TEMP, 
                                       &temp_val);
                if (ret == 0) {
                    int32_t t_int = temp_val.val1;
                    int32_t t_frac = temp_val.val2 / 10000;
                    int32_t temperature_centi = t_int * 100 + t_frac;
                    shared_packet.fields.temperature = (int16_t)temperature_centi;
                }

                /* Update timestamp */
                shared_packet.fields.timeMs = k_uptime_get_32();
            }

            k_mutex_unlock(&data_mutex);

            /* Signal that new data is ready */
            k_sem_give(&data_ready_sem);
        }

        /* Wait before next reading */
        k_msleep(500);  /* Sample at 2Hz */
    }
}

/* Transmitter thread function */
static void tx_thread(void *arg1, void *arg2, void *arg3) {
    /* Configure LoRa radio */
    struct lora_modem_config config = {
        .frequency = 922600000,
        .bandwidth = BW_125_KHZ,
        .datarate = SF_12,
        .coding_rate = CR_4_5,
        .preamble_len = 8,
        .tx_power = 13,
        .tx = true,
        .public_network = false,
        .iq_inverted = false,
    };

    if (lora_config(lora_dev, &config) < 0) {
        printk("LoRa config failed\n");
        return;
    }

    union DataPacket tx_packet;

    while (1) {
        /* Wait for new data */
        k_sem_take(&data_ready_sem, K_FOREVER);

        /* Copy shared data safely */
        k_mutex_lock(&data_mutex, K_FOREVER);
        memcpy(&tx_packet, &shared_packet, sizeof(union DataPacket));
        k_mutex_unlock(&data_mutex);

        /* Compute and add checksum */
        tx_packet.fields.checksum = compute_checksum(tx_packet.buffer, 
                                                   TX_PACKET_SIZE - 1);

        /* Transmit packet */
        int ret = lora_send(lora_dev, tx_packet.buffer, TX_PACKET_SIZE);
        if (ret < 0) {
            printk("LoRa send failed: %d\n", ret);
        } else {
            printk("LoRa packet sent (CHKSUM=0x%02X)\n", 
                   tx_packet.fields.checksum);
        }

        k_msleep(TX_INTERVAL_MS);
    }
}

int main(void)
{
    /* Check device readiness */
    if (!device_is_ready(lora_dev) || !device_is_ready(i2c_dev)) {
        printk("LoRa or I2C device not ready\n");
        return 1;
    }

    if (!device_is_ready(ms5607_dev)) {
        printk("MS5607 sensor not ready\n");
        return 2;
    }

    /* Initialize shared packet */
    memset(&shared_packet, 0, sizeof(shared_packet));

    /* Create threads */
    k_thread_create(&baro_thread_data,
                   baro_stack,
                   BARO_STACK_SIZE,
                   baro_thread,
                   NULL, NULL, NULL,
                   BARO_PRIORITY,
                   0,
                   K_NO_WAIT);

    k_thread_create(&gnss_thread_data,
                   gnss_stack,
                   GNSS_STACK_SIZE,
                   gnss_thread,
                   NULL, NULL, NULL,
                   GNSS_PRIORITY,
                   0,
                   K_NO_WAIT);

    k_thread_create(&tx_thread_data,
                   tx_stack,
                   TX_STACK_SIZE,
                   tx_thread,
                   NULL, NULL, NULL,
                   TX_PRIORITY,
                   0,
                   K_NO_WAIT);

    /* Name threads for debugging */
    k_thread_name_set(&baro_thread_data, "baro_thread");
    k_thread_name_set(&gnss_thread_data, "gnss_thread");
    k_thread_name_set(&tx_thread_data, "tx_thread");

    return 0;
}