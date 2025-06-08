
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/drivers/i2c.h>

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)
// === DeviceTree Labels ===
#define SPI_NODE    DT_NODELABEL(spi1)
#define GPIO_NODE   DT_NODELABEL(gpio0)
#define CS_PIN      4
#define DRDY_PIN    15
#define RESET_PIN   9
// === ADS1299 Commands ===
#define CMD_WAKEUP   0x02
#define CMD_STANDBY  0x04
#define CMD_RESET    0x06
#define CMD_START    0x08
#define CMD_STOP     0x0A
#define CMD_RDATAC   0x10
#define CMD_SDATAC   0x11
#define CMD_RREG     0x20
// === ADS1299 Registers ===
#define CONFIG1   0x01
#define CONFIG2   0x02
#define CONFIG3   0x03
#define CH1SET    0x05
#define CH8SET    0x0C
// === Constants ===
#define VREF 4.5   // 4.5V in microvolts
#define GAIN 1UL
#define FULL_SCALE_CODES 16777215UL  // 2^24 - 1
#define NUM_CHANNELS 8
#define BYTES_PER_CHANNEL 3
#define BLE_MSG_MAX_LEN 256

#define LSM303_ACCEL_ADDR  0x19  // Accelerometer I2C address
#define LSM303_CTRL_REG1_A 0x20  // Control register to enable accelerometer
#define LSM303_OUT_X_L_A   0x28  // X-axis low byte output register (autoincrement works)


uint8_t ble_msg[BLE_MSG_MAX_LEN] = {0};


// Enable accelerometer (normal mode, all axes enabled, 50 Hz data rate)
int enable_accelerometer(const struct device *i2c_dev) {
    uint8_t config[2] = {LSM303_CTRL_REG1_A, 0x47}; // 0x47 = 50 Hz, XYZ enabled
    return i2c_write(i2c_dev, config, 2, LSM303_ACCEL_ADDR);
}

void read_acceleration(const struct device *i2c_dev) {
    uint8_t reg = LSM303_OUT_X_L_A | 0x80; // Set MSB to enable auto-increment
    uint8_t raw_data[6]; // X_L, X_H, Y_L, Y_H, Z_L, Z_H

    // Write register pointer
    int err = i2c_write(i2c_dev, &reg, 1, LSM303_ACCEL_ADDR);
    if (err < 0) {
        printk("Failed to write accel data reg: %d\n", err);
        return;
    }

    // Read 6 bytes of data
    err = i2c_read(i2c_dev, raw_data, 6, LSM303_ACCEL_ADDR);
    if (err < 0) {
        printk("Failed to read accel data: %d\n", err);
        return;
    }

    // Convert to 16-bit signed integers
    int16_t ax = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    int16_t ay = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t az = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // printk("Accel X: %d, Y: %d, Z: %d\n", ax, ay, az);
}

// === SPI & GPIO Devices ===
const struct device *spi_dev;
const struct device *gpio_dev;
struct spi_config spi_cfg = {
    .frequency = 1000000U,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL,  // Manual CS control
};
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};
static void notif_enabled(bool enabled, void *ctx)
{
    ARG_UNUSED(ctx);
    printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}
static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    char message[CONFIG_BT_L2CAP_TX_MTU + 1] = "";
    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);
    memcpy(message, data, MIN(sizeof(message) - 1, len));
    printk("%s() - Len: %d, Message: %s\n", __func__, len, message);
}
struct bt_nus_cb nus_listener = {
    .notif_enabled = notif_enabled,
    .received = received,
};

// === Helper Macros ===
#define CS_LOW()  gpio_pin_set(gpio_dev, CS_PIN, 0)
#define CS_HIGH() gpio_pin_set(gpio_dev, CS_PIN, 1)

// === SPI Communication Helpers ===
int read_data(uint8_t *buffer, size_t len) {
    uint8_t dummy_tx[27] = {0};
    struct spi_buf tx_buf = { .buf = dummy_tx, .len = len };
    struct spi_buf rx_buf = { .buf = buffer, .len = len };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };
    CS_LOW();
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    CS_HIGH();
    return ret;
}
uint8_t ads1299_transfer(uint8_t data) {
    uint8_t tx_buf[1] = { data };
    uint8_t rx_buf[1] = { 0 };
    struct spi_buf tx_spi_buf = { .buf = tx_buf, .len = 1 };
    struct spi_buf rx_spi_buf = { .buf = rx_buf, .len = 1 };
    struct spi_buf_set tx = { .buffers = &tx_spi_buf, .count = 1 };
    struct spi_buf_set rx = { .buffers = &rx_spi_buf, .count = 1 };
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    if (ret < 0) {
        printk("SPI transfer failed: %d\n", ret);
    }
    return rx_buf[0];
}
void ads1299_send_command(uint8_t cmd) {
    gpio_pin_set(gpio_dev, CS_PIN, 0);     // CS LOW
    ads1299_transfer(cmd);                 // Send command byte
    gpio_pin_set(gpio_dev, CS_PIN, 1);     // CS HIGH
    k_busy_wait(3);                        // Small delay (3 µs)
}

void read_all_registers(void) {
    gpio_pin_set(gpio_dev, CS_PIN, 0); // CS LOW
    ads1299_transfer(CMD_SDATAC); // Stop continuous data mode
    ads1299_transfer(CMD_RREG | 0x00); // Start from address 0x00
    ads1299_transfer(0x18 - 1); // Read all registers (n-1)
    for (uint8_t i = 0; i < 0x18; i++) {
        uint8_t data = ads1299_transfer(0x00);
        // print_register_name(i); // Optional helper to display register name
        printk("Reg 0x%02X = 0x%02X | ", i, data);
        for (int j = 7; j >= 0; j--) {
            printk("%d", (data >> j) & 1);
        }
        printk("\n");
    }
    gpio_pin_set(gpio_dev, CS_PIN, 1); // CS HIGH
}
void power_up_sequence(void) {
    uint8_t dummy_data[27] = {0};
    // === Reset the device ===
    ads1299_send_command(CMD_RESET);
    k_msleep(10);
    printk("Power Up Done 111!\n");
    // === Stop continuous read mode ===
    ads1299_send_command(CMD_SDATAC);
    k_msleep(1);
    // === CONFIG3: Enable internal reference buffer (0xE0) ===
    gpio_pin_set(gpio_dev, CS_PIN, 0);
    ads1299_transfer(0x40 | CONFIG3);  // WREG CONFIG3
    ads1299_transfer(0x00);            // 1 register
    ads1299_transfer(0xE0);            // value
    gpio_pin_set(gpio_dev, CS_PIN, 1);
    // === CONFIG1: Set data rate (0x96) ===
    gpio_pin_set(gpio_dev, CS_PIN, 0);
    ads1299_transfer(0x40 | CONFIG1);
    ads1299_transfer(0x00);
    ads1299_transfer(0x96);
    gpio_pin_set(gpio_dev, CS_PIN, 1);
    // === CONFIG2: PDB_REFBUF = 1, INT_TEST = 0 (0xC0) ===
    gpio_pin_set(gpio_dev, CS_PIN, 0);
    ads1299_transfer(0x40 | CONFIG2);
    ads1299_transfer(0x00);
    ads1299_transfer(0xC0);
    gpio_pin_set(gpio_dev, CS_PIN, 1);
    // === CHnSET: All channels shorted (0x01) ===
    for (uint8_t reg = CH1SET; reg <= CH8SET; reg++) {
        gpio_pin_set(gpio_dev, CS_PIN, 0);
        ads1299_transfer(0x40 | reg);
        ads1299_transfer(0x00);
        ads1299_transfer(0x01);
        gpio_pin_set(gpio_dev, CS_PIN, 1);
    }
    // === Wait for internal reference to settle ===
    k_msleep(600);
    // === Start conversion ===
    ads1299_send_command(CMD_START);
    k_msleep(5);
    // === Begin continuous data read ===
    ads1299_send_command(CMD_RDATAC);
    printk("Power Up Done 222!\n");
    // === Wait for DRDY and read one frame ===
    while (gpio_pin_get(gpio_dev, DRDY_PIN) != 0) {
        k_msleep(1);
    }
    read_data(dummy_data, sizeof(dummy_data));
    printk("Power Up Done 333!\n");
    // === Stop RDATAC to change channel settings ===
    ads1299_send_command(CMD_SDATAC);
    // === CONFIG2: Enable internal test signal (0xD0) ===
    gpio_pin_set(gpio_dev, CS_PIN, 0);
    ads1299_transfer(0x40 | CONFIG2);
    ads1299_transfer(0x00);
    ads1299_transfer(0xC0);
    gpio_pin_set(gpio_dev, CS_PIN, 1);
    // === CHnSET: Set all channels to test signal (0x65) ===
    for (uint8_t reg = CH1SET; reg <= CH8SET; reg++) {
        gpio_pin_set(gpio_dev, CS_PIN, 0);
        ads1299_transfer(0x40 | reg);
        ads1299_transfer(0x00);
        ads1299_transfer(0x00);  // 0x65 = 01100101: MUX[2:0]=101 (test), PDB=1
        gpio_pin_set(gpio_dev, CS_PIN, 1);
    }
    // === Resume RDATAC ===
    ads1299_send_command(CMD_RDATAC);
    while (gpio_pin_get(gpio_dev, DRDY_PIN) != 0) {
        k_msleep(1);
    }
    read_data(dummy_data, sizeof(dummy_data));
}



// === Main Logic ===
void main(void) {
    int err;
    printk("Sample - Bluetooth Peripheral NUS\n");
    err = bt_nus_cb_register(&nus_listener, NULL);
    if (err) {
     printk("Failed to register NUS callback: %d\n", err);
     return err;
    }
    err = bt_enable(NULL);
    if (err) {
     printk("Failed to enable bluetooth: %d\n", err);
     return err;
    }
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
     printk("Failed to start advertising: %d\n", err);
     return err;
    }
    printk("Initialization complete\n");

    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }

    printk("Enabling accelerometer...\n");
    if (enable_accelerometer(i2c_dev) < 0) {
        printk("Failed to enable accelerometer\n");
        return;
    }
    
    printk("Starting ADS1299 test...\n");
    spi_dev = DEVICE_DT_GET(SPI_NODE);
    gpio_dev = DEVICE_DT_GET(GPIO_NODE);
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready!\n");
        return;
    }
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready!\n");
        return;
    }
    gpio_pin_configure(gpio_dev, CS_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio_dev, DRDY_PIN, GPIO_INPUT);
    // === Run ADS1299 power-up sequence ===
    read_all_registers();
    power_up_sequence();
    printk("Power Up Done!\n");
    read_all_registers();
    // === Main loop: read data when DRDY goes low ===
    ads1299_send_command(CMD_SDATAC);
    k_msleep(1);
    ads1299_send_command(CMD_START);
    k_msleep(1);
    ads1299_send_command(CMD_RDATAC);
    k_msleep(1);
    k_msleep(3000);
    uint8_t buffer[27];
    int64_t start_time = k_uptime_get();
    // uint8_t data[27];
    // read_data(data, sizeof(data));

    uint8_t ble_payload[24];
            // ⚠️ Reset buffer and offset at the start of each sample
    // char ble_msg[64];/
    memset(ble_msg, 0, sizeof(ble_msg));
    size_t offset = 0;

    while (1) {
        // read_acceleration(i2c_dev);
        while (gpio_pin_get(gpio_dev, DRDY_PIN));  // Wait for DRDY LOW

        if (read_data(buffer, sizeof(buffer)) != 0) {
            printk("SPI read error\n");
            continue;
        }

        char ble_msg[BLE_MSG_MAX_LEN] = {0};
        int offset = 0;
        bool valid_sample = true;

        for (int ch = 7; ch < NUM_CHANNELS; ch++) {
            int base = 3 + ch * 3;  // Skip 3-byte status

            long raw = ((long)buffer[base] << 16) | ((long)buffer[base + 1] << 8) | buffer[base + 2];
            if (raw & 0x800000) raw |= 0xFF000000;

            float lsb = (2.0f * VREF) / (GAIN * FULL_SCALE_CODES);
            float volts = raw * lsb;

            // Filter out bad data
            if ((raw & 0xFFFFFF) == 0xC00000 || fabs(volts) > 0.05f) {
                printk("⚠️ Desync or spike on CH%d (0x%06lx), skipping sample...\n", ch + 1, raw & 0xFFFFFF);
                valid_sample = false;
                break;  // skip entire sample set if one channel is bad
            }

            int int_part = (int)volts;
            int frac_part = (int)((volts - int_part) * 1000000000);  // 9-digit precision

            int len = snprintf(ble_msg + offset, sizeof(ble_msg) - offset,
                "%s%d.%09d%s",
                (volts < 0) ? "-" : "", abs(int_part), abs(frac_part),
                (ch < NUM_CHANNELS - 1) ? "," : "\n");  // comma except after last

            if (len < 0 || len >= sizeof(ble_msg) - offset) {
                printk("⚠️ BLE msg buffer overflow on CH%d\n", ch + 1);
                valid_sample = false;
                break;
            }

            offset += len;
        }

        if (!valid_sample) {
            continue;
        }

        // Debug print
        // printk("BLE message: %s", ble_msg);

        // Send BLE data
        int err = bt_nus_send(NULL, ble_msg, strlen(ble_msg));
        if (err) {
            printk("Failed to send BLE data: %d\n", err);
        }
    }
}
