#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>

#include "batterydisplay.h"
#include "value.h"

LOG_MODULE_REGISTER(co2_watchdog);

#define RECEIVE_TIMEOUT 1000
#define MSG_SIZE 9
#define CO2_MULTIPLIER 256
#define CO2_SEGMENTS 8
#define MATRIX_COLUMNS 16

#define MAX_SENSORVALUE 1000
#define MIN_SENSORVALUE 8
#define SENSOR_INVALID_VALUE 65500

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

#define BUTTON_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(BUTTON_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(BUTTON_NODE, gpios, {0});
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

K_MSGQ_DEFINE(co2_segment_msgq, sizeof(int), MATRIX_COLUMNS, 4);
K_MSGQ_DEFINE(sound_segment_msgq, sizeof(int), MATRIX_COLUMNS, 4);

#define LED_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(holtek_ht16k33)
#define UART_NODE DT_ALIAS(myserial)

static int sound_segments[MATRIX_COLUMNS] = {0};
static const struct device *const uart_serial = DEVICE_DT_GET(UART_NODE);
static const struct device *const led = DEVICE_DT_GET(LED_NODE);

static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

static volatile bool co2_data_ready = false;
static volatile bool init_ready = false;
static int co2_segment = 0;
static uint16_t co2_value = 0;
static struct bt_conn *current_conn = NULL;

enum uart_fsm_code {
    UART_FSM_IDLE,
    UART_FSM_HEADER,
    UART_FSM_DATA,
    UART_FSM_CHECKSUM,
    UART_FSM_END,
};

static uint8_t uart_fsm = UART_FSM_IDLE;

uint8_t check_usart_fsm(uint8_t read_data) {
    switch (uart_fsm) {
        case UART_FSM_IDLE:
            if (read_data == 0xFF) {
                uart_fsm = UART_FSM_HEADER;
            }
            break;
        case UART_FSM_HEADER:
            if (read_data == 0x86) {
                uart_fsm = UART_FSM_DATA;
            } else {
                uart_fsm = UART_FSM_IDLE;
            }
            break;
        case UART_FSM_DATA:
            if (rx_buf_pos == MSG_SIZE - 2) {
                uart_fsm = UART_FSM_CHECKSUM;
            }
            break;
        case UART_FSM_CHECKSUM:
            if (rx_buf_pos == MSG_SIZE - 1) {
                uart_fsm = UART_FSM_END;
            }
            break;
        case UART_FSM_END:
            uart_fsm = UART_FSM_IDLE;
            break;
        default:
            uart_fsm = UART_FSM_IDLE;
            break;
    }
    return uart_fsm;
}

unsigned char getCheckSum(unsigned char *packet) {
    unsigned char i, checksum = 0;
    for (i = 1; i < 8; i++) {
        checksum += packet[i];
    }
    checksum = 0xff - checksum;
    checksum += 1;
    return checksum;
}

uint8_t fromHexadecimalToDecimal(uint8_t hexadecimalValue) {
    uint8_t decimalValue = 0;
    decimalValue += hexadecimalValue / 10 * 16;
    decimalValue += hexadecimalValue % 10;
    return decimalValue;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void serial_callback(const struct device *dev, void *user_data) {
    uint8_t c, high, low;
    char checksum_ok, value_calc_flag;
    int checksum;

    if (!uart_irq_update(uart_serial)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_serial)) {
        LOG_INF("UART RX not ready");
        return;
    }

    while (uart_fifo_read(uart_serial, &c, 1) == 1) {
        
        if (uart_fsm == UART_FSM_IDLE) {
            rx_buf_pos = 0;
        }
        check_usart_fsm(c);

        if (rx_buf_pos >= MSG_SIZE) {
            rx_buf_pos = 0;
        }
        rx_buf[rx_buf_pos++] = c;
    }

    checksum = getCheckSum((unsigned char *)rx_buf);
    checksum_ok = checksum == rx_buf[8];
    if (checksum_ok) {
        LOG_INF("Checksum OK (%d == %d, index=%d)", checksum, rx_buf[8], rx_buf_pos);
    } else {
        LOG_ERR("Checksum failed (%d == %d, index=%d)", checksum, rx_buf[8], rx_buf_pos);
    }

    value_calc_flag = rx_buf_pos == MSG_SIZE && checksum_ok;
    if (value_calc_flag) {
        high = rx_buf[2];
        high = fromHexadecimalToDecimal(high);
        low = rx_buf[3];
        low = fromHexadecimalToDecimal(low);
        int ppm = (high * CO2_MULTIPLIER) + low;
        co2_value = ppm;
        printk("CO2: %d ppm (high = %d, low = %d)\n", ppm, high, low);
        for (int i = 0; i < MSG_SIZE; i += 1) {
            printk("%x ", rx_buf[i]);
        }
        printk("\n");

        if (ppm < 375) co2_segment = 1;
        else if (ppm < 750) co2_segment = 2;
        else if (ppm < 1125) co2_segment = 3;
        else if (ppm < 1500) co2_segment = 4;
        else if (ppm < 1875) co2_segment = 5;
        else if (ppm < 2250) co2_segment = 6;
        else if (ppm < 2625) co2_segment = 7;
        else if (ppm < 3000) co2_segment = 8;

        co2_data_ready = true;
    }
}

void serial_write() {
    uint8_t tx_buf[MSG_SIZE] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    for (int i = 0; i < MSG_SIZE; i += 1) {
        uart_poll_out(uart_serial, tx_buf[i]);
    }
}

void update_led_matrix(void) {
    if (!device_is_ready(led)) {
        printk("LED matrix device not ready\n");
        return;
    }

    for (int col = 0; col < MATRIX_COLUMNS; col++) {
        for (int row = 0; row < CO2_SEGMENTS; row++) {
            led_off(led, row * 16 + col);
        }
    }

    int start_col = MATRIX_COLUMNS - k_msgq_num_used_get(&sound_segment_msgq);
    for (int col = start_col; col < MATRIX_COLUMNS; col++) {
        for (int row = 0; row < sound_segments[col - start_col]; row++) {
            led_on(led, (CO2_SEGMENTS - 1 - row) * 16 + col);
        }
    }
}

void add_sound_segment(int segment) {
    if (k_msgq_put(&sound_segment_msgq, &segment, K_NO_WAIT) != 0) {
        int old_segment;
        k_msgq_get(&sound_segment_msgq, &old_segment, K_NO_WAIT);
        k_msgq_put(&sound_segment_msgq, &segment, K_NO_WAIT);
    }

    memset(sound_segments, 0, sizeof(sound_segments));
    int count = k_msgq_num_used_get(&sound_segment_msgq);
    for (int i = 0; i < count; i++) {
        k_msgq_get(&sound_segment_msgq, &sound_segments[MATRIX_COLUMNS - count + i], K_NO_WAIT);
        k_msgq_put(&sound_segment_msgq, &sound_segments[MATRIX_COLUMNS - count + i], K_NO_WAIT);
    }

    update_led_matrix();
}

void battery_disp()
{
    if (init_ready) {
        printk("Initializing battery display to 0\n");
        display_level(0);
        init_ready = false;
    }
    
    printk("CO2 scale: %d\n", co2_segment);

    if (co2_segment >= 7) {
        display_level(1);
    } else if (co2_segment >= 6 && co2_segment < 7) {
        display_level(2);
    } else if (co2_segment >= 5 && co2_segment < 6) {
        display_level(3);
    } else if (co2_segment >= 4 && co2_segment < 5) {
        display_level(4);
    } else if (co2_segment >= 3 && co2_segment < 4) {
        display_level(5);
    } else if (co2_segment >= 2 && co2_segment < 3) {
        display_level(6);
    } else if (co2_segment >= 1 && co2_segment < 2) {
        display_level(7);
    } else {
        display_level(7);
    }
}

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)
#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000
#define USER_LED                DK_LED3
#define USER_BUTTON             DK_BTN1_MSK

static bool app_button_state;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct bt_uuid_128 co2_service_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345678));
static struct bt_uuid_128 co2_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x87654321, 0x4321, 0x8765, 0x4321, 0x876543218765));

static const char co2_char_desc[] = "CO2 Monitoring";

static void co2_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

    printk("Notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(co2_svc,
    BT_GATT_PRIMARY_SERVICE(&co2_service_uuid),
    BT_GATT_CHARACTERISTIC(&co2_char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           NULL, NULL, NULL),
    BT_GATT_CCC(co2_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_DESCRIPTOR(BT_UUID_GATT_CUD, BT_GATT_PERM_READ, NULL, NULL, (void *)co2_char_desc)
);

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Bluetooth connection failed (err %u)\n", err);
        return;
    }

    printk("Connected\n");
    current_conn = bt_conn_ref(conn);
    dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnect Bluetooth (Reason: %u)\n", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    dk_set_led_off(CON_STATUS_LED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
};

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Initializing system...\n");

    memset(sound_segments, 0, sizeof(sound_segments));
    update_led_matrix();
    co2_data_ready = false;
    battery_disp();
    co2_segment = 0;
    co2_value = 0;

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    dk_set_led_off(CON_STATUS_LED);
    printk("System was initialized.\n");
}

static struct gpio_callback button_cb_data;

static int init_button(void)
{
    int ret;

    if (!device_is_ready(button.port)) {
        printk("Error: button device %s is not ready\n", button.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n",
               ret, button.port->name, button.pin);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on %s pin %d\n",
               ret, button.port->name, button.pin);
        return ret;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    printk("Setting button %s pin %d\n", button.port->name, button.pin);

    return 0;
}

void bluetooth_init(void)
{
    int err;

    err = dk_leds_init();
    if (err) {
        printk("LEDs init failed (err %d)\n", err);
        return;
    }

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Advertising successfully started\n");
}

void co2_thread(void) {
    while (1) {
        k_sleep(K_MSEC(3000));
        serial_write();
        
        if (co2_data_ready) {
            if (current_conn) {
                int rc = bt_gatt_notify(current_conn, &co2_svc.attrs[1], &co2_value, sizeof(co2_value));
                if (rc < 0) {
                    printk("notifying error: %d\n", rc);
                }
            }
            battery_disp(co2_segment);
            co2_data_ready = false;
        }
    }
}

void sound_thread(void) {
    int err;
    uint32_t sound_value;
    uint16_t buf;

    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
        .channels = BIT(adc_channel.channel_id),
        .resolution = adc_channel.resolution,
    };

    while (1) {
        k_sleep(K_MSEC(300));
        err = adc_read(adc_channel.dev, &sequence);
        if (err < 0) {
            printk("read error (%d)\n", err);
            continue;
        }

        sound_value = (int32_t)buf;
        if (sound_value >= SENSOR_INVALID_VALUE) {
            printk("sound_value: invalid data %" PRIu32 "\n", sound_value);
            continue;
        }
        int sound_level = map(sound_value, 0, MAX_SENSORVALUE, 0, MIN_SENSORVALUE);
        printk("sound_value: %" PRIu32 " sound_level: %d\n", sound_value, sound_level);

        add_sound_segment(sound_level);
    }
}

K_THREAD_DEFINE(co2_thread_id, 1024, co2_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(sound_thread_id, 1024, sound_thread, NULL, NULL, NULL, 7, 0, 0);

int main(void) {
    if (!adc_is_ready_dt(&adc_channel)) {
        LOG_ERR("ADC controller device %s not ready", adc_channel.dev->name);
        return 0;
    }

    int err = adc_channel_setup_dt(&adc_channel);
    if (err < 0) {
        LOG_ERR("Could not setup channel (%d)", err);
        return 0;
    }

    if (!device_is_ready(uart_serial)) {
        LOG_ERR("UART device not found!");
        return 0;
    }

    if (!device_is_ready(led)) {
        LOG_ERR("LED device not ready!");
        return 0;
    }

    int ret = uart_irq_callback_user_data_set(uart_serial, serial_callback, NULL);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            LOG_ERR("Interrupt-driven UART API support not enabled");
        } else if (ret == -ENOSYS) {
            LOG_ERR("UART device does not support interrupt-driven API");
        } else {
            LOG_ERR("Error setting UART callback: %d", ret);
        }
        return 0;
    }
    uart_irq_rx_enable(uart_serial);

    LOG_INF("Initializing Bluetooth...");
    bluetooth_init();
    LOG_INF("Initializing Button1...");
    init_button();

    return 0;
}
