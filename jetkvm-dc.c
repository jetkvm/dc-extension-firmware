#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "driver_ina219.h"

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 16
#define UART_RX_PIN 17

#define PWR_PIN 4

#define INA219_I2C_SDA_PIN 8
#define INA219_I2C_SCL_PIN 9

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events)
{
    for (uint i = 0; i < 4; i++)
    {
        uint mask = (1 << i);
        if (events & mask)
        {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0')
            {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events)
            {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

static char event_str[128];

void gpio_callback(uint gpio, uint32_t events)
{
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
}

#define UART_BUF_SIZE 128
static char uart_buf[UART_BUF_SIZE];
static int uart_buf_pos = 0;


void on_uart_line(const char *line)
{
    printf("UART LINE: %s\n", line);
    if (strcmp(line, "PWR_ON\n") == 0) {
        gpio_put(PWR_PIN, 1);
        printf("Power ON\n");
    }
    else if (strcmp(line, "PWR_OFF\n") == 0) {
        gpio_put(PWR_PIN, 0);
        printf("Power OFF\n");
    }
}

void on_uart_rx()
{
    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);

        if (uart_buf_pos < UART_BUF_SIZE - 1)
        {
            uart_buf[uart_buf_pos++] = ch;
        }

        if (ch == '\n' || uart_buf_pos >= UART_BUF_SIZE - 1)
        {
            uart_buf[uart_buf_pos] = '\0';
            on_uart_line(uart_buf);
            uart_buf_pos = 0;
        }
    }
}

static ina219_handle_t ina219;

uint8_t ina219_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    printf("ina219: i2c read %d bytes from reg 0x%02x addr %d\n", len, reg, addr);
    int ret = i2c_write_blocking(i2c0, addr, &reg, 1, true);
    if (ret < 0) {
        printf("ina219: i2c write failed with error %d\n", ret);
        return 1;
    }
    ret = i2c_read_blocking(i2c0, addr, buf, len, false);
    if (ret < 0) {
        printf("ina219: i2c read failed with error %d\n", ret);
        return 1;
    }
    return 0;
}

uint8_t ina219_i2c_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t tmp_buf[len + 1];
    tmp_buf[0] = reg;
    memcpy(&tmp_buf[1], buf, len);
    int ret = i2c_write_blocking(i2c0, addr, tmp_buf, len + 1, false);
    if (ret < 0) {
        return 1;
    }
    return 0;
}

uint8_t ina219_i2c_init() {
    i2c_init(i2c0, 100000);
    gpio_set_function(INA219_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(INA219_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(INA219_I2C_SDA_PIN);
    gpio_pull_up(INA219_I2C_SCL_PIN);
    return 0;
}

uint8_t ina219_i2c_deinit() {
    // i2c_deinit(i2c_default);
    return 0;
}

uint8_t power_init() {
    uint8_t res;

    // Initialize handle
    DRIVER_INA219_LINK_INIT(&ina219, ina219_handle_t);

    // Link functions
    DRIVER_INA219_LINK_IIC_INIT(&ina219, ina219_i2c_init);
    DRIVER_INA219_LINK_IIC_DEINIT(&ina219, ina219_i2c_deinit); 
    DRIVER_INA219_LINK_IIC_READ(&ina219, ina219_i2c_read);
    DRIVER_INA219_LINK_IIC_WRITE(&ina219, ina219_i2c_write);
    DRIVER_INA219_LINK_DELAY_MS(&ina219, sleep_ms);
    DRIVER_INA219_LINK_DEBUG_PRINT(&ina219, printf);

ina219_i2c_init();
    // Set I2C address
    res = ina219_set_addr_pin(&ina219, 0x40);
    if (res != 0) {
        printf("ina219: set addr pin failed \n");
        return res;
    }
    /* set the r */
    res = ina219_set_resistance(&ina219, 0.01);
    if (res != 0)
    {
        printf("ina219: set resistance failed.\n");
        return res;
    }
    
    /* init */
    res = ina219_init(&ina219);
    if (res != 0)
    {
        printf("ina219: init failed.\n");
        return res;
    }
    
    /* set bus voltage range */
    res = ina219_set_bus_voltage_range(&ina219, INA219_BUS_VOLTAGE_RANGE_32V);
    if (res != 0)
    {
        printf("ina219: set bus voltage range failed.\n");        
        return res;
    }
    
    /* set bus voltage adc mode */
    res = ina219_set_bus_voltage_adc_mode(&ina219, INA219_ADC_MODE_12_BIT_1_SAMPLES);
    if (res != 0)
    {
        printf("ina219: set bus voltage adc mode failed.\n");
        return res;
    }

    /* set shunt voltage adc mode */
    res = ina219_set_shunt_voltage_adc_mode(&ina219, INA219_ADC_MODE_12_BIT_1_SAMPLES);
    if (res != 0)
    {
        printf("ina219: set shunt voltage adc mode failed.\n");        
        return res;
    }
    
    /* set shunt bus voltage continuous */
    res = ina219_set_mode(&ina219, INA219_MODE_SHUNT_BUS_VOLTAGE_CONTINUOUS);
    if (res != 0)
    {
        printf("ina219: set mode failed.\n");
        return res;
    }
    
    /* set pga */
    res = ina219_set_pga(&ina219, INA219_PGA_160_MV);
    if (res != 0)
    {
        printf("ina219: set pga failed.\n");        
        return res;
    }
    
    uint16_t calibration;
    /* calculate calibration */
    res = ina219_calculate_calibration(&ina219, (uint16_t *)&calibration);
    if (res != 0)
    {
        printf("ina219: calculate calibration failed.\n");
        return res;
    }
    
    /* set calibration */
    res = ina219_set_calibration(&ina219, calibration);
    if (res != 0)
    {
        printf("ina219: set calibration failed.\n");
        return res;
    }

    return 0;
}

uint8_t ina219_basic_read(float *mV, float *mA, float *mW)
{
    uint8_t res;
    int16_t s_raw;
    uint16_t u_raw;

    /* read bus voltage */
    res = ina219_read_bus_voltage(&ina219, (uint16_t *)&u_raw, mV);
    if (res != 0)
    {
        return 1;
    }
    
    /* read current */
    res = ina219_read_current(&ina219, (int16_t *)&s_raw, mA);
    if (res != 0)
    {
        return 1;
    }
    
    /* read power */
    res = ina219_read_power(&ina219, (uint16_t *)&u_raw, mW);
    if (res != 0)
    {
        return 1;
    }
    
    return 0;
}

int main()
{
    sleep_ms(1000);

    stdio_init_all();

    if (watchdog_caused_reboot())
    {
        printf("Rebooted by Watchdog!\n");
    }

    // watchdog_enable(8388, true);

    printf("Initing!\n");

    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    uart_init(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    gpio_init(PWR_PIN);
    gpio_set_dir(PWR_PIN, GPIO_OUT);
    gpio_put(PWR_PIN, 0);
    
    power_init();

    float voltage;
    float current;
    float power;

    while (true)
    {
        watchdog_update();

        printf("Uptime: %llu s\n", time_us_64() / 1000000);
        if (ina219_basic_read(&voltage, &current, &power) == 0) {
            int power_state = gpio_get(PWR_PIN);
            printf("Power state: %d\n", power_state);
            printf("Voltage: %.2f mV\n", voltage);
            printf("Current: %.2f mA\n", current);
            printf("Power: %.2f mW\n", power);
            char uart_msg[128];
            snprintf(uart_msg, sizeof(uart_msg), "%d;%.2f;%.2f;%.2f\n", power_state, voltage, current, power);
            uart_puts(UART_ID, uart_msg);
        } else {
            printf("Error reading INA219\n");
            uart_puts(UART_ID, "Error reading INA219\n");
        }

        sleep_ms(1000);
    }
}
