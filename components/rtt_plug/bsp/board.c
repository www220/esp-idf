#include <rtthread.h>
#include <rthw.h>
#include <rtm.h>
#include <rtdevice.h>
#include <board.h>

#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "esp_log.h"
#include "rom/uart.h"
#include "soc/uart_reg.h"
#include "soc/dport_reg.h"
#include "soc/uart_struct.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_clk.h"
#include "tcpip_adapter.h"
#include "eth_phy/phy_lan8720.h"

#include <thread_esp32.h>
#include <sys/time.h>

#define UART_FULL_THRESH_DEFAULT  (120)
#define UART_TOUT_THRESH_DEFAULT   (10)
#define UART_TX_IDLE_NUM_DEFAULT   (0)

#define UART_FIFO_LEN           (128)        /*!< Length of the hardware FIFO buffers */
#define UART_INTR_MASK          0x1ff        /*!< mask of all UART interrupts */
#define UART_LINE_INV_MASK      (0x3f << 19) /*!< TBD */
#define UART_BITRATE_MAX        5000000      /*!< Max bit rate supported by UART */
#define UART_PIN_NO_CHANGE      (-1)         /*!< Constant for uart_set_pin function which indicates that UART pin should not be changed */

#define UART_INVERSE_DISABLE  (0x0)            /*!< Disable UART signal inverse*/
#define UART_INVERSE_RXD   (UART_RXD_INV_M)    /*!< UART RXD input inverse*/
#define UART_INVERSE_CTS   (UART_CTS_INV_M)    /*!< UART CTS input inverse*/
#define UART_INVERSE_TXD   (UART_TXD_INV_M)    /*!< UART TXD output inverse*/
#define UART_INVERSE_RTS   (UART_RTS_INV_M)    /*!< UART RTS output inverse*/

/**
 * @brief UART word length constants
 */
typedef enum {
    UART_DATA_5_BITS = 0x0,    /*!< word length: 5bits*/
    UART_DATA_6_BITS = 0x1,    /*!< word length: 6bits*/
    UART_DATA_7_BITS = 0x2,    /*!< word length: 7bits*/
    UART_DATA_8_BITS = 0x3,    /*!< word length: 8bits*/
    UART_DATA_BITS_MAX = 0X4,
} uart_word_length_t;

/**
 * @brief UART stop bits number
 */
typedef enum {
    UART_STOP_BITS_1   = 0x1,  /*!< stop bit: 1bit*/
    UART_STOP_BITS_1_5 = 0x2,  /*!< stop bit: 1.5bits*/
    UART_STOP_BITS_2   = 0x3,  /*!< stop bit: 2bits*/
    UART_STOP_BITS_MAX = 0x4,
} uart_stop_bits_t;

/**
 * @brief UART peripheral number
 */
typedef enum {
    UART_NUM_0 = 0x0,  /*!< UART base address 0x3ff40000*/
    UART_NUM_1 = 0x1,  /*!< UART base address 0x3ff50000*/
    UART_NUM_2 = 0x2,  /*!< UART base address 0x3ff6E000*/
    UART_NUM_MAX,
} uart_port_t;

/**
 * @brief UART parity constants
 */
typedef enum {
    UART_PARITY_DISABLE = 0x0,   /*!< Disable UART parity*/
    UART_PARITY_EVEN = 0x2,     /*!< Enable UART even parity*/
    UART_PARITY_ODD  = 0x3      /*!< Enable UART odd parity*/
} uart_parity_t;

/**
 * @brief UART hardware flow control modes
 */
typedef enum {
    UART_HW_FLOWCTRL_DISABLE = 0x0,   /*!< disable hardware flow control*/
    UART_HW_FLOWCTRL_RTS     = 0x1,   /*!< enable RX hardware flow control (rts)*/
    UART_HW_FLOWCTRL_CTS     = 0x2,   /*!< enable TX hardware flow control (cts)*/
    UART_HW_FLOWCTRL_CTS_RTS = 0x3,   /*!< enable hardware flow control*/
    UART_HW_FLOWCTRL_MAX     = 0x4,
} uart_hw_flowcontrol_t;

#define UART_INTR_NUM_0           17
#define UART_INTR_NUM_1           18
#define UART_INTR_NUM_2           19
static DRAM_ATTR uart_dev_t* const UART[UART_NUM_MAX] = {&UART0, &UART1, &UART2};
static void IRAM_ATTR esp32_clr_rxfifo(int uart_num);

struct esp32_uart
{
    int num;
    int irq;
};

static rt_err_t esp32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct esp32_uart* uart = (struct esp32_uart *)serial->parent.user_data;
    int uart_num = uart->num;
    
    int uart_clk_freq;
    if (UART[uart_num]->conf0.tick_ref_always_on == 0) {
        /* this UART has been configured to use REF_TICK */
        uart_clk_freq = REF_CLK_FREQ;
    } else {
        uart_clk_freq = esp_clk_apb_freq();
    }
    uint32_t clk_div = (((uart_clk_freq) << 4) / cfg->baud_rate);
    UART[uart_num]->clk_div.div_int = clk_div >> 4;
    UART[uart_num]->clk_div.div_frag = clk_div & 0xf;
    //
    uint32_t databits = UART_DATA_8_BITS;
    switch (cfg->data_bits)
    {
    case DATA_BITS_7:
        databits = UART_DATA_7_BITS;
        break;
    }
    UART[uart_num]->conf0.bit_num = databits;
    //
    uint32_t parity = UART_PARITY_DISABLE;
    switch (cfg->parity)
    {
    case PARITY_EVEN:
        parity = UART_PARITY_EVEN;
        break;
    case PARITY_ODD:
        parity = UART_PARITY_ODD;
        break;
    }
    UART[uart_num]->conf0.parity = parity & 0x1;
    UART[uart_num]->conf0.parity_en = (parity >> 1) & 0x1;
    //
    uint32_t stop_bit = UART_STOP_BITS_1;
    switch (cfg->stop_bits)
    {
    case STOP_BITS_2:
        stop_bit = UART_STOP_BITS_2;
        break;
    }
    if (stop_bit == UART_STOP_BITS_2) {
        stop_bit = UART_STOP_BITS_1;
        UART[uart_num]->rs485_conf.dl1_en = 1;
    } else {
        UART[uart_num]->rs485_conf.dl1_en = 0;
    }
    UART[uart_num]->conf0.stop_bit_num = stop_bit;
	//
    CLEAR_PERI_REG_MASK(UART_CONF0_REG(uart_num), UART_LINE_INV_MASK);
    SET_PERI_REG_MASK(UART_CONF0_REG(uart_num), UART_INVERSE_DISABLE);
    //
	UART[uart_num]->conf1.rx_flow_thrhd = UART_FULL_THRESH_DEFAULT;
    if(cfg->reserved & UART_HW_FLOWCTRL_RTS || uart_num == 2)
        UART[uart_num]->conf1.rx_flow_en = 1;
    else
        UART[uart_num]->conf1.rx_flow_en = 0;
    if(cfg->reserved & UART_HW_FLOWCTRL_CTS)
        UART[uart_num]->conf0.tx_flow_en = 1;
    else
        UART[uart_num]->conf0.tx_flow_en = 0;
    UART[uart_num]->idle_conf.tx_idle_num = UART_TX_IDLE_NUM_DEFAULT;
    //
    esp32_clr_rxfifo(uart_num);
	ESP_INTR_DISABLE(uart->irq);
    return RT_EOK;
}

static rt_err_t esp32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct esp32_uart* uart = (struct esp32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        esp32_clr_rxfifo(uart->num);
        ESP_INTR_DISABLE(uart->irq);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        esp32_clr_rxfifo(uart->num);
        ESP_INTR_ENABLE(uart->irq);
        break;
    }

    return RT_EOK;
}

static void IRAM_ATTR esp32_clr_rxfifo(int uart_num)
{
    // we read the data out and make `fifo_len == 0 && rd_addr == wr_addr`.
    while(UART[uart_num]->status.rxfifo_cnt != 0 
        || (UART[uart_num]->mem_rx_status.wr_addr != UART[uart_num]->mem_rx_status.rd_addr))
    {
        READ_PERI_REG(UART_FIFO_REG(uart_num));
    }
}

static int IRAM_ATTR esp32_putc(struct rt_serial_device *serial, char c)
{
    struct esp32_uart* uart = (struct esp32_uart *)serial->parent.user_data;
    int uart_num = uart->num;

    while (true) {
        uint8_t tx_fifo_cnt = UART[uart_num]->status.txfifo_cnt;
        if (tx_fifo_cnt < UART_FIFO_LEN)
            break;
    }
    WRITE_PERI_REG(UART_FIFO_AHB_REG(uart_num), c);
    return 1;
}

static int IRAM_ATTR esp32_getc(struct rt_serial_device *serial)
{
    struct esp32_uart* uart = (struct esp32_uart *)serial->parent.user_data;
    return (READ_PERI_REG(UART_FIFO_REG(uart->num))) & 0xff;
}

//internal isr handler for default driver code.
static void IRAM_ATTR uart_rx_intr_handler_default(void *param)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)param;
    struct esp32_uart* uart = (struct esp32_uart *)serial->parent.user_data;
    uint8_t uart_num = uart->num;
    uint32_t uart_intr_status = UART[uart_num]->int_st.val;
    int rx_fifo_len = 0;

    while(uart_intr_status != 0x0) 
    {
        if((uart_intr_status & UART_RXFIFO_TOUT_INT_ST_M) 
            || (uart_intr_status & UART_RXFIFO_FULL_INT_ST_M)) {
            rx_fifo_len = UART[uart_num]->status.rxfifo_cnt;
            if (rx_fifo_len <= 0 || rx_fifo_len > 128)
                esp32_clr_rxfifo(uart_num);
            else
                rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND, rx_fifo_len);
            UART[uart_num]->int_clr.rxfifo_tout = 1;
            UART[uart_num]->int_clr.rxfifo_full = 1;
        } else if(uart_intr_status & UART_RXFIFO_OVF_INT_ST_M) {
            esp32_clr_rxfifo(uart_num);
            UART[uart_num]->int_clr.rxfifo_ovf = 1;
        } else if(uart_intr_status & UART_BRK_DET_INT_ST_M) {
            UART[uart_num]->int_clr.brk_det = 1;
        } else if(uart_intr_status & UART_PARITY_ERR_INT_ST_M ) {
            UART[uart_num]->int_clr.parity_err = 1;
        } else if(uart_intr_status & UART_FRM_ERR_INT_ST_M) {
            UART[uart_num]->int_clr.frm_err = 1;
        } else {
            /*simply clear all other intr status*/
            UART[uart_num]->int_clr.val = uart_intr_status; 
        }
        uart_intr_status = UART[uart_num]->int_st.val;
    }
}

static DRAM_ATTR const struct rt_uart_ops esp32_uart_ops =
{
    esp32_configure,
    esp32_control,
    esp32_putc,
    esp32_getc,
    RT_NULL,
};

#ifdef RT_USING_UART0
static struct rt_serial_device serial0;
static struct esp32_uart uart0 = { UART_NUM_0, UART_INTR_NUM_0 };
#endif

#ifdef RT_USING_UART2
static struct rt_serial_device serial1;
static struct esp32_uart uart1 = { UART_NUM_1, UART_INTR_NUM_1 };
#endif

#ifdef RT_USING_UART1
static struct rt_serial_device serial2;
static struct esp32_uart uart2 = { UART_NUM_2, UART_INTR_NUM_2 };
#endif

void rt_hw_usart_init() 
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	config.bufsz = 4096;
	config.reserved = UART_HW_FLOWCTRL_DISABLE;

#ifdef RT_USING_UART0
	periph_module_enable(PERIPH_UART0_MODULE);
	esp32_clr_rxfifo(uart0.num);

	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_1)) rtc_gpio_deinit(GPIO_NUM_1);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_3)) rtc_gpio_deinit(GPIO_NUM_3);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_23)) rtc_gpio_deinit(GPIO_NUM_23);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_1], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_1, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_1, U0TXD_OUT_IDX, 0, 0);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_3], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_3, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_3, GPIO_MODE_INPUT);
	gpio_matrix_in(GPIO_NUM_3, U0RXD_IN_IDX, 0);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_23], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_23, GPIO_FLOATING);
	gpio_set_level(GPIO_NUM_23, 0);
	gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT_OD);
	gpio_matrix_out(GPIO_NUM_23, SIG_GPIO_OUT_IDX, 0, 0);

	intr_matrix_set(xPortGetCoreID(), ETS_UART0_INTR_SOURCE, uart0.irq);
	xt_set_interrupt_handler(uart0.irq, uart_rx_intr_handler_default, &serial0);
	UART[uart0.num]->int_clr.val = UART_INTR_MASK;
	UART[uart0.num]->conf1.rx_tout_thrhd = UART_TOUT_THRESH_DEFAULT;
	UART[uart0.num]->conf1.rx_tout_en = 1;
	UART[uart0.num]->conf1.rxfifo_full_thrhd = UART_FULL_THRESH_DEFAULT;
	UART[uart0.num]->int_ena.val = UART_RXFIFO_FULL_INT_ENA_M
							| UART_RXFIFO_TOUT_INT_ENA_M
							| UART_RXFIFO_OVF_INT_ENA_M
							| UART_FRM_ERR_INT_ENA_M
							| UART_BRK_DET_INT_ENA_M
							| UART_PARITY_ERR_INT_ENA_M;
	ESP_INTR_DISABLE(uart0.irq);

    serial0.ops = &esp32_uart_ops;
    serial0.config = config;
    rt_hw_serial_register(&serial0, "uart0", 
        RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart0);
#endif
#ifdef RT_USING_UART2
	periph_module_enable(PERIPH_UART1_MODULE);
	esp32_clr_rxfifo(uart1.num);

	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_12)) rtc_gpio_deinit(GPIO_NUM_12);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_34)) rtc_gpio_deinit(GPIO_NUM_34);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_5)) rtc_gpio_deinit(GPIO_NUM_5);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_12], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_12, U1TXD_OUT_IDX, 0, 0);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_34], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_34, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_34, GPIO_MODE_INPUT);
	gpio_matrix_in(GPIO_NUM_34, U1RXD_IN_IDX, 0);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_5], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_5, 0);
	gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_5, SIG_GPIO_OUT_IDX, 0, 0);

    intr_matrix_set(xPortGetCoreID(), ETS_UART1_INTR_SOURCE, uart1.irq);
    xt_set_interrupt_handler(uart1.irq, uart_rx_intr_handler_default, &serial1);
    UART[uart1.num]->int_clr.val = UART_INTR_MASK;
    UART[uart1.num]->conf1.rx_tout_thrhd = UART_TOUT_THRESH_DEFAULT;
    UART[uart1.num]->conf1.rx_tout_en = 1;
    UART[uart1.num]->conf1.rxfifo_full_thrhd = UART_FULL_THRESH_DEFAULT;
    UART[uart1.num]->int_ena.val = UART_RXFIFO_FULL_INT_ENA_M
                            | UART_RXFIFO_TOUT_INT_ENA_M
                            | UART_RXFIFO_OVF_INT_ENA_M
                            | UART_FRM_ERR_INT_ENA_M
                            | UART_BRK_DET_INT_ENA_M
                            | UART_PARITY_ERR_INT_ENA_M;
    ESP_INTR_DISABLE(uart1.irq);
	
    serial1.ops = &esp32_uart_ops;
    serial1.config = config;
    rt_hw_serial_register(&serial1, "uart2", 
        RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart1);
#endif
#ifdef RT_USING_UART1
	periph_module_enable(PERIPH_UART2_MODULE);
	esp32_clr_rxfifo(uart2.num);

	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_2)) rtc_gpio_deinit(GPIO_NUM_2);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_18)) rtc_gpio_deinit(GPIO_NUM_18);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_0)) rtc_gpio_deinit(GPIO_NUM_0);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_17)) rtc_gpio_deinit(GPIO_NUM_17);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_2], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_2, U2TXD_OUT_IDX, 0, 0);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_18], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_18, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_18, GPIO_MODE_INPUT);
	gpio_matrix_in(GPIO_NUM_18, U2RXD_IN_IDX, 0);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_0], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_0, U2RTS_OUT_IDX, 0, 0);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_17], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_17, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_17, GPIO_MODE_INPUT);
	gpio_matrix_in(GPIO_NUM_17, U2CTS_IN_IDX, 0);

    intr_matrix_set(xPortGetCoreID(), ETS_UART2_INTR_SOURCE, uart2.irq);
    xt_set_interrupt_handler(uart2.irq, uart_rx_intr_handler_default, &serial2);
    UART[uart2.num]->int_clr.val = UART_INTR_MASK;
    UART[uart2.num]->conf1.rx_tout_thrhd = UART_TOUT_THRESH_DEFAULT;
    UART[uart2.num]->conf1.rx_tout_en = 1;
    UART[uart2.num]->conf1.rxfifo_full_thrhd = UART_FULL_THRESH_DEFAULT;
    UART[uart2.num]->int_ena.val = UART_RXFIFO_FULL_INT_ENA_M
                            | UART_RXFIFO_TOUT_INT_ENA_M
                            | UART_RXFIFO_OVF_INT_ENA_M
                            | UART_FRM_ERR_INT_ENA_M
                            | UART_BRK_DET_INT_ENA_M
                            | UART_PARITY_ERR_INT_ENA_M;
    ESP_INTR_DISABLE(uart2.irq);
	
    serial2.ops = &esp32_uart_ops;
    serial2.config = config;
    rt_hw_serial_register(&serial2, "uart1", 
        RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX, &uart2);
#endif
}

rt_size_t rt_device_write_485(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct esp32_uart* uart = (struct esp32_uart *)dev->user_data;
    uint8_t uart_num = uart->num;
	
	if (uart_num == 1){
		gpio_set_level(GPIO_NUM_5, 1);
		int ret = rt_device_write(dev, pos, buffer, size);
		while (UART[uart_num]->status.txfifo_cnt);
		uart_tx_wait_idle(uart_num);
		gpio_set_level(GPIO_NUM_5, 0);
    	return ret;
	}else{
    	return rt_device_write(dev, pos, buffer, size);
	}
}

#define I2C_MASTER_SDA_GPIO GPIO_NUM_13
#define I2C_MASTER_SCL_GPIO GPIO_NUM_4
static void gpio_set_sda(void *data, rt_int32_t state)
{
	gpio_set_level(I2C_MASTER_SDA_GPIO, state);
}

static void gpio_set_scl(void *data, rt_int32_t state)
{
	gpio_set_level(I2C_MASTER_SCL_GPIO, state);
}

static rt_int32_t gpio_get_sda(void *data)
{
   return gpio_get_level(I2C_MASTER_SDA_GPIO);
}

static rt_int32_t gpio_get_scl(void *data)
{
    return gpio_get_level(I2C_MASTER_SCL_GPIO);
}

static void gpio_udelay(rt_uint32_t us)
{
    ets_delay_us(us);
}

static const struct rt_i2c_bit_ops _i2c_bit_ops =
{
    NULL,
    gpio_set_sda,
    gpio_set_scl,
    gpio_get_sda,
    gpio_get_scl,
    gpio_udelay,
    10,
    10
};
static struct rt_i2c_bus_device i2c_device;

void rt_hw_i2c_init()
{
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_4)) rtc_gpio_deinit(GPIO_NUM_4);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_4], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_4, 1);
	gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT_OUTPUT_OD);
	gpio_matrix_out(GPIO_NUM_4, SIG_GPIO_OUT_IDX, 0, 0);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_13)) rtc_gpio_deinit(GPIO_NUM_13);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_13], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_13, 1);
	gpio_set_direction(GPIO_NUM_13, GPIO_MODE_INPUT_OUTPUT_OD);
	gpio_matrix_out(GPIO_NUM_13, SIG_GPIO_OUT_IDX, 0, 0);
	
    i2c_device.priv = (void *)&_i2c_bit_ops;
    rt_i2c_bit_add_bus(&i2c_device, "i2c");
}

static inline unsigned int bcd2bin(rt_uint8_t val)
{
	return ((val) & 0x0f) + ((val) >> 4) * 10;
}

static inline rt_uint8_t bin2bcd (unsigned int val)
{
	return (((val / 10) << 4) | (val % 10));
}

int init_ds3231(void)
{
    uint8_t d[2] = { 0x0e, 0x0c };
    if (rt_i2c_master_send(&i2c_device, 0x68, 0, d, 2) != 2)
		return 0;
	return 1;
}

int load_ds3231(void)
{
    uint8_t sec[7] = { 0x00 };
    if (rt_i2c_master_send(&i2c_device, 0x68, 0, sec, 1) != 1)
        return 0;
    if (rt_i2c_master_recv(&i2c_device, 0x68, 0, sec, 7) != 7)
        return 0;
	struct tm t;
	t.tm_sec = bcd2bin(sec[0]&0x7f);
	t.tm_min = bcd2bin(sec[1]&0x7f);
	t.tm_hour = bcd2bin(sec[2]&0x3f);
	t.tm_mday = bcd2bin(sec[4]&0x3f);
	t.tm_mon = bcd2bin(sec[5]&0x1f)-1;
	t.tm_year = 100+bcd2bin(sec[6]);
    struct timeval tv_set;
    tv_set.tv_sec = mktime(&t);
    tv_set.tv_usec = 0;
    settimeofday(&tv_set, NULL);
	printf("time updated %s",ctime(&tv_set.tv_sec));
    return 1;
}

int save_ds3231(int time)
{
	struct tm t;
	t = *localtime((time_t *)&time);
	rt_uint8_t sec[8] = {
		0x00,
		bin2bcd(t.tm_sec),
		bin2bcd(t.tm_min),
		bin2bcd(t.tm_hour),
		bin2bcd(t.tm_wday+1),
		bin2bcd(t.tm_mday),
		bin2bcd(t.tm_mon+1),
		bin2bcd(t.tm_year%100)};
	if (rt_i2c_master_send(&i2c_device, 0x68, 0, sec, 8) != 8)
		return 0;
	return 1;
}

void init_rtc(void)
{
	int i;
	for (i=0; i<3; i++)
	{
		rt_tick_t tnow = rt_tick_get();
		rt_kprintf("init ds3231\n");
		if (init_ds3231()!=0)
		{
			rt_kprintf("init ds3231 success:%d\n", rt_tick_get()-tnow);
			break;
		}
		rt_kprintf("init ds3231 timeout:%d\n", rt_tick_get()-tnow);
	}
}

void loadrtc(void)
{
	int i;
	for (i=0; i<3; i++)
	{
		rt_tick_t tnow = rt_tick_get();
		rt_kprintf("read ds3231\n");
		if (load_ds3231()!=0)
		{
			rt_kprintf("read ds3231 success:%d\n", rt_tick_get()-tnow);
			break;
		}
		rt_kprintf("read ds3231 timeout:%d\n", rt_tick_get()-tnow);
	}
}

void savertc(int time)
{
	int i;
	for (i=0; i<3; i++)
	{
		rt_tick_t tnow = rt_tick_get();
		rt_kprintf("write ds3231\n");
		if (save_ds3231(time)!=0)
		{
			rt_kprintf("write ds3231 success:%d\n", rt_tick_get()-tnow);
			break;
		}
		rt_kprintf("write ds3231 timeout:%d\n", rt_tick_get()-tnow);
	}
}

#define ETH_PHY_ADDR  0
#define PIN_SMI_MDC   33
#define PIN_SMI_MDIO  32
static void eth_phy_init(void)
{
	gpio_set_level(GPIO_NUM_14, 0);
	rt_thread_delay(M2T(100));
	gpio_set_level(GPIO_NUM_14, 1);	
	rt_thread_delay(M2T(100));
	phy_lan8720_init();
}

static void eth_gpio_config_rmii(void)
{
	phy_rmii_configure_data_interface_pins();

	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_33)) rtc_gpio_deinit(GPIO_NUM_33);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_32)) rtc_gpio_deinit(GPIO_NUM_32);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_33], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_33, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_33, EMAC_MDC_O_IDX, 0, 0);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_32], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_32, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT_OUTPUT);
	gpio_matrix_in(GPIO_NUM_32, EMAC_MDI_I_IDX, 0);
	gpio_matrix_out(GPIO_NUM_32, EMAC_MDO_O_IDX, 0, 0);

	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_14)) rtc_gpio_deinit(GPIO_NUM_14);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_14], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_14, 1);
	gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_14, SIG_GPIO_OUT_IDX, 0, 0);
}

int rt_hw_eth_init(void)
{
    eth_config_t config = phy_lan8720_default_ethernet_config;
    config.phy_addr = ETH_PHY_ADDR;
    config.phy_init = eth_phy_init,
    config.gpio_config = eth_gpio_config_rmii;
    config.tcpip_input = tcpip_adapter_eth_input;
    config.clock_mode = ETH_CLOCK_GPIO16_OUT;
    config.flow_ctrl_enable = false;

    int ret = ESP_FAIL;
    if(esp_eth_init(&config) == ESP_OK)
    {
        esp_eth_enable();
        ret = ESP_OK;
    }
    return ret;
}

struct rb
{
    rt_uint16_t read_index, write_index;
    rt_uint8_t *buffer_ptr;
    rt_uint16_t buffer_size;
};
struct telnet_device
{
    struct rt_device device;
    struct rt_mutex mutex;
    struct rb rx_buf;
    struct rb tx_buf;
};
struct telnet_device telnet;

/* 一个环形buffer的实现 */
/* 初始化环形buffer，size指的是buffer的大小。注：这里并没对数据地址对齐做处理 */
static void rb_init(struct rb* rb, rt_uint8_t *pool, rt_uint16_t size)
{
    RT_ASSERT(rb != RT_NULL);

    /* 对读写指针清零*/
    rb->read_index = rb->write_index = 0;

    /* 环形buffer的内存数据块 */
    rb->buffer_ptr = pool;
    rb->buffer_size = size;
}

/* 向环形buffer中写入一个字符 */
static rt_size_t rb_putchar(struct rb* rb, const rt_uint8_t ch)
{
    rt_uint16_t next;

    /* 判断是否有多余的空间 */
    next = rb->write_index + 1;
    if (next >= rb->buffer_size) next = 0;

    if (next == rb->read_index) return 0;

    /* 放入字符 */
    rb->buffer_ptr[rb->write_index] = ch;
    rb->write_index = next;

    return 1;
}

/* 从环形buffer中读出数据 */
static rt_size_t rb_get(struct rb* rb, rt_uint8_t *ptr, rt_uint16_t length)
{
    rt_size_t size;

    /* 判断是否有足够的数据 */
    if (rb->read_index > rb->write_index)
        size = rb->buffer_size - rb->read_index + rb->write_index;
    else
        size = rb->write_index - rb->read_index;

    /* 没有足够的数据 */
    if (size == 0) return 0;

    /* 数据不够指定的长度，取环形buffer中实际的长度 */
    if (size < length) length = size;

    if (rb->read_index > rb->write_index)
    {
        if (rb->buffer_size - rb->read_index > length)
        {
            /* read_index的数据足够多，直接复制 */
			if (ptr)
				rt_memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
            rb->read_index += length;
        }
        else
        {
            /* read_index的数据不够，需要分段复制 */
			if (ptr)
			{
	            rt_memcpy(ptr, &rb->buffer_ptr[rb->read_index],
	                   rb->buffer_size - rb->read_index);
	            rt_memcpy(&ptr[rb->buffer_size - rb->read_index], &rb->buffer_ptr[0],
	                   length - rb->buffer_size + rb->read_index);
			}
            rb->read_index = length - rb->buffer_size + rb->read_index;
        }
    }
    else
    {
        /* read_index要比write_index小，总的数据量够（前面已经有总数据量的判断），直接复制出数据 */
		if (ptr)
	        rt_memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
        rb->read_index += length;
    }

    return length;
}

/* RT-Thread Device Driver Interface */
static rt_err_t telnet_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t telnet_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t telnet_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t telnet_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_size_t result;

    /* read from rx ring buffer */
    rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
    result = rb_get(&telnet.rx_buf, buffer, size);
    rt_mutex_release(&telnet.mutex);

    return result;
}

static rt_size_t telnet_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_size_t i,result = 0;
    rt_uint8_t *buf = (rt_uint8_t *)buffer;

    /* write to tx ring buffer */
    rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
    for (i=0; i<size; i++)
    {
        if (buf[i] == '\n')
        {
            char ch = '\r';
            if (rb_putchar(&telnet.tx_buf,ch) == 0)
            {
                rb_get(&telnet.tx_buf, NULL, 128);
                if (rb_putchar(&telnet.tx_buf,ch) == 0)
                    break;
            }
        }
        if (rb_putchar(&telnet.tx_buf,buf[i]) == 0)
        {
            rb_get(&telnet.tx_buf, NULL, 128);
			if (rb_putchar(&telnet.tx_buf,buf[i]) == 0)
				break;
        }
        result++;
    }
    rt_mutex_release(&telnet.mutex);

    return result;
}

static rt_err_t telnet_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    return RT_EOK;
}

rt_size_t telnet_recv(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_size_t result;

    /* read from tx ring buffer */
    rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
    result = rb_get(&telnet.tx_buf, buffer, size);
    rt_mutex_release(&telnet.mutex);

    return result;
}

rt_size_t telnet_send(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_size_t i,result = 0;
    rt_uint8_t *buf = (rt_uint8_t *)buffer;

    /* write to rx ring buffer */
    rt_mutex_take(&telnet.mutex, RT_WAITING_FOREVER);
    for (i=0; i<size; i++)
    {
        if (rb_putchar(&telnet.rx_buf,buf[i]) == 0)
        {
            rb_get(&telnet.rx_buf, NULL, 128);
			if (rb_putchar(&telnet.rx_buf,buf[i]) == 0)
				break;
        }
        result++;
    }
    rt_mutex_release(&telnet.mutex);

	/* indicate there are reception data */
    if ((result > 0) && (telnet.device.rx_indicate != RT_NULL))
        telnet.device.rx_indicate(&telnet.device, result);
    return result;
}

int rt_hw_telnet_init(void)
{
    rt_memset(&telnet, 0, sizeof(telnet));
    rb_init(&telnet.rx_buf, rt_malloc(1024), 1024);
    rb_init(&telnet.tx_buf, rt_malloc(4096), 4096);
    rt_mutex_init(&telnet.mutex, "telnet", RT_IPC_FLAG_FIFO);

    /* register telnet device */
    telnet.device.type     = RT_Device_Class_Char;
    telnet.device.init     = telnet_init;
    telnet.device.open     = telnet_open;
    telnet.device.close    = telnet_close;
    telnet.device.read     = telnet_read;
    telnet.device.write    = telnet_write;
    telnet.device.control  = telnet_control;

    /* no private */
    telnet.device.user_data = &telnet;

    /* register telnet device */
    rt_device_register(&telnet.device, "telnet", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STREAM);
    return ESP_OK;
}

volatile int sys_stauts = -1;
volatile int uptime_count = 0;
volatile unsigned char PZ[16] = {0};
char RTT_USER[16] = {"admin"};
char RTT_PASS[36] = {"21232f297a57a5a743894a0e4a801fc3"};
char RTT_NTP[32] = {"www.baidu.com"};
unsigned long long RTT_PRJNO = 0;

static void rt_hw_write_char(char c)
{
    rt_device_t dev = rt_console_get_device();
    if (dev == NULL)
        ets_write_char_uart(c);
    else
        rt_device_write(dev, 0, &c, 1);
}
void rt_hw_board_init(void)
{
	/* initialize led gpio */
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_15)) rtc_gpio_deinit(GPIO_NUM_15);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_15], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_15, 1);
	gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_15, SIG_GPIO_OUT_IDX, 0, 0);
	/* initialize key gpio */
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_35)) rtc_gpio_deinit(GPIO_NUM_35);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_35], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_35, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);
    /* initialize uart */
    rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif
    ets_install_putc1(rt_hw_write_char);
    ets_install_putc2(NULL);
	/* initialize i2c */
    rt_hw_i2c_init();

	/* set cst */
    putenv("TZ=CST-8:00");
    tzset();
}

#ifdef RT_USING_LWIP
#include <lwip/tcp.h>
#include <lwip/priv/tcp_priv.h>
#include <lwip/netif.h>
#include <lwip/inet.h>
#include <lwip/dns.h>
#include <string.h>

void set_if(char* netif_name, char* ip_addr, char* gw_addr, char* nm_addr)
{
    ip_addr_t addr;
    const ip4_addr_t *ip = &addr.u_addr.ip4;
    struct netif * netif = netif_list;

    if(rt_strlen(netif_name) > sizeof(netif->name))
    {
        rt_kprintf("network interface name too long!\r\n");
        return;
    }

    while(netif != RT_NULL)
    {
        if(rt_strncmp(netif_name, netif->name, sizeof(netif->name)) == 0)
            break;

        netif = netif->next;
        if( netif == RT_NULL )
        {
            rt_kprintf("network interface: %s not found!\r\n", netif_name);
            return;
        }
    }

    /* set ip address */
    if ((ip_addr != RT_NULL) && ipaddr_aton(ip_addr, &addr))
    {
        netif_set_ipaddr(netif, ip);
    }

    /* set gateway address */
    if ((gw_addr != RT_NULL) && ipaddr_aton(gw_addr, &addr))
    {
        netif_set_gw(netif, ip);
    }

    /* set netmask address */
    if ((nm_addr != RT_NULL) && ipaddr_aton(nm_addr, &addr))
    {
        netif_set_netmask(netif, ip);
    }
}

void list_if(void)
{
    rt_ubase_t index;
    struct netif * netif;

    netif = netif_list;
    while( netif != RT_NULL )
    {
        rt_kprintf("network interface: %c%c%s\n",
                   netif->name[0],
                   netif->name[1],
                   (netif == netif_default)?" (Default)":"");
        rt_kprintf("MTU: %d\n", netif->mtu);
        rt_kprintf("MAC: ");
        for (index = 0; index < netif->hwaddr_len; index ++)
            rt_kprintf("%02x ", netif->hwaddr[index]);
        rt_kprintf("\nFLAGS:");
        if (netif->flags & NETIF_FLAG_UP) rt_kprintf(" UP");
        else rt_kprintf(" DOWN");
        if (netif->flags & NETIF_FLAG_LINK_UP) rt_kprintf(" LINK_UP");
        else rt_kprintf(" LINK_DOWN");
        if (netif->flags & NETIF_FLAG_ETHARP) rt_kprintf(" ETHARP");
        if (netif->flags & NETIF_FLAG_IGMP) rt_kprintf(" IGMP");
        rt_kprintf("\n");
        rt_kprintf("ip address: %s\n", ipaddr_ntoa(&(netif->ip_addr)));
        rt_kprintf("gw address: %s\n", ipaddr_ntoa(&(netif->gw)));
        rt_kprintf("net mask  : %s\n", ipaddr_ntoa(&(netif->netmask)));
        rt_kprintf("\r\n");

        netif = netif->next;
    }

#if LWIP_DNS
	extern int cmd_dns(int argc, char **argv);
	cmd_dns(1, NULL);
#endif /**< #if LWIP_DNS */
}

void list_tcps(void)
{
    rt_uint32_t num = 0;
    struct tcp_pcb *pcb;
    char local_ip_str[16];
    char remote_ip_str[16];

    extern struct tcp_pcb *tcp_active_pcbs;
    extern union tcp_listen_pcbs_t tcp_listen_pcbs;
    extern struct tcp_pcb *tcp_tw_pcbs;

    rt_kprintf("Active PCB states:\n");
    for(pcb = tcp_active_pcbs; pcb != NULL; pcb = pcb->next)
    {
        strcpy(local_ip_str, ipaddr_ntoa(&(pcb->local_ip)));
        strcpy(remote_ip_str, ipaddr_ntoa(&(pcb->remote_ip)));

        rt_kprintf("#%d %s:%d <==> %s:%d snd_nxt 0x%08X rcv_nxt 0x%08X ",
                   num++,
                   local_ip_str,
                   pcb->local_port,
                   remote_ip_str,
                   pcb->remote_port,
                   pcb->snd_nxt,
                   pcb->rcv_nxt);
        rt_kprintf("state: %s\n", tcp_debug_state_str(pcb->state));
    }

    rt_kprintf("Listen PCB states:\n");
    num = 0;
    for(pcb = (struct tcp_pcb *)tcp_listen_pcbs.pcbs; pcb != NULL; pcb = pcb->next)
    {
        rt_kprintf("#%d local port %d ", num++, pcb->local_port);
        rt_kprintf("state: %s\n", tcp_debug_state_str(pcb->state));
    }

    rt_kprintf("TIME-WAIT PCB states:\n");
    num = 0;
    for(pcb = tcp_tw_pcbs; pcb != NULL; pcb = pcb->next)
    {
        strcpy(local_ip_str, ipaddr_ntoa(&(pcb->local_ip)));
        strcpy(remote_ip_str, ipaddr_ntoa(&(pcb->remote_ip)));

        rt_kprintf("#%d %s:%d <==> %s:%d snd_nxt 0x%08X rcv_nxt 0x%08X ",
                   num++,
                   local_ip_str,
                   pcb->local_port,
                   remote_ip_str,
                   pcb->remote_port,
                   pcb->snd_nxt,
                   pcb->rcv_nxt);
        rt_kprintf("state: %s\n", tcp_debug_state_str(pcb->state));
    }
}

#if LWIP_DNS
void set_dns(char* dns_server)
{
    ip_addr_t addr;
    if ((dns_server != RT_NULL) && ipaddr_aton(dns_server, &addr))
    {
        dns_setserver(DNS_FALLBACK_SERVER_INDEX, &addr);
    }
}
#endif
#endif

#ifdef RT_USING_FINSH
#include <finsh.h>

int pin_cfg(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("pin_cfg pin_num pin_dir");
        return 1;
    }
	int io_num = atoi(argv[1]);
	int io_dir = atoi(argv[2]);	
	if(RTC_GPIO_IS_VALID_GPIO(io_num)) rtc_gpio_deinit(io_num);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[io_num], PIN_FUNC_GPIO);
	gpio_set_pull_mode(io_num, GPIO_PULLUP_PULLDOWN);
	if (io_dir)
	{
		gpio_set_direction(io_num, GPIO_MODE_OUTPUT);
		gpio_matrix_out(io_num, SIG_GPIO_OUT_IDX, 0, 0);
	}
	else
	{
		gpio_set_direction(io_num, GPIO_MODE_INPUT);
	}
    return 0;
}

int pin_val(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("pin_val pin_num [pin_val]");
        return 1;
    }
	int io_num = atoi(argv[1]);
	if (argc == 2)
	{
		int val = gpio_get_level(io_num);
		rt_kprintf("pin_val %d=%d\n",io_num,val);
	}
	else
	{
		int io_val = atoi(argv[2]);
		gpio_set_level(io_num, io_val);
	}
    return 0;
}

int uart_test(int argc, char **argv)
{
    int i,j;
    int openflag = 0;
    int count = 10;
    char chbuf[100];
    rt_device_t dev;
    if (argc < 3)
    {
        rt_kprintf("uart_test uart[0,1,2] test\n");
        return 1;
    }

    dev = rt_device_find(argv[1]);
    if (dev == NULL)
    {
        rt_kprintf("%s not find\n", argv[1]);
        return 0;
    }
    if (!(dev->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        openflag = 1;
    }
    if (argc > 3) count = atoi(argv[3]);
    for (i=0; i<count; i++)
    {
        int recvflag = 0;
        rt_kprintf("send:%s\r\n", argv[2]);
        rt_device_write_485(dev, 0, argv[2], strlen(argv[2]));
        rt_kprintf("read:");
        for (j=0; j<10; j++)
        {
            int len = rt_device_read(dev, 0, chbuf, 99);
            if (len > 0)
            {
                chbuf[len] = 0;
                rt_kprintf(chbuf);
                recvflag = 1;
            }
            rt_thread_delay(100);
        }
        rt_kprintf("%s\r\n",(recvflag==0)?"timeout...":"");
    }
    if (openflag)
    {
        rt_device_close(dev);
        openflag = 0;
    }
    return 0;
}

MSH_CMD_EXPORT_ALIAS(pin_cfg, pin_cfg, PIN Config.);
MSH_CMD_EXPORT_ALIAS(pin_val, pin_val, PIN Read.);
MSH_CMD_EXPORT_ALIAS(uart_test, uart_test, UART Test.);

#include <sys/time.h>
void cmd_date(int argc, char **argv)
{
    time_t now = time(RT_NULL);
    rt_kprintf("%s\n", ctime(&now));
}
MSH_CMD_EXPORT_ALIAS(cmd_date, date, show date and time.)

void cmd_reboot(int argc, char **argv)
{
    rt_kprintf("sys reboot\n");
    rt_thread_delay(1000);
    esp_restart();
    rt_thread_delay(60000);
}
MSH_CMD_EXPORT_ALIAS(cmd_reboot, reboot, reboot esp.)

extern void rt_usage_info(rt_uint32_t *major, rt_uint32_t *minor);
void cmd_cpusage(int argc, char **argv)
{
	rt_uint32_t cpu_usage_major,cpu_usage_minor;
	rt_usage_info(&cpu_usage_major, &cpu_usage_minor);
    rt_kprintf("Cpu Usage: %d.%d%%\n",cpu_usage_major,cpu_usage_minor);
}
MSH_CMD_EXPORT_ALIAS(cmd_cpusage, cpusage, cpu usage.)

int cmd_uptime(int argc, char** argv)
{
    unsigned updays, uphours, upminutes;

    updays = (unsigned) uptime_count / (unsigned)(120*60*24);
    if (updays)
        rt_kprintf("%u day%s, ", updays, (updays != 1) ? "s" : "");
    upminutes = (unsigned) uptime_count / (unsigned)120;
    uphours = (upminutes / (unsigned)60) % (unsigned)24;
    upminutes %= 60;
    if (uphours)
        rt_kprintf("%2u:%02u\n", uphours, upminutes);
    else
        rt_kprintf("%u min\n", upminutes);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(cmd_uptime, uptime, system up time.)

extern int lua_main (int argc, char **argv);
MSH_CMD_EXPORT_ALIAS(lua_main, lua, LUA run engine.);
#endif
