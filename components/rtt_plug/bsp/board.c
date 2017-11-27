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

#define UART_FULL_THRESH_DEFAULT  (120)
#define UART_TOUT_THRESH_DEFAULT   (10)

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
    if(cfg->reserved & UART_HW_FLOWCTRL_RTS)
        UART[uart_num]->conf1.rx_flow_en = 1;
    else
        UART[uart_num]->conf1.rx_flow_en = 0;
    if(cfg->reserved & UART_HW_FLOWCTRL_CTS)
        UART[uart_num]->conf0.tx_flow_en = 1;
    else
        UART[uart_num]->conf0.tx_flow_en = 0;
	//
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
        ESP_INTR_DISABLE(uart->irq);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        ESP_INTR_ENABLE(uart->irq);
        break;
    }

    return RT_EOK;
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
    int uart_num = uart->num;

    int ch = -1;
    uint8_t rx_fifo_cnt = UART[uart_num]->status.rxfifo_cnt;
    if (rx_fifo_cnt > 0)
        ch = UART[uart_num]->fifo.rw_byte;
    return ch;
}

//internal isr handler for default driver code.
static void IRAM_ATTR uart_rx_intr_handler_default(void *param)
{
    struct rt_serial_device *serial = (struct rt_serial_device *)param;
    struct esp32_uart* uart = (struct esp32_uart *)serial->parent.user_data;
    uint8_t uart_num = uart->num;
    uint32_t uart_intr_status = UART[uart_num]->int_st.val;

    while(uart_intr_status != 0x0) 
    {
        if((uart_intr_status & UART_RXFIFO_TOUT_INT_ST_M) 
            || (uart_intr_status & UART_RXFIFO_FULL_INT_ST_M)) {
            rt_interrupt_enter();
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
            rt_interrupt_leave();
            UART[uart_num]->int_clr.rxfifo_tout = 1;
            UART[uart_num]->int_clr.rxfifo_full = 1;
        } else if(uart_intr_status & UART_RXFIFO_OVF_INT_ST_M) {
            UART[uart_num]->conf0.rxfifo_rst = 1;
            UART[uart_num]->conf0.rxfifo_rst = 0;
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

#ifdef RT_USING_UART1
static struct rt_serial_device serial1;
static struct esp32_uart uart1 = { UART_NUM_1, UART_INTR_NUM_1 };
#endif

#ifdef RT_USING_UART2
static struct rt_serial_device serial2;
static struct esp32_uart uart2 = { UART_NUM_2, UART_INTR_NUM_2 };
#endif

void rt_hw_usart_init() 
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	config.bufsz = 4096;
	
#ifdef RT_USING_UART0
	// wait for fifo empty
	while (UART[uart0.num]->status.txfifo_cnt);
	uart_tx_wait_idle(0);
	//
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
	gpio_set_pull_mode(GPIO_NUM_23, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_23, 1);
	gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_23, SIG_GPIO_OUT_IDX, 0, 0);	
	config.reserved = UART_HW_FLOWCTRL_DISABLE;

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
    UART[uart1.num]->conf0.rxfifo_rst = 1;
    UART[uart1.num]->conf0.rxfifo_rst = 0;
    UART[uart1.num]->conf0.txfifo_rst = 1;
    UART[uart1.num]->conf0.txfifo_rst = 0;
	
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
	gpio_set_level(GPIO_NUM_5, 1);
	gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_5, SIG_GPIO_OUT_IDX, 0, 0);
	config.reserved = UART_HW_FLOWCTRL_DISABLE;

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
    UART[uart2.num]->conf0.rxfifo_rst = 1;
    UART[uart2.num]->conf0.rxfifo_rst = 0;
    UART[uart2.num]->conf0.txfifo_rst = 1;
    UART[uart2.num]->conf0.txfifo_rst = 0;

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
	config.reserved = UART_HW_FLOWCTRL_DISABLE;

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

void rt_hw_idle_hook(void)
{
	extern void esp_vApplicationIdleHook( void );

#if ( configUSE_IDLE_HOOK == 1 )
	extern void vApplicationIdleHook( void );
	vApplicationIdleHook();
#endif /* configUSE_IDLE_HOOK */

	esp_vApplicationIdleHook();
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

volatile int eth_linkstatus = 0;
volatile int sys_stauts = -1;
volatile int ppp_linkstatus = 0;
volatile int uptime_count = 0;

unsigned char PZ[4] = {0};
char RTT_USER[16] = {"admin"};
char RTT_PASS[36] = {"21232f297a57a5a743894a0e4a801fc3"};
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
	/* initialize gpio */
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_15)) rtc_gpio_deinit(GPIO_NUM_15);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_15], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);
	gpio_set_level(GPIO_NUM_15, 1);
	gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
	gpio_matrix_out(GPIO_NUM_15, SIG_GPIO_OUT_IDX, 0, 0);
	if(RTC_GPIO_IS_VALID_GPIO(GPIO_NUM_35)) rtc_gpio_deinit(GPIO_NUM_35);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GPIO_NUM_35], PIN_FUNC_GPIO);
	gpio_set_pull_mode(GPIO_NUM_35, GPIO_PULLUP_ONLY);
	gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);
	gpio_matrix_out(GPIO_NUM_35, SIG_GPIO_OUT_IDX, 0, 0);
    /* initialize uart */
    rt_hw_usart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif
    ets_install_putc1(rt_hw_write_char);
    ets_install_putc2(NULL);

	rt_thread_idle_sethook(rt_hw_idle_hook);
	rt_thread_inited_sethook(thread_inited);

	/* set cst */	
    putenv("TZ=CST-8:00");
    tzset();
}

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

MSH_CMD_EXPORT_ALIAS(pin_cfg, pin_cfg, PIN Config.);
MSH_CMD_EXPORT_ALIAS(pin_val, pin_val, PIN Read.);

#include <sys/time.h>
void cmd_date(int argc, char **argv)
{
    time_t now = time(RT_NULL);
    rt_kprintf("%s\n", ctime(&now));
}
MSH_CMD_EXPORT_ALIAS(cmd_date, date, show date and time.)

extern int lua_main (int argc, char **argv);
MSH_CMD_EXPORT_ALIAS(lua_main, lua, LUA run engine.);
#endif
