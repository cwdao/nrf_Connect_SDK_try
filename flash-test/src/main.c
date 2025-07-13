/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>


#ifdef CONFIG_LOG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER (m, LOG_LEVEL_DBG);
#define log_d(...) {LOG_DBG(__VA_ARGS__);}
#define log_i(...) {LOG_INF(__VA_ARGS__);}
#define log_w(...) {LOG_WRN(__VA_ARGS__);}
#define log_e(...) {LOG_ERR(__VA_ARGS__);}
#define log_hex(a,b,c) {LOG_HEXDUMP_INF(b,c,a);}
#else
#define log_d(fmt, ...)
#define log_i(fmt, ...)
#define log_w(fmt, ...)
#define log_e(fmt, ...)
#define log_hex(fmt, ...)
#define log_char(fmt, ...)
#endif


const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25r64));
#define SPI_FLASH_TEST_REGION_OFFSET 0xff000
#define SPI_FLASH_SECTOR_SIZE 4096
#define CONFIG_BT_RAS_MAX_ANTENNA_PATHS 2


typedef enum
{
    CS_DE_TONE_QUALITY_OK,
    CS_DE_TONE_QUALITY_BAD,
} cs_de_tone_quality_t;
typedef enum
{
    CS_DE_QUALITY_OK,
    CS_DE_QUALITY_DO_NOT_USE,
} cs_de_quality_t;
/** Channel sounding role */
enum bt_conn_le_cs_role {
	/** CS initiator role */
	BT_CONN_LE_CS_ROLE_INITIATOR,
	/** CS reflector role */
	BT_CONN_LE_CS_ROLE_REFLECTOR,
};
typedef struct
{
    /** In-phase measurements of tones on this device */
    float i_local[80];
    /** Quadrature-phase measurement of tones on this device */
    float q_local[80];
    /** In-phase measurements of tones from remote device */
    float i_remote[80];
    /** Quadrature-phase measurements of tones from remote device */
    float q_remote[80];
} cs_de_iq_tones_t;
typedef struct
{                      /** Distance estimate based on inverse fourier transform. */
    float ifft;        /** Distance estimate based on average phase slope. */
    float phase_slope; /** Distance estimate based on RTT. */
    float rtt;         /** Best effort distance estimate. * * This is a convenience value which is
                          automatically set to the most * accurate of the estimation methods. */
    float best;
} cs_de_dist_estimates_t;

typedef struct
{                                 
    int64_t index;
    /** CS Role. */
    enum bt_conn_le_cs_role role; 
    /** Number of antenna paths present in data. */
    uint8_t n_ap;                
    /** IQ values for local and remote measured tones. */
    cs_de_iq_tones_t iq_tones[CONFIG_BT_RAS_MAX_ANTENNA_PATHS]; 
    /** Tone quality indicators */
    cs_de_tone_quality_t tone_quality[CONFIG_BT_RAS_MAX_ANTENNA_PATHS]; 
    /** Distance estimate results */
    cs_de_dist_estimates_t distance_estimates[CONFIG_BT_RAS_MAX_ANTENNA_PATHS]; 
    /** Total time measured during RTT  measurements */
    int32_t rtt_accumulated_half_ns;                         
    /** Number of RTT measurements taken */
    uint8_t rtt_count;
} cs_de_report_t;

//========================================================================
static void timer_expiry_function(struct k_timer *timer_id);
static K_TIMER_DEFINE(my_timer, timer_expiry_function, NULL);

//========================================================================
#define LED0_PORT        DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(led0), gpios))
#define LED0_PIN         DT_GPIO_PIN(DT_NODELABEL(led0), gpios)
#define LED0_FLAGS       DT_GPIO_FLAGS(DT_NODELABEL(led0), gpios)
#define LED0_INIT()      {gpio_pin_configure(LED0_PORT,LED0_PIN,GPIO_OUTPUT_LOW|LED0_FLAGS);}
#define LED0_ON()        {gpio_pin_set_raw(LED0_PORT,LED0_PIN,1);}
#define LED0_OFF()       {gpio_pin_set_raw(LED0_PORT,LED0_PIN,0);}

#define LED1_PORT        DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(led1), gpios))
#define LED1_PIN         DT_GPIO_PIN(DT_NODELABEL(led1), gpios)
#define LED1_FLAGS       DT_GPIO_FLAGS(DT_NODELABEL(led1), gpios)
#define LED1_INIT()      {gpio_pin_configure(LED1_PORT,LED1_PIN,GPIO_OUTPUT_LOW|LED1_FLAGS);}
#define LED1_ON()        {gpio_pin_set_raw(LED1_PORT,LED1_PIN,1);}
#define LED1_OFF()       {gpio_pin_set_raw(LED1_PORT,LED1_PIN,0);}

#define _BUTTON_0_PRT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(button0), gpios))
#define _BUTTON_0_PIN DT_GPIO_PIN(DT_NODELABEL(button0), gpios)
#define _BUTTON_0_FLG DT_GPIO_FLAGS(DT_NODELABEL(button0), gpios)
static struct gpio_callback _button0_cb;

#define _BUTTON_1_PRT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(button1), gpios))
#define _BUTTON_1_PIN DT_GPIO_PIN(DT_NODELABEL(button1), gpios)
#define _BUTTON_1_FLG DT_GPIO_FLAGS(DT_NODELABEL(button1), gpios)
static struct gpio_callback _button1_cb;

#define _BUTTON_2_PRT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(button2), gpios))
#define _BUTTON_2_PIN DT_GPIO_PIN(DT_NODELABEL(button2), gpios)
#define _BUTTON_2_FLG DT_GPIO_FLAGS(DT_NODELABEL(button2), gpios)
static struct gpio_callback _button2_cb;

#define _BUTTON_3_PRT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(button3), gpios))
#define _BUTTON_3_PIN DT_GPIO_PIN(DT_NODELABEL(button3), gpios)
#define _BUTTON_3_FLG DT_GPIO_FLAGS(DT_NODELABEL(button3), gpios)
static struct gpio_callback _button3_cb;
//========================================================================
#define WRITE_FRQ_MS    20

const uint8_t erased[] = {0xff, 0xff, 0xff, 0xff};
static __IO uint8_t run_state = 0xff;
static __IO uint64_t flash_index = 0;
static uint64_t flash_size = 0;
static cs_de_report_t the_rep = {0};


static void _debug_report(cs_de_report_t *p_rep)
{
    LOG_INF("=======log %lld=======",p_rep->index);
    LOG_INF("nap:%d",p_rep->n_ap);
    
}
static void timer1_handler(struct k_work *work);
K_WORK_DEFINE(timer1_def, timer1_handler);
static void timer_expiry_function(struct k_timer *timer_id) 
{
    k_work_submit(&timer1_def);
}
static void timer1_handler(struct k_work *work)
{
    int err = -1;
    
    if(0 == run_state)
    {
        the_rep.index = flash_index;
        the_rep.n_ap = flash_index+1;
        LED0_ON();
        err = flash_write(flash_dev, flash_index*SPI_FLASH_SECTOR_SIZE, &the_rep, sizeof(cs_de_report_t));
        LED0_OFF();
        if(0 != err)
        {
            LOG_WRN("===> flash wirte err! %d",err);
            k_timer_stop(&my_timer);
            run_state = 0xff;
            return;
        }
        flash_index++;
        return;
    }
    if(2 == run_state)
    {
        LED0_ON();
        err = flash_read(flash_dev, flash_index*SPI_FLASH_SECTOR_SIZE, &the_rep, sizeof(cs_de_report_t));
        LED0_OFF();
        if(0 != err)
        {
            LOG_WRN("===> flash read err! %d",err);
            run_state = 0xff;
            return;
        }
        if(-1 == the_rep.index)
        {
            LOG_WRN("Log read got -1, stop");
            run_state = 0xff;
            return;
        }
        _debug_report(&the_rep);
        flash_index++;
        k_timer_start(&my_timer, K_MSEC(100), K_NO_WAIT);
        return;
    }
}


static void work_irq0_handler(struct k_work *work)
{
	if(!gpio_pin_get(_BUTTON_0_PRT,_BUTTON_0_PIN)) 
	{
		LOG_INF("irq0 release");
		return;
	}
    if(0xff == run_state)
    {
        run_state = 0;
        flash_index = 0;
        LOG_INF("start write timer");
        k_timer_start(&my_timer, K_MSEC(1000), K_MSEC(WRITE_FRQ_MS));
        return;
    }
    LOG_INF("button0 press but start err %d",run_state);
}
K_WORK_DEFINE(work_irq0_def, work_irq0_handler);
static void _irq0_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_work_submit(&work_irq0_def);
}
static void _init_button0()
{
	int err = gpio_pin_configure(_BUTTON_0_PRT, _BUTTON_0_PIN, GPIO_INPUT| _BUTTON_0_FLG);
	if(err)
	{
		LOG_WRN("button pin config error");
	}
	gpio_init_callback(&_button0_cb, _irq0_cb, BIT( _BUTTON_0_PIN));
	err = gpio_add_callback(_BUTTON_0_PRT, &_button0_cb);
	if(err)
	{
		LOG_WRN("button add cb error");
	}
    gpio_pin_interrupt_configure(_BUTTON_0_PRT, _BUTTON_0_PIN, GPIO_INT_EDGE_FALLING);
}
static void work_irq1_handler(struct k_work *work)
{
	if(!gpio_pin_get(_BUTTON_1_PRT,_BUTTON_1_PIN)) 
	{
		LOG_INF("irq1 release");
		return;
	}
    if(0 == run_state
      ||  2 == run_state)
    {
        LOG_INF("stop write/log timer");
        k_timer_stop(&my_timer);
        run_state = 0xff;
        return;
    }
    LOG_INF("button1 press but start err %d",run_state);
}
K_WORK_DEFINE(work_irq1_def, work_irq1_handler);
static void _irq1_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_work_submit(&work_irq1_def);
}
static void _init_button1()
{
	int err = gpio_pin_configure(_BUTTON_1_PRT, _BUTTON_1_PIN, GPIO_INPUT| _BUTTON_1_FLG);
	if(err)
	{
		LOG_WRN("button pin config error");
	}
	gpio_init_callback(&_button1_cb, _irq1_cb, BIT( _BUTTON_1_PIN));
	err = gpio_add_callback(_BUTTON_1_PRT, &_button1_cb);
	if(err)
	{
		LOG_WRN("button add cb error");
	}
    gpio_pin_interrupt_configure(_BUTTON_1_PRT, _BUTTON_1_PIN, GPIO_INT_EDGE_FALLING);
}
static void work_irq2_handler(struct k_work *work)
{
	if(!gpio_pin_get(_BUTTON_2_PRT,_BUTTON_2_PIN)) 
	{
		LOG_INF("irq2 release");
		return;
	}
    if(0xff == run_state)
    {
        run_state = 2;
        flash_index = 0;
        LOG_INF("start log timer");
        k_timer_start(&my_timer, K_MSEC(1000), K_NO_WAIT);
        return;
    }
    LOG_INF("button2 press but start err %d",run_state);
}
K_WORK_DEFINE(work_irq2_def, work_irq2_handler);
static void _irq2_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_work_submit(&work_irq2_def);
}
static void _init_button2()
{
	int err = gpio_pin_configure(_BUTTON_2_PRT, _BUTTON_2_PIN, GPIO_INPUT| _BUTTON_2_FLG);
	if(err)
	{
		LOG_WRN("button pin config error");
	}
	gpio_init_callback(&_button2_cb, _irq2_cb, BIT( _BUTTON_2_PIN));
	err = gpio_add_callback(_BUTTON_2_PRT, &_button2_cb);
	if(err)
	{
		LOG_WRN("button add cb error");
	}
    gpio_pin_interrupt_configure(_BUTTON_2_PRT, _BUTTON_2_PIN, GPIO_INT_EDGE_FALLING);
}
static void work_irq3_handler(struct k_work *work)
{
    int err = 0;
	if(!gpio_pin_get(_BUTTON_3_PRT,_BUTTON_3_PIN)) 
	{
		LOG_INF("irq3 release");
		return;
	}
    if(0xff == run_state)
    {
#if 1
        for(uint32_t i=0; i<flash_size/SPI_FLASH_SECTOR_SIZE; i++)
        {
            LED0_ON();
            err = flash_read(flash_dev, i*SPI_FLASH_SECTOR_SIZE, &the_rep, sizeof(cs_de_report_t));
            if(-1 == the_rep.index)
            {
                break;
            }
            LOG_INF("start earse flash %d",i);
            err = flash_erase(flash_dev,i*SPI_FLASH_SECTOR_SIZE,SPI_FLASH_SECTOR_SIZE);
            LED0_OFF();
        }
#else
        LOG_INF("start full earse flash");
        LED0_ON();
        err = flash_erase(flash_dev,0,flash_size);
        LED0_OFF();
#endif
        LOG_INF("earse flash done %d",err);
        return;
    }
    LOG_INF("button3 press but start err %d",run_state);
}
K_WORK_DEFINE(work_irq3_def, work_irq3_handler);
static void _irq3_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_work_submit(&work_irq3_def);
}
static void _init_button3()
{
	int err = gpio_pin_configure(_BUTTON_3_PRT, _BUTTON_3_PIN, GPIO_INPUT| _BUTTON_3_FLG);
	if(err)
	{
		LOG_WRN("button pin config error");
	}
	gpio_init_callback(&_button3_cb, _irq3_cb, BIT( _BUTTON_3_PIN));
	err = gpio_add_callback(_BUTTON_3_PRT, &_button3_cb);
	if(err)
	{
		LOG_WRN("button add cb error");
	}
    gpio_pin_interrupt_configure(_BUTTON_3_PRT, _BUTTON_3_PIN, GPIO_INT_EDGE_FALLING);
}
static void _init_buttonled()
{
    //led0 
    LED0_INIT();
    LED1_INIT();    
    //button0 button1 button2 button3
    _init_button0();
    _init_button1();
    _init_button2();
    _init_button3();
}
int main(void)
{
    LOG_INF("Start Main");
    
    if (!device_is_ready(flash_dev))
    {
        LOG_INF("%s: device not ready.", flash_dev->name);
        return 0;
    }
    flash_get_size(flash_dev,&flash_size);
    //8M 2616 bytes
    LOG_INF("==> SPI flash[%s] Ready: %lldMB rec:%d", flash_dev->name,flash_size/(1024*1024),sizeof(cs_de_report_t));
    _init_buttonled();
//    single_sector_test(flash_dev);

    return 0;
}
