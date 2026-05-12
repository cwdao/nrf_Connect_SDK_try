#ifndef BUTTON_LED_H
#define BUTTON_LED_H

#include <stdint.h>

// 按键配置
#define BUTTON0_PORT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(button0), gpios))
#define BUTTON0_PIN DT_GPIO_PIN(DT_NODELABEL(button0), gpios)
#define BUTTON0_FLAGS DT_GPIO_FLAGS(DT_NODELABEL(button0), gpios)

#define BUTTON1_PORT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(button1), gpios))
#define BUTTON1_PIN DT_GPIO_PIN(DT_NODELABEL(button1), gpios)
#define BUTTON1_FLAGS DT_GPIO_FLAGS(DT_NODELABEL(button1), gpios)

#define BUTTON2_PORT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(button2), gpios))
#define BUTTON2_PIN DT_GPIO_PIN(DT_NODELABEL(button2), gpios)
#define BUTTON2_FLAGS DT_GPIO_FLAGS(DT_NODELABEL(button2), gpios)

#define BUTTON3_PORT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(button3), gpios))
#define BUTTON3_PIN DT_GPIO_PIN(DT_NODELABEL(button3), gpios)
#define BUTTON3_FLAGS DT_GPIO_FLAGS(DT_NODELABEL(button3), gpios)
// 按键和 LED 模块初始化
void button_led_init(void);

// 按键事件处理函数类型
typedef void (*button_callback_t)(void);

// 按键注册回调函数
void button_register_callback(uint8_t button_id, button_callback_t callback);

// LED 操作接口
void led_on(uint8_t led_id);
void led_off(uint8_t led_id);
void led_toggle(uint8_t led_id);

#endif // BUTTON_LED_H