#ifndef BUTTON_LED_H
#define BUTTON_LED_H

#include <stdint.h>

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