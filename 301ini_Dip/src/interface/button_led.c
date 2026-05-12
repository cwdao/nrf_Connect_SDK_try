#include "button_led.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(button_led, LOG_LEVEL_DBG);

// 按键和 LED 定义
#define LED_COUNT 2
#define BUTTON_COUNT 4

// LED 配置
#define LED0_PORT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(led2), gpios))
#define LED0_PIN DT_GPIO_PIN(DT_NODELABEL(led2), gpios)
#define LED0_FLAGS DT_GPIO_FLAGS(DT_NODELABEL(led2), gpios)

#define LED1_PORT DEVICE_DT_GET(DT_GPIO_CTLR(DT_NODELABEL(led3), gpios))
#define LED1_PIN DT_GPIO_PIN(DT_NODELABEL(led3), gpios)
#define LED1_FLAGS DT_GPIO_FLAGS(DT_NODELABEL(led3), gpios)



// LED 初始化
static const struct device *led_ports[LED_COUNT] = {LED0_PORT, LED1_PORT};
static const uint8_t led_pins[LED_COUNT] = {LED0_PIN, LED1_PIN};
static const uint32_t led_flags[LED_COUNT] = {LED0_FLAGS, LED1_FLAGS};

// 按键初始化
static const struct device *button_ports[BUTTON_COUNT] = {
    BUTTON0_PORT, BUTTON1_PORT, BUTTON2_PORT, BUTTON3_PORT};
static const uint8_t button_pins[BUTTON_COUNT] = {BUTTON0_PIN, BUTTON1_PIN,
                                                  BUTTON2_PIN, BUTTON3_PIN};
static const uint32_t button_flags[BUTTON_COUNT] = {
    BUTTON0_FLAGS, BUTTON1_FLAGS, BUTTON2_FLAGS, BUTTON3_FLAGS};

// 按键回调
static struct gpio_callback button_callbacks[BUTTON_COUNT];
static button_callback_t button_handlers[BUTTON_COUNT] = {0};

// 按键中断回调函数
static void button_irq_handler(const struct device *port,
                               struct gpio_callback *cb,
                               gpio_port_pins_t pins) {
  for (int i = 0; i < BUTTON_COUNT; i++) {
    if (cb == &button_callbacks[i] && button_handlers[i]) {
      button_handlers[i](); // 调用注册的回调函数
    }
  }
}

// 注册按键回调
void button_register_callback(uint8_t button_id, button_callback_t callback) {
  if (button_id < BUTTON_COUNT) {
    button_handlers[button_id] = callback;
  }
}

// 控制 LED
void led_on(uint8_t led_id) {
  if (led_id < LED_COUNT) {
    gpio_pin_set_raw(led_ports[led_id], led_pins[led_id], 1);
  }
}

void led_off(uint8_t led_id) {
  if (led_id < LED_COUNT) {
    gpio_pin_set_raw(led_ports[led_id], led_pins[led_id], 0);
  }
}

void led_toggle(uint8_t led_id) {
  if (led_id < LED_COUNT) {
    int state = gpio_pin_get_raw(led_ports[led_id], led_pins[led_id]);
    gpio_pin_set_raw(led_ports[led_id], led_pins[led_id], !state);
  }
}

// 模块初始化
void button_led_init(void) {
  int err;

  // 初始化 LED
  for (int i = 0; i < LED_COUNT; i++) {
    err = gpio_pin_configure(led_ports[i], led_pins[i],
                             GPIO_OUTPUT_LOW | led_flags[i]);
    if (err) {
      LOG_ERR("Failed to configure LED %d", i);
    }
  }

  // 初始化按键
  for (int i = 0; i < BUTTON_COUNT; i++) {
    err = gpio_pin_configure(button_ports[i], button_pins[i],
                             GPIO_INPUT | button_flags[i]);
    if (err) {
      LOG_ERR("Failed to configure button %d", i);
      continue;
    }

    // 注册中断回调
    gpio_init_callback(&button_callbacks[i], button_irq_handler,
                       BIT(button_pins[i]));
    err = gpio_add_callback(button_ports[i], &button_callbacks[i]);
    if (err) {
      LOG_ERR("Failed to add callback for button %d", i);
      continue;
    }

    // 启用中断
    err = gpio_pin_interrupt_configure(button_ports[i], button_pins[i],
                                       GPIO_INT_EDGE_FALLING);
    if (err) {
      LOG_ERR("Failed to configure interrupt for button %d", i);
    }
  }
}