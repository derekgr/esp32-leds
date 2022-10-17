#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CLOCK_PIN (25)
#define DATA_PIN  (26)
#define CLOCK_PERIOD_MICROS (100)
#define LED_LENGTH (30)

typedef struct {
  // Sampled down to 32 levels.
  unsigned char brightness;

  unsigned char r, g, b;
} rgb_t;

void tick() {
  const uint32_t delay = CLOCK_PERIOD_MICROS / 2;
  ets_delay_us(delay);
}

void clocked_send(unsigned char data) {
  // Clock and data low.
  gpio_set_level(DATA_PIN, 0);
  gpio_set_level(CLOCK_PIN, 0);
  tick();

  // Send data, MSB first, sampled on rising clock edge.
  for (int i = 7; i >= 0; i--) {
    gpio_set_level(DATA_PIN, (data & (1ULL << i)) >> i);
    tick();
    gpio_set_level(CLOCK_PIN, 1);
    tick();
    gpio_set_level(CLOCK_PIN, 0);
    tick();
  }

  // Data low.
  gpio_set_level(DATA_PIN, 0);
  tick();
}

void send_led_update(rgb_t* led_frames) {
  // Start frame - 32 bits of zeroes.
  clocked_send(0);
  clocked_send(0);
  clocked_send(0);
  clocked_send(0);

  for (int i = 0; i < LED_LENGTH; i++) {
    // 11100000 | brightness from 1-32 levels, using remaining 5 bits
    unsigned char brightness = (6ULL << 5) | (led_frames[i].brightness / 32);
    clocked_send(brightness);
    clocked_send(led_frames[i].b);
    clocked_send(led_frames[i].g);
    clocked_send(led_frames[i].r);
  }

  // End frame - 32 bits of all ones.
  clocked_send(255);
  clocked_send(255);
  clocked_send(255);
  clocked_send(255);
}

void app_main() {
  // Give LED strip time a short time to settle.
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  const gpio_config_t gpio_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ULL<<CLOCK_PIN) | (1ULL<<DATA_PIN),
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .pull_up_en = 0,
  };
  gpio_config(&gpio_conf);

  rgb_t led_strip[LED_LENGTH];
  while(1) {
    for (int i = 0; i < LED_LENGTH; i++) {
      led_strip[i].brightness = esp_random() % 255;

      switch (esp_random()%3) {
      case 0:
        led_strip[i].r = 255;
        led_strip[i].g = 0;
        led_strip[i].b = 0;
        break;

      case 1:
        led_strip[i].r = 0;
        led_strip[i].g = 255;
        led_strip[i].b = 0;
        break;

      default:
        led_strip[i].r = 0;
        led_strip[i].b = 255;
        led_strip[i].g = 0;
      }
    }

    send_led_update(led_strip);
  }
}
