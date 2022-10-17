#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CLOCK_PIN (25)
#define DATA_PIN  (26)

// At least 2.
#define CLOCK_PERIOD_MICROS (10)
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
  // We assume clock is already pulled low.

  // Send data, MSB first, sampled on rising clock edge.
  for (int i = 7; i >= 0; i--) {
    gpio_set_level(DATA_PIN, (data & (1 << i)) >> i);
    tick();
    gpio_set_level(CLOCK_PIN, 1);
    tick();
    gpio_set_level(CLOCK_PIN, 0);
    tick();
  }

  // Data low.
  gpio_set_level(DATA_PIN, 0);
}

void send_led_update(rgb_t* led_frames) {
  // Start frame - 32 bits of zeroes.
  clocked_send(0);
  clocked_send(0);
  clocked_send(0);
  clocked_send(0);

  for (int i = 0; i < LED_LENGTH; i++) {
    // 11100000 | brightness from 1-32 levels, using remaining 5 bits
    unsigned char brightness = (7 << 5) | (led_frames[i].brightness / 32);
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
  // Give LED strip a short time to settle.
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  const gpio_config_t gpio_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1<<CLOCK_PIN) | (1<<DATA_PIN),
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .pull_up_en = 0,
  };
  gpio_config(&gpio_conf);

  rgb_t led_strip[LED_LENGTH];
  int pos = 0;
  int incr = 1;
  while(1) {
    uint32_t brightness = esp_random();
    for (int i = 0; i < LED_LENGTH; i++) {
      if (i == pos) {
        led_strip[i].brightness = brightness % 256;
        led_strip[i].r = (brightness * (LED_LENGTH - i)) % 256;
        led_strip[i].g = (brightness * i) % 256;
        led_strip[i].b = brightness % 256;
      } else {
        led_strip[i].brightness = 0;
      }
    }

    send_led_update(led_strip);
    if (pos + incr >= LED_LENGTH) {
      incr = -1;
    } else if (pos + incr < 0) {
      incr = 1;
    }

    pos += incr;
  }
}
