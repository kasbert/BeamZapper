
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cwiid.h"
#include "esp_bt.h"
#include "esp_hidh_api.h"
#include "esp_hid_gap.h"
#include "esp_hidh.h"
#include "nvs_flash.h"



static const char* TAG = "beamzapper";

#define LED_PIN 2

#define STACK_SIZE 4096

#define ESP_INTR_FLAG_DEFAULT 0

void init_gpio();
void sync_debug();
void set_xy(uint16_t x, uint16_t y);
void bt_init();
bool is_video();

extern struct wiimote wiimote;

static void read_serial() {
    static unsigned char j0 = 1, j1 = 1, j2 = 1, j3 = 1;

    int c = getchar();
    while (c > 0) {
        ESP_LOGI(TAG, "Serial read %d", c);
        if (c == '0') {
            j0 = !j0;
            gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY0, j0);
        }
        if (c == '1') {
            j1 = !j1;
            gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY1, j1);
        }
        if (c == '2') {
            j2 = !j2;
            gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY2, j2);
        }
        if (c == '3') {
            j3 = !j3;
            gpio_set_level(CONFIG_BEAMZAPPER_PIN_JOY3, j3);
        }

        if (c == 105)  // i
        {
            cwiid_set_rpt_mode(&wiimote, CWIID_RPT_BTN | CWIID_RPT_IR);
        }
        if (c == 97)  // a
        {
            cwiid_set_rpt_mode(&wiimote, CWIID_RPT_BTN | CWIID_RPT_ACC);
        }
        if (c == 98) {  // b
            cwiid_set_rpt_mode(&wiimote, CWIID_RPT_BTN);
        }

        if (c == 100 && wiimote.dev)  // d
            esp_bt_hid_host_disconnect(esp_hidh_dev_bda_get(wiimote.dev));

        if (c == 108)  // l
            cwiid_set_led(&wiimote, 1);

        c = getchar();
    }
}



static void IRAM_ATTR gpio_loop(void* p) {
    init_gpio();
    /*
    */
   gpio_dump_io_configuration(stdout,
      (1 << CONFIG_BEAMZAPPER_PIN_VSYNC) |
          (1 << CONFIG_BEAMZAPPER_PIN_HSYNC) |
          (1 << CONFIG_BEAMZAPPER_PIN_BUTTON) |
          (1 << CONFIG_BEAMZAPPER_PIN_JOY0) |
          (1 << CONFIG_BEAMZAPPER_PIN_JOY1) |
          (1 << CONFIG_BEAMZAPPER_PIN_JOY2) |
          (1 << CONFIG_BEAMZAPPER_PIN_JOY3));

    ESP_LOGI(TAG, "Start Core %d debug_loop", xPortGetCoreID());
    esp_intr_dump(stdout);
    printf("Task Name\tStatus\tPrio\tHWM\tTask\tAffinity\n");
    char stats_buffer[1024];
    vTaskList(stats_buffer);
    printf("%s\n", stats_buffer);

    // status blinks
    static int c = 0;
    const static char seqs[4][6] = {
        { // no_sync_bt_scan
            1,1,1,0,1,0,
        },
        { // bt_scan
            1,1,1,0,0,0,
        },
        { // no_sync
            0,1,0,1,0,1,
        },
        { // ok
            1,1,1,1,1,1,
        }
    };

    // Just for debugging
    while (1) {
        for (int i = 0; i < 50; i++) {
            vTaskDelay(CONFIG_FREERTOS_HZ / 5);  // 0.2s
            read_serial();
            const char* seq = seqs[(is_video() ? 1 : 0) + (wiimote.dev ? 2 : 0)];
            gpio_set_level(CONFIG_BEAMZAPPER_PIN_LED, seq[c]);
            c++;
            if (c >= 6) c = 0;
        }
        sync_debug();
        //vTaskDelay(CONFIG_FREERTOS_HZ * 60);
    }
}

void app_main(void) {
    TaskHandle_t xHandle1;
    esp_err_t ret;

    gpio_set_direction(CONFIG_BEAMZAPPER_PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_BEAMZAPPER_PIN_LED, 1);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    bt_init();

    xTaskCreatePinnedToCore(
        gpio_loop,    // Function that implements the task.
        "gpio_loop",  // Text name for the task.
        STACK_SIZE,   // Stack size in bytes, not words.
        (void*)1,    // Parameter passed into the task.
        tskIDLE_PRIORITY + 1,
        &xHandle1,  // Variable to hold the task's data structure.
        1); // interrupts in core 1

}
