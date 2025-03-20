#include "esp_bt.h"
#include "esp_event.h"
#include "esp_hidh_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#define ESP_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_BD_ADDR_HEX(addr) \
  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#else
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#endif

#include "cwiid.h"
#include "esp_hid_gap.h"
#include "esp_hidh.h"

#include <driver/gpio.h>


static const char* TAG = "beamzapper";

struct wiimote wiimote;

static bool ir_state, acc_state;
static int64_t last_activity_time;
static uint16_t last_buttons;

#define STACK_SIZE 4096


void set_joy_state(bool j0, bool j1, bool j2, bool j3, bool fire);
void set_xy(uint16_t x, uint16_t y);


void print_state(struct cwiid_state* state) {
    int i;
    int valid_source = 0;

    printf("Report Mode:");
    if (state->rpt_mode & CWIID_RPT_STATUS) printf(" STATUS");
    if (state->rpt_mode & CWIID_RPT_BTN) printf(" BTN");
    if (state->rpt_mode & CWIID_RPT_ACC) printf(" ACC");
    if (state->rpt_mode & CWIID_RPT_IR) printf(" IR");
    if (state->rpt_mode & CWIID_RPT_NUNCHUK) printf(" NUNCHUK");
    if (state->rpt_mode & CWIID_RPT_CLASSIC) printf(" CLASSIC");
    if (state->rpt_mode & CWIID_RPT_BALANCE) printf(" BALANCE");
    if (state->rpt_mode & CWIID_RPT_MOTIONPLUS) printf(" MOTIONPLUS");
    printf("\n");

    printf("Active LEDs:");
    if (state->led & CWIID_LED1_ON) printf(" 1");
    if (state->led & CWIID_LED2_ON) printf(" 2");
    if (state->led & CWIID_LED3_ON) printf(" 3");
    if (state->led & CWIID_LED4_ON) printf(" 4");
    printf("\n");

    printf("Rumble: %s\n", state->rumble ? "On" : "Off");

    printf("Battery: %d%%\n", (int)(100.0 * state->battery / CWIID_BATTERY_MAX));

    printf("Buttons: %X\n", state->buttons);

    printf("Acc: x=%d y=%d z=%d\n", state->acc[CWIID_X], state->acc[CWIID_Y],
        state->acc[CWIID_Z]);

    printf("IR: ");
    for (i = 0; i < CWIID_IR_SRC_COUNT; i++) {
        if (state->ir_src[i].valid) {
            valid_source = 1;
            printf("(%d,%d) ", state->ir_src[i].pos[CWIID_X],
                state->ir_src[i].pos[CWIID_Y]);
        }
    }
    if (!valid_source) {
        printf("no sources detected");
    }
    printf("\n");

    switch (state->ext_type) {
    case CWIID_EXT_NONE:
        printf("No extension\n");
        break;
    case CWIID_EXT_UNKNOWN:
        printf("Unknown extension attached\n");
        break;
    case CWIID_EXT_NUNCHUK:
        printf(
            "Nunchuk: btns=%.2X stick=(%d,%d) acc.x=%d acc.y=%d "
            "acc.z=%d\n",
            state->ext.nunchuk.buttons, state->ext.nunchuk.stick[CWIID_X],
            state->ext.nunchuk.stick[CWIID_Y], state->ext.nunchuk.acc[CWIID_X],
            state->ext.nunchuk.acc[CWIID_Y], state->ext.nunchuk.acc[CWIID_Z]);
        break;
    case CWIID_EXT_CLASSIC:
        printf(
            "Classic: btns=%.4X l_stick=(%d,%d) r_stick=(%d,%d) "
            "l=%d r=%d\n",
            state->ext.classic.buttons, state->ext.classic.l_stick[CWIID_X],
            state->ext.classic.l_stick[CWIID_Y],
            state->ext.classic.r_stick[CWIID_X],
            state->ext.classic.r_stick[CWIID_Y], state->ext.classic.l,
            state->ext.classic.r);
        break;
    case CWIID_EXT_BALANCE:
        printf(
            "Balance: right_top=%d right_bottom=%d "
            "left_top=%d left_bottom=%d\n",
            state->ext.balance.right_top, state->ext.balance.right_bottom,
            state->ext.balance.left_top, state->ext.balance.left_bottom);
        break;
    case CWIID_EXT_MOTIONPLUS:
        printf("MotionPlus: angle_rate=(%d,%d,%d) low_speed=(%d,%d,%d)\n",
            state->ext.motionplus.angle_rate[0],
            state->ext.motionplus.angle_rate[1],
            state->ext.motionplus.angle_rate[2],
            state->ext.motionplus.low_speed[0],
            state->ext.motionplus.low_speed[1],
            state->ext.motionplus.low_speed[2]);
        break;
    }
}


static void cwiid_handle_message(struct mesg_array* ma) {
    int mesg_count = ma->count;
    union cwiid_mesg* mesg = ma->array;

    int i, j;
    int valid_source;
    for (i = 0; i < mesg_count; i++) {
        switch (mesg[i].type) {
        case CWIID_MESG_STATUS:
            printf("Status Report: battery=%d extension=",
                mesg[i].status_mesg.battery);
            switch (mesg[i].status_mesg.ext_type) {
            case CWIID_EXT_NONE:
                printf("none");
                break;
            case CWIID_EXT_NUNCHUK:
                printf("Nunchuk");
                break;
            case CWIID_EXT_CLASSIC:
                printf("Classic Controller");
                break;
            case CWIID_EXT_BALANCE:
                printf("Balance Board");
                break;
            case CWIID_EXT_MOTIONPLUS:
                printf("MotionPlus");
                break;
            default:
                printf("Unknown Extension");
                break;
            }
            printf("\n");
            break;

        case CWIID_MESG_BTN:
            last_activity_time = esp_timer_get_time();

            // ESP_LOGD(TAG, "Button Report: %.4X\n", mesg[i].btn_mesg.buttons);

            uint16_t buttons = mesg[i].btn_mesg.buttons;
            /*
            uint16_t changed_buttons = last_buttons ^ buttons;
            uint16_t pressed_buttons = changed_buttons & buttons;
            uint16_t released_buttons = changed_buttons & ~buttons;
            */
            bool wiimote_button_down = !!(buttons & CWIID_BTN_DOWN);
            bool wiimote_button_up = !!(buttons & CWIID_BTN_UP);
            bool wiimote_button_right = !!(buttons & CWIID_BTN_RIGHT);
            bool wiimote_button_left = !!(buttons & CWIID_BTN_LEFT);
            /*
            bool wiimote_button_home = !!(buttons & CWIID_BTN_HOME);
            bool wiimote_button_plus = !!(buttons & CWIID_BTN_PLUS);
            bool wiimote_button_minus = !!(buttons & CWIID_BTN_MINUS);
            bool wiimote_button_2 = !!(buttons & CWIID_BTN_2);
            bool wiimote_button_1 = !!(buttons & CWIID_BTN_1);
            */
            bool wiimote_button_B = !!(buttons & CWIID_BTN_B);
            bool wiimote_button_A = !!(buttons & CWIID_BTN_A);

            set_joy_state(wiimote_button_up | wiimote_button_A, wiimote_button_down, 
                wiimote_button_left, wiimote_button_right, wiimote_button_B);

            /*
            if (pressed_buttons & CWIID_BTN_1) {
                acc_state = !acc_state;
                cwiid_set_rpt_mode(&wiimote, acc_state
                    ? CWIID_RPT_BTN | CWIID_RPT_ACC
                    : CWIID_RPT_BTN);
            }
            if (pressed_buttons & CWIID_BTN_2) {
                ir_state = !ir_state;
                cwiid_set_rpt_mode(&wiimote, ir_state ? CWIID_RPT_BTN | CWIID_RPT_IR
                    : CWIID_RPT_BTN);
            }
            */

            last_buttons = buttons;
            break;

        case CWIID_MESG_ACC: {
            uint8_t x = mesg[i].acc_mesg.acc[CWIID_X];
            uint8_t y = mesg[i].acc_mesg.acc[CWIID_Y];
            uint8_t z = mesg[i].acc_mesg.acc[CWIID_Z];
            // void acc_add_event(uint8_t x, uint8_t y, uint8_t z);
            // acc_add_event(x, y, z);
            ESP_LOGI(TAG, "Acc Report: x=%+4d, y=%+4d, z=%+4d \n", x, y, z);
        }
                           break;

        case CWIID_MESG_IR:
            // ESP_LOGI(TAG, "IR Report: ");
            valid_source = 0;
            for (j = 0; j < CWIID_IR_SRC_COUNT; j++) {
                if (mesg[i].ir_mesg.src[j].valid) {
                    if (!valid_source) {
                        unsigned int x = 1023 - mesg[i].ir_mesg.src[j].pos[CWIID_X];
                        unsigned int y = mesg[i].ir_mesg.src[j].pos[CWIID_Y];
                        if (y > 750) {
                            y = 750;  // about 0-250 in CBM
                        }
                        if (x > 1006) {
                            x = 1006;  // about 0-249 in CBM
                        }
                        set_xy(x / 2, y / 3);
                    }
                    valid_source = 1;
                }
            }
            if (!valid_source) {
                // ESP_LOGI(TAG, "no sources detected");
            }
            break;
        case CWIID_MESG_NUNCHUK:
            ESP_LOGI(TAG,
                "Nunchuk Report: btns=%.2X stick=(%d,%d) acc.x=%d acc.y=%d "
                "acc.z=%d\n",
                mesg[i].nunchuk_mesg.buttons, mesg[i].nunchuk_mesg.stick[CWIID_X],
                mesg[i].nunchuk_mesg.stick[CWIID_Y],
                mesg[i].nunchuk_mesg.acc[CWIID_X],
                mesg[i].nunchuk_mesg.acc[CWIID_Y],
                mesg[i].nunchuk_mesg.acc[CWIID_Z]);
            break;
        case CWIID_MESG_CLASSIC:
            ESP_LOGI(TAG,
                "Classic Report: btns=%.4X l_stick=(%d,%d) r_stick=(%d,%d) "
                "l=%d r=%d\n",
                mesg[i].classic_mesg.buttons, mesg[i].classic_mesg.l_stick[CWIID_X],
                mesg[i].classic_mesg.l_stick[CWIID_Y],
                mesg[i].classic_mesg.r_stick[CWIID_X],
                mesg[i].classic_mesg.r_stick[CWIID_Y], mesg[i].classic_mesg.l,
                mesg[i].classic_mesg.r);
            break;
        case CWIID_MESG_BALANCE:
            ESP_LOGI(TAG,
                "Balance Report: right_top=%d right_bottom=%d "
                "left_top=%d left_bottom=%d\n",
                mesg[i].balance_mesg.right_top, mesg[i].balance_mesg.right_bottom,
                mesg[i].balance_mesg.left_top, mesg[i].balance_mesg.left_bottom);
            break;
        case CWIID_MESG_MOTIONPLUS:
            ESP_LOGI(TAG,
                "MotionPlus Report: angle_rate=(%d,%d,%d) low_speed=(%d,%d,%d)\n",
                mesg[i].motionplus_mesg.angle_rate[0],
                mesg[i].motionplus_mesg.angle_rate[1],
                mesg[i].motionplus_mesg.angle_rate[2],
                mesg[i].motionplus_mesg.low_speed[0],
                mesg[i].motionplus_mesg.low_speed[1],
                mesg[i].motionplus_mesg.low_speed[2]);
            break;
        case CWIID_MESG_ERROR:
            /*
            if (cwiid_close(wiimote)) {
                    fprintf(stderr, "Error on wiimote disconnect\n"); exit(-1);
            }
            exit(0);
              */
            break;
        default:
            ESP_LOGI(TAG, "Unknown Report");
            break;
        }
    }
}

static char* bda2str(esp_bd_addr_t bda, char* str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }
    uint8_t* p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4],
        p[5]);
    return str;
}

static void hidh_callback(void* handler_args, esp_event_base_t base, int32_t id,
    void* event_data) {
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t* param = (esp_hidh_event_data_t*)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        const uint8_t* bda = esp_hidh_dev_bda_get(param->open.dev);
        if (param->open.status == ESP_OK) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda),
                esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);

            wiimote.dev = param->open.dev;

            cwiid_set_led(&wiimote, 1);
            //cwiid_set_rpt_mode(&wiimote, CWIID_RPT_BTN);

            ir_state = 1;
            cwiid_set_rpt_mode(&wiimote, ir_state ? CWIID_RPT_BTN | CWIID_RPT_IR
                : CWIID_RPT_BTN);


        } else {
            const char* bt_status_str(esp_bt_status_t status);
            ESP_LOGE(TAG, "OPEN failed! %d %s", param->open.status, esp_err_to_name(param->open.status));

            // Something went wrong, clear all paired devices
            int wPairedDevices = esp_bt_gap_get_bond_device_num();
            if (wPairedDevices) {
#define C_PAIR_MAX_DEVICE 12
                if (C_PAIR_MAX_DEVICE < wPairedDevices) {
                    wPairedDevices = C_PAIR_MAX_DEVICE;
                }
                esp_bd_addr_t tPairedDeviceBtAddr[C_PAIR_MAX_DEVICE];
                esp_err_t tError = esp_bt_gap_get_bond_device_list(&wPairedDevices, tPairedDeviceBtAddr);
                if (ESP_OK == tError) {
                    for (int wDeviceToGet = 0; wDeviceToGet < wPairedDevices; wDeviceToGet++) {
                        tError = esp_bt_gap_remove_bond_device(tPairedDeviceBtAddr[wDeviceToGet]);
                        if (ESP_OK == tError) {
                            ESP_LOGI(TAG, "Paired device "ESP_BD_ADDR_STR" removed", ESP_BD_ADDR_HEX(tPairedDeviceBtAddr[wDeviceToGet]));
                        } else {
                            ESP_LOGE(TAG, "Paired device "ESP_BD_ADDR_STR" not removed", ESP_BD_ADDR_HEX(tPairedDeviceBtAddr[wDeviceToGet]));
                        }
                    }
                }

            }
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t* bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda),
            param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t* bda = esp_hidh_dev_bda_get(param->input.dev);
        if (!ir_state && !acc_state) {
            ESP_LOGI(TAG,
                ESP_BD_ADDR_STR
                " INPUT: %8s, MAP: %2u, ID: %3u %s, Len: %d, Data:",
                ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage),
                param->input.map_index, param->input.report_id,
                rpt2str(param->input.report_id), param->input.length);
            ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        }

        struct mesg_array ma;
        parse_input_msg(&wiimote, param->input.report_id, param->input.data,
            param->input.length, &ma);
        cwiid_handle_message(&ma);

        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t* bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d",
            ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->feature.usage),
            param->feature.map_index, param->feature.report_id,
            param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t* bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda),
            esp_hidh_dev_name_get(param->close.dev));
        wiimote.dev = 0;
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

static void idle_disconnect_task(void* arg) {
    int64_t now = esp_timer_get_time();
    // Disconnect after 5 min
    if (wiimote.dev && now - last_activity_time > 300 * 1000 * 1000) {
        ESP_LOGI(TAG, "Idle disconnect");
        esp_bt_hid_host_disconnect(esp_hidh_dev_bda_get(wiimote.dev));
    }
}

#define SCAN_DURATION_SECONDS 5

static void hid_init_task(void* pvParameters) {
    size_t results_len = 0;
    esp_hid_scan_result_t* results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    // start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t* r = results;
        esp_hid_scan_result_t* cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ",
                (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ",
                ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%d', ", r->ble.addr_type);
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        if (cr) {
            // open the last result
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
        // free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}

#if CONFIG_BT_NIMBLE_ENABLED
void ble_hid_host_task(void* param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
void ble_store_config_init(void);
#endif

void bt_init() {
    char bda_str[18] = { 0 };

#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK(
        esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
#endif /* CONFIG_BT_BLE_ENABLED */

    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));

    ESP_LOGI(
        TAG, "Own address:[%s]",
        bda2str((uint8_t*)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
#if CONFIG_BT_NIMBLE_ENABLED
    /* XXX Need to have template for store */
    ble_store_config_init();

    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    /* Starting nimble task after gatts is initialized*/
    esp_err_t ret;
    ret = esp_nimble_enable(ble_hid_host_task);
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }
#endif
    xTaskCreate(&hid_init_task, "hid_task", 6 * 1024, NULL, 2, NULL);


    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &idle_disconnect_task, .name = "idle_check"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 60 * 1000 * 1000));
}