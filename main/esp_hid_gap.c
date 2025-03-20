/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#define CONFIG_EXAMPLE_SSP_ENABLED 1

#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_hid_gap.h"
#if CONFIG_BT_BLUEDROID_ENABLED
#include "esp_bt_device.h"
#endif
#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "nimble/ble.h"
#include "host/ble_sm.h"
#define BLE_HID_SVC_UUID 0x1812          /* HID Service*/
#else
#include "esp_bt_device.h"
#endif

static const char* TAG = "ESP_HID_GAP";

// uncomment to print all devices that were seen during a scan
#define GAP_DBG_PRINTF(...) printf(__VA_ARGS__)

#if CONFIG_BT_HID_HOST_ENABLED
static const char* gap_bt_prop_type_names[5] = { "","BDNAME","COD","RSSI","EIR" };
#endif

static esp_hid_scan_result_t* bt_scan_results = NULL;
static size_t num_bt_scan_results = 0;

static esp_hid_scan_result_t* ble_scan_results = NULL;
static size_t num_ble_scan_results = 0;

static SemaphoreHandle_t bt_hidh_cb_semaphore = NULL;
#define WAIT_BT_CB() xSemaphoreTake(bt_hidh_cb_semaphore, portMAX_DELAY)
#define SEND_BT_CB() xSemaphoreGive(bt_hidh_cb_semaphore)

static SemaphoreHandle_t ble_hidh_cb_semaphore = NULL;
#define WAIT_BLE_CB() xSemaphoreTake(ble_hidh_cb_semaphore, portMAX_DELAY)
#define SEND_BLE_CB() xSemaphoreGive(ble_hidh_cb_semaphore)

#define SIZEOF_ARRAY(a) (sizeof(a)/sizeof(*a))

#if !CONFIG_BT_NIMBLE_ENABLED
static const char* ble_gap_evt_names[] = {
    "ADV_DATA_SET_COMPLETE",
    "SCAN_RSP_DATA_SET_COMPLETE",
    "SCAN_PARAM_SET_COMPLETE",
    "SCAN_RESULT",
    "ADV_DATA_RAW_SET_COMPLETE",
    "SCAN_RSP_DATA_RAW_SET_COMPLETE",
    "ADV_START_COMPLETE",
    "SCAN_START_COMPLETE",
    "AUTH_CMPL",
    "KEY",
    "SEC_REQ",
    "PASSKEY_NOTIF",
    "PASSKEY_REQ",
    "OOB_REQ",
    "LOCAL_IR",
    "LOCAL_ER",
    "NC_REQ",
    "ADV_STOP_COMPLETE",
    "SCAN_STOP_COMPLETE",
    "SET_STATIC_RAND_ADDR",
    "UPDATE_CONN_PARAMS",
    "SET_PKT_LENGTH_COMPLETE",
    "SET_LOCAL_PRIVACY_COMPLETE",
    "REMOVE_BOND_DEV_COMPLETE",
    "CLEAR_BOND_DEV_COMPLETE",
    "GET_BOND_DEV_COMPLETE",
    "READ_RSSI_COMPLETE",
    "UPDATE_WHITELIST_COMPLETE"
};
static const char* bt_gap_evt_names[] = {
    "DISC_RES",
    "DISC_STATE_CHANGED",
    "RMT_SRVCS",
    "RMT_SRVC_REC",
    "AUTH_CMPL",
    "PIN_REQ",
    "CFM_REQ",
    "KEY_NOTIF",
    "KEY_REQ",
    "READ_RSSI_DELTA",
    "CONFIG_EIR_DATA",                 /*!< Config EIR data event */
    "SET_AFH_CHANNELS",                /*!< Set AFH channels event */
    "READ_REMOTE_NAME",                /*!< Read Remote Name event */
    "MODE_CHG",
    "REMOVE_BOND_DEV_COMPLETE",         /*!< remove bond device complete event */
    "QOS_CMPL",                        /*!< QOS complete event */
    "ACL_CONN_CMPL_STAT",              /*!< ACL connection complete status event */
    "ACL_DISCONN_CMPL_STAT",           /*!< ACL disconnection complete status event */
    "SET_PAGE_TO",                     /*!< Set page timeout event */
    "GET_PAGE_TO",                     /*!< Get page timeout event */
    "ACL_PKT_TYPE_CHANGED",            /*!< Set ACL packet types event */
    "ENC_CHG",                         /*!< Encryption change event */
    "SET_MIN_ENC_KEY_SIZE",            /*!< Set minimum encryption key size */
    "GET_DEV_NAME_CMPL",               /*!< Get device name complete event */
};
static const char* ble_addr_type_names[] = { "PUBLIC", "RANDOM", "RPA_PUBLIC", "RPA_RANDOM" };

const char* ble_addr_type_str(esp_ble_addr_type_t ble_addr_type) {
    if (ble_addr_type > BLE_ADDR_TYPE_RPA_RANDOM) {
        return "UNKNOWN";
    }
    return ble_addr_type_names[ble_addr_type];
}

const char* ble_gap_evt_str(uint8_t event) {
    if (event >= SIZEOF_ARRAY(ble_gap_evt_names)) {
        return "UNKNOWN";
    }
    return ble_gap_evt_names[event];
}

const char* bt_gap_evt_str(uint8_t event) {
    if (event >= SIZEOF_ARRAY(bt_gap_evt_names)) {
        return "UNKNOWN";
    }
    return bt_gap_evt_names[event];
}

const char* bt_status_str(esp_bt_status_t status) {
    switch (status) {
    case ESP_BT_STATUS_SUCCESS: return "SUCCESS";
    case ESP_BT_STATUS_FAIL: return "FAIL";
    case ESP_BT_STATUS_NOT_READY: return "NOT_READY";
    case ESP_BT_STATUS_NOMEM: return "NOMEM";
    case ESP_BT_STATUS_BUSY: return "BUSY";
    case ESP_BT_STATUS_DONE: return "DONE";
    case ESP_BT_STATUS_UNSUPPORTED: return "UNSUPPORTED";
    case ESP_BT_STATUS_PARM_INVALID: return "PARM_INVALID";
    case ESP_BT_STATUS_UNHANDLED: return "UNHANDLED";
    case ESP_BT_STATUS_AUTH_FAILURE: return "AUTH_FAILURE";
    case ESP_BT_STATUS_RMT_DEV_DOWN: return "RMT_DEV_DOWN";
    case ESP_BT_STATUS_AUTH_REJECTED: return "AUTH_REJECTED";
    case ESP_BT_STATUS_INVALID_STATIC_RAND_ADDR: return "INVALID_STATIC_RAND_ADDR";
    case ESP_BT_STATUS_PENDING: return "PENDING";
    case ESP_BT_STATUS_UNACCEPT_CONN_INTERVAL: return "UNACCEPT_CONN_INTERVAL";
    case ESP_BT_STATUS_PARAM_OUT_OF_RANGE: return "PARAM_OUT_OF_RANGE";
    case ESP_BT_STATUS_TIMEOUT: return "TIMEOUT";
    case ESP_BT_STATUS_PEER_LE_DATA_LEN_UNSUPPORTED: return "PEER_LE_DATA_LEN_UNSUPPORTED";
    case ESP_BT_STATUS_CONTROL_LE_DATA_LEN_UNSUPPORTED: return "CONTROL_LE_DATA_LEN_UNSUPPORTED";
    case ESP_BT_STATUS_ERR_ILLEGAL_PARAMETER_FMT: return "ERR_ILLEGAL_PARAMETER_FMT";
    case ESP_BT_STATUS_MEMORY_FULL: return "MEMORY_FULL";
    case ESP_BT_STATUS_EIR_TOO_LARGE: return "EIR_TOO_LARGE";
    case ESP_BT_STATUS_HCI_SUCCESS: return "HCI_SUCCESS";
    case ESP_BT_STATUS_HCI_ILLEGAL_COMMAND: return "HCI_ILLEGAL_COMMAND";
    case ESP_BT_STATUS_HCI_NO_CONNECTION: return "HCI_NO_CONNECTION";
    case ESP_BT_STATUS_HCI_HW_FAILURE: return "HCI_HW_FAILURE";
    case ESP_BT_STATUS_HCI_PAGE_TIMEOUT: return "HCI_PAGE_TIMEOUT";
    case ESP_BT_STATUS_HCI_AUTH_FAILURE: return "HCI_AUTH_FAILURE";
    case ESP_BT_STATUS_HCI_KEY_MISSING: return "HCI_KEY_MISSING";
    case ESP_BT_STATUS_HCI_MEMORY_FULL: return "HCI_MEMORY_FULL";
    case ESP_BT_STATUS_HCI_CONNECTION_TOUT: return "HCI_CONNECTION_TOUT";
    case ESP_BT_STATUS_HCI_MAX_NUM_OF_CONNECTIONS: return "HCI_MAX_NUM_OF_CONNECTIONS";
    case ESP_BT_STATUS_HCI_MAX_NUM_OF_SCOS: return "HCI_MAX_NUM_OF_SCOS";
    case ESP_BT_STATUS_HCI_CONNECTION_EXISTS: return "HCI_CONNECTION_EXISTS";
    case ESP_BT_STATUS_HCI_COMMAND_DISALLOWED: return "HCI_COMMAND_DISALLOWED";
    case ESP_BT_STATUS_HCI_HOST_REJECT_RESOURCES: return "HCI_HOST_REJECT_RESOURCES";
    case ESP_BT_STATUS_HCI_HOST_REJECT_SECURITY: return "HCI_HOST_REJECT_SECURITY";
    case ESP_BT_STATUS_HCI_HOST_REJECT_DEVICE: return "HCI_HOST_REJECT_DEVICE";
    case ESP_BT_STATUS_HCI_HOST_TIMEOUT: return "HCI_HOST_TIMEOUT";
    case ESP_BT_STATUS_HCI_UNSUPPORTED_VALUE: return "HCI_UNSUPPORTED_VALUE";
    case ESP_BT_STATUS_HCI_ILLEGAL_PARAMETER_FMT: return "HCI_ILLEGAL_PARAMETER_FMT";
    case ESP_BT_STATUS_HCI_PEER_USER: return "HCI_PEER_USER";
    case ESP_BT_STATUS_HCI_PEER_LOW_RESOURCES: return "HCI_PEER_LOW_RESOURCES";
    case ESP_BT_STATUS_HCI_PEER_POWER_OFF: return "HCI_PEER_POWER_OFF";
    case ESP_BT_STATUS_HCI_CONN_CAUSE_LOCAL_HOST: return "HCI_CONN_CAUSE_LOCAL_HOST";
    case ESP_BT_STATUS_HCI_REPEATED_ATTEMPTS: return "HCI_REPEATED_ATTEMPTS";
    case ESP_BT_STATUS_HCI_PAIRING_NOT_ALLOWED: return "HCI_PAIRING_NOT_ALLOWED";
    case ESP_BT_STATUS_HCI_UNKNOWN_LMP_PDU: return "HCI_UNKNOWN_LMP_PDU";
    case ESP_BT_STATUS_HCI_UNSUPPORTED_REM_FEATURE: return "HCI_UNSUPPORTED_REM_FEATURE";
    case ESP_BT_STATUS_HCI_SCO_OFFSET_REJECTED: return "HCI_SCO_OFFSET_REJECTED";
    case ESP_BT_STATUS_HCI_SCO_INTERVAL_REJECTED: return "HCI_SCO_INTERVAL_REJECTED";
    case ESP_BT_STATUS_HCI_SCO_AIR_MODE: return "HCI_SCO_AIR_MODE";
    case ESP_BT_STATUS_HCI_INVALID_LMP_PARAM: return "HCI_INVALID_LMP_PARAM";
    case ESP_BT_STATUS_HCI_UNSPECIFIED: return "HCI_UNSPECIFIED";
    case ESP_BT_STATUS_HCI_UNSUPPORTED_LMP_PARAMETERS: return "HCI_UNSUPPORTED_LMP_PARAMETERS";
    case ESP_BT_STATUS_HCI_ROLE_CHANGE_NOT_ALLOWED: return "HCI_ROLE_CHANGE_NOT_ALLOWED";
    case ESP_BT_STATUS_HCI_LMP_RESPONSE_TIMEOUT: return "HCI_LMP_RESPONSE_TIMEOUT";
    case ESP_BT_STATUS_HCI_LMP_ERR_TRANS_COLLISION: return "HCI_LMP_ERR_TRANS_COLLISION";
    case ESP_BT_STATUS_HCI_LMP_PDU_NOT_ALLOWED: return "HCI_LMP_PDU_NOT_ALLOWED";
    case ESP_BT_STATUS_HCI_ENCRY_MODE_NOT_ACCEPTABLE: return "HCI_ENCRY_MODE_NOT_ACCEPTABLE";
    case ESP_BT_STATUS_HCI_UNIT_KEY_USED: return "HCI_UNIT_KEY_USED";
    case ESP_BT_STATUS_HCI_QOS_NOT_SUPPORTED: return "HCI_QOS_NOT_SUPPORTED";
    case ESP_BT_STATUS_HCI_INSTANT_PASSED: return "HCI_INSTANT_PASSED";
    case ESP_BT_STATUS_HCI_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED: return "HCI_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED";
    case ESP_BT_STATUS_HCI_DIFF_TRANSACTION_COLLISION: return "HCI_DIFF_TRANSACTION_COLLISION";
    case ESP_BT_STATUS_HCI_UNDEFINED_0x2B: return "HCI_UNDEFINED_0x2B";
    case ESP_BT_STATUS_HCI_QOS_UNACCEPTABLE_PARAM: return "HCI_QOS_UNACCEPTABLE_PARAM";
    case ESP_BT_STATUS_HCI_QOS_REJECTED: return "HCI_QOS_REJECTED";
    case ESP_BT_STATUS_HCI_CHAN_CLASSIF_NOT_SUPPORTED: return "HCI_CHAN_CLASSIF_NOT_SUPPORTED";
    case ESP_BT_STATUS_HCI_INSUFFCIENT_SECURITY: return "HCI_INSUFFCIENT_SECURITY";
    case ESP_BT_STATUS_HCI_PARAM_OUT_OF_RANGE: return "HCI_PARAM_OUT_OF_RANGE";
    case ESP_BT_STATUS_HCI_UNDEFINED_0x31: return "HCI_UNDEFINED_0x31";
    case ESP_BT_STATUS_HCI_ROLE_SWITCH_PENDING: return "HCI_ROLE_SWITCH_PENDING";
    case ESP_BT_STATUS_HCI_UNDEFINED_0x33: return "HCI_UNDEFINED_0x33";
    case ESP_BT_STATUS_HCI_RESERVED_SLOT_VIOLATION: return "HCI_RESERVED_SLOT_VIOLATION";
    case ESP_BT_STATUS_HCI_ROLE_SWITCH_FAILED: return "HCI_ROLE_SWITCH_FAILED";
    case ESP_BT_STATUS_HCI_INQ_RSP_DATA_TOO_LARGE: return "HCI_INQ_RSP_DATA_TOO_LARGE";
    case ESP_BT_STATUS_HCI_SIMPLE_PAIRING_NOT_SUPPORTED: return "HCI_SIMPLE_PAIRING_NOT_SUPPORTED";
    case ESP_BT_STATUS_HCI_HOST_BUSY_PAIRING: return "HCI_HOST_BUSY_PAIRING";
    case ESP_BT_STATUS_HCI_REJ_NO_SUITABLE_CHANNEL: return "HCI_REJ_NO_SUITABLE_CHANNEL";
    case ESP_BT_STATUS_HCI_CONTROLLER_BUSY: return "HCI_CONTROLLER_BUSY";
    case ESP_BT_STATUS_HCI_UNACCEPT_CONN_INTERVAL: return "HCI_UNACCEPT_CONN_INTERVAL";
    case ESP_BT_STATUS_HCI_DIRECTED_ADVERTISING_TIMEOUT: return "HCI_DIRECTED_ADVERTISING_TIMEOUT";
    case ESP_BT_STATUS_HCI_CONN_TOUT_DUE_TO_MIC_FAILURE: return "HCI_CONN_TOUT_DUE_TO_MIC_FAILURE";
    case ESP_BT_STATUS_HCI_CONN_FAILED_ESTABLISHMENT: return "HCI_CONN_FAILED_ESTABLISHMENT";
    case ESP_BT_STATUS_HCI_MAC_CONNECTION_FAILED: return "HCI_MAC_CONNECTION_FAILED";
    }
    return "***UNKNOWN***";
}


#endif
#if CONFIG_BT_BLE_ENABLED
const char* esp_ble_key_type_str(esp_ble_key_type_t key_type) {
    const char* key_str = NULL;
    switch (key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

    }
    return key_str;
}
#endif /* CONFIG_BT_BLE_ENABLED */

void esp_hid_scan_results_free(esp_hid_scan_result_t* results) {
    esp_hid_scan_result_t* r = NULL;
    while (results) {
        r = results;
        results = results->next;
        if (r->name != NULL) {
            free((char*)r->name);
        }
        free(r);
    }
}

#if (CONFIG_BT_HID_HOST_ENABLED || CONFIG_BT_BLE_ENABLED)
static esp_hid_scan_result_t* find_scan_result(esp_bd_addr_t bda, esp_hid_scan_result_t* results) {
    esp_hid_scan_result_t* r = results;
    while (r) {
        if (memcmp(bda, r->bda, sizeof(esp_bd_addr_t)) == 0) {
            return r;
        }
        r = r->next;
    }
    return NULL;
}
#endif /* (CONFIG_BT_HID_HOST_ENABLED || CONFIG_BT_BLE_ENABLED) */

#if (CONFIG_BT_NIMBLE_ENABLED)
static esp_hid_scan_result_t* find_scan_result(const uint8_t* bda, esp_hid_scan_result_t* results) {
    esp_hid_scan_result_t* r = results;
    while (r) {
        if (memcmp(bda, r->bda, sizeof(r->bda)) == 0) {
            return r;
        }
        r = r->next;
    }
    return NULL;
}
#endif
#if CONFIG_BT_HID_HOST_ENABLED
static void add_bt_scan_result(esp_bd_addr_t bda, esp_bt_cod_t* cod, esp_bt_uuid_t* uuid, uint8_t* name, uint8_t name_len, int rssi) {
    esp_hid_scan_result_t* r = find_scan_result(bda, bt_scan_results);
    if (r) {
        //Some info may come later
        if (r->name == NULL && name && name_len) {
            char* name_s = (char*)malloc(name_len + 1);
            if (name_s == NULL) {
                ESP_LOGE(TAG, "Malloc result name failed!");
                return;
            }
            memcpy(name_s, name, name_len);
            name_s[name_len] = 0;
            r->name = (const char*)name_s;
        }
        if (r->bt.uuid.len == 0 && uuid->len) {
            memcpy(&r->bt.uuid, uuid, sizeof(esp_bt_uuid_t));
        }
        if (rssi != 0) {
            r->rssi = rssi;
        }
        return;
    }

    r = (esp_hid_scan_result_t*)malloc(sizeof(esp_hid_scan_result_t));
    if (r == NULL) {
        ESP_LOGE(TAG, "Malloc bt_hidh_scan_result_t failed!");
        return;
    }
    r->transport = ESP_HID_TRANSPORT_BT;
    memcpy(r->bda, bda, sizeof(esp_bd_addr_t));
    memcpy(&r->bt.cod, cod, sizeof(esp_bt_cod_t));
    memcpy(&r->bt.uuid, uuid, sizeof(esp_bt_uuid_t));
    r->usage = esp_hid_usage_from_cod((uint32_t)cod);
    r->rssi = rssi;
    r->name = NULL;
    if (name_len && name) {
        char* name_s = (char*)malloc(name_len + 1);
        if (name_s == NULL) {
            free(r);
            ESP_LOGE(TAG, "Malloc result name failed!");
            return;
        }
        memcpy(name_s, name, name_len);
        name_s[name_len] = 0;
        r->name = (const char*)name_s;
    }
    r->next = bt_scan_results;
    bt_scan_results = r;
    num_bt_scan_results++;
}
#endif

#if CONFIG_BT_BLE_ENABLED
static void add_ble_scan_result(esp_bd_addr_t bda, esp_ble_addr_type_t addr_type, uint16_t appearance, uint8_t* name, uint8_t name_len, int rssi) {
    if (find_scan_result(bda, ble_scan_results)) {
        ESP_LOGW(TAG, "Result already exists!");
        return;
    }
    esp_hid_scan_result_t* r = (esp_hid_scan_result_t*)malloc(sizeof(esp_hid_scan_result_t));
    if (r == NULL) {
        ESP_LOGE(TAG, "Malloc ble_hidh_scan_result_t failed!");
        return;
    }
    r->transport = ESP_HID_TRANSPORT_BLE;
    memcpy(r->bda, bda, sizeof(esp_bd_addr_t));
    r->ble.appearance = appearance;
    r->ble.addr_type = addr_type;
    r->usage = esp_hid_usage_from_appearance(appearance);
    r->rssi = rssi;
    r->name = NULL;
    if (name_len && name) {
        char* name_s = (char*)malloc(name_len + 1);
        if (name_s == NULL) {
            free(r);
            ESP_LOGE(TAG, "Malloc result name failed!");
            return;
        }
        memcpy(name_s, name, name_len);
        name_s[name_len] = 0;
        r->name = (const char*)name_s;
    }
    r->next = ble_scan_results;
    ble_scan_results = r;
    num_ble_scan_results++;
}
#endif /* CONFIG_BT_BLE_ENABLED */

#if CONFIG_BT_NIMBLE_ENABLED
static void add_ble_scan_result(const uint8_t* bda, uint8_t addr_type, uint16_t appearance, uint8_t* name, uint8_t name_len, int rssi) {
    if (find_scan_result(bda, ble_scan_results)) {
        ESP_LOGW(TAG, "Result already exists!");
        return;
    }
    esp_hid_scan_result_t* r = (esp_hid_scan_result_t*)malloc(sizeof(esp_hid_scan_result_t));
    if (r == NULL) {
        ESP_LOGE(TAG, "Malloc ble_hidh_scan_result_t failed!");
        return;
    }
    r->transport = ESP_HID_TRANSPORT_BLE;
    memcpy(r->bda, bda, sizeof(r->bda));
    r->ble.appearance = appearance;
    r->ble.addr_type = addr_type;
    r->usage = esp_hid_usage_from_appearance(appearance);
    r->rssi = rssi;
    r->name = NULL;
    if (name_len && name) {
        char* name_s = (char*)malloc(name_len + 1);
        if (name_s == NULL) {
            free(r);
            ESP_LOGE(TAG, "Malloc result name failed!");
            return;
        }
        memcpy(name_s, name, name_len);
        name_s[name_len] = 0;
        r->name = (const char*)name_s;
    }
    r->next = ble_scan_results;
    ble_scan_results = r;
    num_ble_scan_results++;
}
#endif /* CONFIG_BT_BLE_ENABLED */

#if !CONFIG_BT_NIMBLE_ENABLED
void print_uuid(esp_bt_uuid_t* uuid) {
    if (uuid->len == ESP_UUID_LEN_16) {
        GAP_DBG_PRINTF("UUID16: 0x%04x", uuid->uuid.uuid16);
    } else if (uuid->len == ESP_UUID_LEN_32) {
        GAP_DBG_PRINTF("UUID32: 0x%08"PRIx32, uuid->uuid.uuid32);
    } else if (uuid->len == ESP_UUID_LEN_128) {
        GAP_DBG_PRINTF("UUID128: %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x", uuid->uuid.uuid128[0],
            uuid->uuid.uuid128[1], uuid->uuid.uuid128[2], uuid->uuid.uuid128[3],
            uuid->uuid.uuid128[4], uuid->uuid.uuid128[5], uuid->uuid.uuid128[6],
            uuid->uuid.uuid128[7], uuid->uuid.uuid128[8], uuid->uuid.uuid128[9],
            uuid->uuid.uuid128[10], uuid->uuid.uuid128[11], uuid->uuid.uuid128[12],
            uuid->uuid.uuid128[13], uuid->uuid.uuid128[14], uuid->uuid.uuid128[15]);
    }
}

#if CONFIG_BT_HID_HOST_ENABLED
static void handle_bt_device_result(struct disc_res_param* disc_res) {
    GAP_DBG_PRINTF("BT : " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(disc_res->bda));
    uint32_t codv = 0;
    esp_bt_cod_t* cod = (esp_bt_cod_t*)&codv;
    int8_t rssi = 0;
    uint8_t* name = NULL;
    uint8_t name_len = 0;
    esp_bt_uuid_t uuid;

    uuid.len = ESP_UUID_LEN_16;
    uuid.uuid.uuid16 = 0;

    for (int i = 0; i < disc_res->num_prop; i++) {
        esp_bt_gap_dev_prop_t* prop = &disc_res->prop[i];
        if (prop->type != ESP_BT_GAP_DEV_PROP_EIR) {
            GAP_DBG_PRINTF(", %s: ", gap_bt_prop_type_names[prop->type]);
        }
        if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
            name = (uint8_t*)prop->val;
            name_len = strlen((const char*)name);
            GAP_DBG_PRINTF("%s", (const char*)name);
        } else if (prop->type == ESP_BT_GAP_DEV_PROP_RSSI) {
            rssi = *((int8_t*)prop->val);
            GAP_DBG_PRINTF("%d", rssi);
        } else if (prop->type == ESP_BT_GAP_DEV_PROP_COD) {
            memcpy(&codv, prop->val, sizeof(uint32_t));
            GAP_DBG_PRINTF("major: %s, minor: %d, service: 0x%03x", esp_hid_cod_major_str(cod->major), cod->minor, cod->service);
        } else if (prop->type == ESP_BT_GAP_DEV_PROP_EIR) {
            uint8_t len = 0;
            uint8_t* data = 0;

            data = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val, ESP_BT_EIR_TYPE_CMPL_16BITS_UUID, &len);
            if (data == NULL) {
                data = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val, ESP_BT_EIR_TYPE_INCMPL_16BITS_UUID, &len);
            }
            if (data && len == ESP_UUID_LEN_16) {
                uuid.len = ESP_UUID_LEN_16;
                uuid.uuid.uuid16 = data[0] + (data[1] << 8);
                GAP_DBG_PRINTF(", "); print_uuid(&uuid);
                continue;
            }

            data = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val, ESP_BT_EIR_TYPE_CMPL_32BITS_UUID, &len);
            if (data == NULL) {
                data = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val, ESP_BT_EIR_TYPE_INCMPL_32BITS_UUID, &len);
            }
            if (data && len == ESP_UUID_LEN_32) {
                uuid.len = len;
                memcpy(&uuid.uuid.uuid32, data, sizeof(uint32_t));
                GAP_DBG_PRINTF(", "); print_uuid(&uuid);
                continue;
            }

            data = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val, ESP_BT_EIR_TYPE_CMPL_128BITS_UUID, &len);
            if (data == NULL) {
                data = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val, ESP_BT_EIR_TYPE_INCMPL_128BITS_UUID, &len);
            }
            if (data && len == ESP_UUID_LEN_128) {
                uuid.len = len;
                memcpy(uuid.uuid.uuid128, (uint8_t*)data, len);
                GAP_DBG_PRINTF(", "); print_uuid(&uuid);
                continue;
            }

            //try to find a name
            if (name == NULL) {
                data = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
                if (data == NULL) {
                    data = esp_bt_gap_resolve_eir_data((uint8_t*)prop->val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
                }
                if (data && len) {
                    name = data;
                    name_len = len;
                    GAP_DBG_PRINTF(", NAME: ");
                    for (int x = 0; x < len; x++) {
                        GAP_DBG_PRINTF("%c", (char)data[x]);
                    }
                }
            }
        }
    }
    GAP_DBG_PRINTF("\n");

    if (cod->major == ESP_BT_COD_MAJOR_DEV_PERIPHERAL || (find_scan_result(disc_res->bda, bt_scan_results) != NULL)) {
        add_bt_scan_result(disc_res->bda, cod, &uuid, name, name_len, rssi);
    }
}
#endif

#if CONFIG_BT_BLE_ENABLED
static void handle_ble_device_result(struct ble_scan_result_evt_param* scan_rst) {

    uint16_t uuid = 0;
    uint16_t appearance = 0;
    char name[64] = { 0 };

    uint8_t uuid_len = 0;
    uint8_t* uuid_d = esp_ble_resolve_adv_data(scan_rst->ble_adv, ESP_BLE_AD_TYPE_16SRV_CMPL, &uuid_len);
    if (uuid_d != NULL && uuid_len) {
        uuid = uuid_d[0] + (uuid_d[1] << 8);
    }

    uint8_t appearance_len = 0;
    uint8_t* appearance_d = esp_ble_resolve_adv_data(scan_rst->ble_adv, ESP_BLE_AD_TYPE_APPEARANCE, &appearance_len);
    if (appearance_d != NULL && appearance_len) {
        appearance = appearance_d[0] + (appearance_d[1] << 8);
    }

    uint8_t adv_name_len = 0;
    uint8_t* adv_name = esp_ble_resolve_adv_data(scan_rst->ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);

    if (adv_name == NULL) {
        adv_name = esp_ble_resolve_adv_data(scan_rst->ble_adv, ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);
    }

    if (adv_name != NULL && adv_name_len) {
        memcpy(name, adv_name, adv_name_len);
        name[adv_name_len] = 0;
    }

    GAP_DBG_PRINTF("BLE: " ESP_BD_ADDR_STR ", ", ESP_BD_ADDR_HEX(scan_rst->bda));
    GAP_DBG_PRINTF("RSSI: %d, ", scan_rst->rssi);
    GAP_DBG_PRINTF("UUID: 0x%04x, ", uuid);
    GAP_DBG_PRINTF("APPEARANCE: 0x%04x, ", appearance);
    GAP_DBG_PRINTF("ADDR_TYPE: '%s'", ble_addr_type_str(scan_rst->ble_addr_type));
    if (adv_name_len) {
        GAP_DBG_PRINTF(", NAME: '%s'", name);
    }
    GAP_DBG_PRINTF("\n");

    if (uuid == ESP_GATT_UUID_HID_SVC) {
        add_ble_scan_result(scan_rst->bda, scan_rst->ble_addr_type, appearance, adv_name, adv_name_len, scan_rst->rssi);
    }
}
#endif /* CONFIG_BT_BLE_ENABLED */

#if CONFIG_BT_HID_HOST_ENABLED
/*
 * BT GAP
 * */

static char* bda2str(const esp_bd_addr_t bda, char* str, size_t size) {
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }
    const uint8_t* p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4],
        p[5]);
    return str;
}

static void bt_gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param) {
    switch (event) {
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        ESP_LOGV(TAG, "BT GAP DISC_STATE %s", (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) ? "START" : "STOP");
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            SEND_BT_CB();
        }
        break;
    }
    case ESP_BT_GAP_DISC_RES_EVT: {
        handle_bt_device_result(&param->disc_res);
        break;
    }
#if (CONFIG_EXAMPLE_SSP_ENABLED)
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "BT GAP KEY_NOTIF passkey:%"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_CFM_REQ_EVT: {
        ESP_LOGI(TAG, "BT GAP CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    }
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "BT GAP KEY_REQ_EVT Please enter passkey!");
        break;
#endif
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "BT GAP MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(TAG, "BT GAP PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = { 0 };
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            char bda_str[18] = { 0 };
            ESP_LOGI(
                TAG, "Wiimote address: [%s]",
                bda2str((uint8_t*)param->pin_req.bda, bda_str, sizeof(bda_str)));
            esp_bt_pin_code_t pin_code = { 0 };
            pin_code[0] = param->pin_req.bda[5];
            pin_code[1] = param->pin_req.bda[4];
            pin_code[2] = param->pin_req.bda[3];
            pin_code[3] = param->pin_req.bda[2];
            pin_code[4] = param->pin_req.bda[1];
            pin_code[5] = param->pin_req.bda[0];
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 6, pin_code);
            ESP_LOGI(TAG, "Input pin code: %s", bda2str((uint8_t*)pin_code, bda_str, sizeof(bda_str)));
        }
        break;
    }

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGW(TAG, "BT GAP EVENT %s reason: %s", bt_gap_evt_str(event), bt_status_str(param->acl_disconn_cmpl_stat.reason));
        break;

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        ESP_LOGW(TAG, "BT GAP EVENT %s stat: %s", bt_gap_evt_str(event), bt_status_str(param->acl_conn_cmpl_stat.stat));
        break;

    default:
        ESP_LOGW(TAG, "BT GAP EVENT %s %d", bt_gap_evt_str(event), event);
        break;
    }
}

static esp_err_t init_bt_gap(void) {
    esp_err_t ret;
#if (CONFIG_EXAMPLE_SSP_ENABLED)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif
    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    if ((ret = esp_bt_gap_register_callback(bt_gap_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_register_callback failed: %d", ret);
        return ret;
    }

    // Allow BT devices to connect back to us
    if ((ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_set_scan_mode failed: %d", ret);
        return ret;
    }
    return ret;
}

static esp_err_t start_bt_scan(uint32_t seconds) {
    esp_err_t ret = ESP_OK;
    if ((ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, (int)(seconds / 1.28), 0)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_start_discovery failed: %d", ret);
        return ret;
    }
    return ret;
}
#endif

#if CONFIG_BT_BLE_ENABLED
/*
 * BLE GAP
 * */

static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch (event) {
        /*
         * SCAN
         * */
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        ESP_LOGV(TAG, "BLE GAP EVENT SCAN_PARAM_SET_COMPLETE");
        SEND_BLE_CB();
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            handle_ble_device_result(&scan_result->scan_rst);
            break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGV(TAG, "BLE GAP EVENT SCAN DONE: %d", scan_result->scan_rst.num_resps);
            SEND_BLE_CB();
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
        ESP_LOGV(TAG, "BLE GAP EVENT SCAN CANCELED");
        break;
    }

                                           /*
                                            * ADVERTISEMENT
                                            * */
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP ADV_DATA_SET_COMPLETE");
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP ADV_START_COMPLETE");
        break;

        /*
         * AUTHENTICATION
         * */
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(TAG, "BLE GAP AUTH ERROR: 0x%x", param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(TAG, "BLE GAP AUTH SUCCESS");
        }
        break;

    case ESP_GAP_BLE_KEY_EVT: //shows the ble key info share with peer device to the user.
        ESP_LOGI(TAG, "BLE GAP KEY type = %s", esp_ble_key_type_str(param->ble_security.ble_key.key_type));
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: // ESP_IO_CAP_OUT
        // The app will receive this evt when the IO has Output capability and the peer device IO has Input capability.
        // Show the passkey number to the user to input it in the peer device.
        ESP_LOGI(TAG, "BLE GAP PASSKEY_NOTIF passkey:%"PRIu32, param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT: // ESP_IO_CAP_IO
        // The app will receive this event when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        // show the passkey number to the user to confirm it with the number displayed by peer device.
        ESP_LOGI(TAG, "BLE GAP NC_REQ passkey:%"PRIu32, param->ble_security.key_notif.passkey);
        esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT: // ESP_IO_CAP_IN
        // The app will receive this evt when the IO has Input capability and the peer device IO has Output capability.
        // See the passkey number on the peer device and send it back.
        ESP_LOGI(TAG, "BLE GAP PASSKEY_REQ");
        //esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 1234);
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "BLE GAP SEC_REQ");
        // Send the positive(true) security response to the peer device to accept the security request.
        // If not accept the security request, should send the security response with negative(false) accept value.
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    default:
        ESP_LOGV(TAG, "BLE GAP EVENT %s", ble_gap_evt_str(event));
        break;
    }
}
#endif

static esp_err_t init_ble_gap(void) {
    esp_err_t ret;

    if ((ret = esp_ble_gap_register_callback(ble_gap_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", ret);
        return ret;
    }
    return ret;
}

static esp_ble_scan_params_t hid_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
};

static esp_err_t start_ble_scan(uint32_t seconds) {
    esp_err_t ret = ESP_OK;
    if ((ret = esp_ble_gap_set_scan_params(&hid_scan_params)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_set_scan_params failed: %d", ret);
        return ret;
    }
    WAIT_BLE_CB();

    if ((ret = esp_ble_gap_start_scanning(seconds)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_start_scanning failed: %d", ret);
        return ret;
    }
    return ret;
}

esp_err_t esp_hid_ble_gap_adv_init(uint16_t appearance, const char* device_name) {

    esp_err_t ret;

    const uint8_t hidd_service_uuid128[] = {
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
    };

    esp_ble_adv_data_t ble_adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
        .appearance = appearance,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(hidd_service_uuid128),
        .p_service_uuid = (uint8_t*)hidd_service_uuid128,
        .flag = 0x6,
    };

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;//you have to enter the key on the host
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;//you have to enter the key on the device
    esp_ble_io_cap_t iocap = ESP_IO_CAP_IO;//you have to agree that key matches on both
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;//device is not capable of input or output, insecure
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t key_size = 16; //the key size should be 7~16 bytes
    uint32_t passkey = 1234;//ESP_IO_CAP_OUT

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param AUTHEN_REQ_MODE failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param IOCAP_MODE failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param SET_INIT_KEY failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param SET_RSP_KEY failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param MAX_KEY_SIZE failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t))) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param SET_STATIC_PASSKEY failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_set_device_name(device_name)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_device_name failed: %d", ret);
        return ret;
    }

    if ((ret = esp_ble_gap_config_adv_data(&ble_adv_data)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP config_adv_data failed: %d", ret);
        return ret;
    }

    return ret;
}

esp_err_t esp_hid_ble_gap_adv_start(void) {
    static esp_ble_adv_params_t hidd_adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x30,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    return esp_ble_gap_start_advertising(&hidd_adv_params);
}
#endif /* CONFIG_BT_BLE_ENABLED */

/*
 * CONTROLLER INIT
 * */

#if CONFIG_BT_NIMBLE_ENABLED
static esp_err_t init_low_level(uint8_t mode) {
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32
    bt_cfg.mode = mode;
#endif
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_mem_release failed: %d", ret);
        return ret;
    }
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %d", ret);
        return ret;
    }

    ret = esp_bt_controller_enable(mode);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %d", ret);
        return ret;
    }

    ret = esp_nimble_init();
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_init failed: %d", ret);
        return ret;
    }

    return ret;
}

static void handle_ble_device_result(const struct ble_gap_disc_desc* disc) {
    int rc;
    struct ble_hs_adv_fields fields;
    uint16_t appearance;
    uint8_t adv_name[BLE_HS_ADV_MAX_SZ];
    uint8_t adv_name_len = 0;

    appearance = 0; /* silent warnings for now */
    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        return;
    }

    if (fields.name != NULL) {
        assert(fields.name_len < sizeof adv_name - 1);
        memcpy(adv_name, fields.name, fields.name_len);
        adv_name[fields.name_len] = '\0';
        adv_name_len = fields.name_len;
        MODLOG_DFLT(DEBUG, "    name(%scomplete)=%s\n",
            fields.name_is_complete ? "" : "in", adv_name);
    }

    if (fields.appearance_is_present) {
        MODLOG_DFLT(DEBUG, "    appearance=0x%04x\n", fields.appearance);
        appearance = fields.appearance;
    }

    for (int i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == BLE_HID_SVC_UUID &&
            ((adv_name_len > 0 && memcmp("ESP BLE HID2", adv_name, adv_name_len) == 0) ||
                (adv_name_len > 0 && memcmp("ESP Mouse", adv_name, adv_name_len) == 0) ||
                (adv_name_len > 0 && memcmp("ESP Keyboard", adv_name, adv_name_len) == 0))) {
            add_ble_scan_result(disc->addr.val, disc->addr.type, appearance, adv_name, adv_name_len, disc->rssi);
            break;
        }
    }
}
#endif

#if CONFIG_BT_NIMBLE_ENABLED
static int
nimble_hid_gap_event(struct ble_gap_event* event, void* arg) {
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        handle_ble_device_result(&event->disc);
        if (rc != 0) {
            return 0;
        }

        /* An advertisement report was received during GAP discovery. */
        return 0;
        break;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        MODLOG_DFLT(INFO, "discovery complete; reason=%d\n",
            event->disc_complete.reason);
        SEND_BLE_CB();
        return 0;
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d",
            event->connect.status == 0 ? "established" : "failed",
            event->connect.status);
        return 0;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);

        return 0;
    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d",
            event->conn_update.status);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "advertise complete; reason=%d",
            event->adv_complete.reason);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d "
            "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
            event->subscribe.conn_handle,
            event->subscribe.attr_handle,
            event->subscribe.reason,
            event->subscribe.prev_notify,
            event->subscribe.cur_notify,
            event->subscribe.prev_indicate,
            event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d",
            event->mtu.conn_handle,
            event->mtu.channel_id,
            event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        MODLOG_DFLT(INFO, "encryption change event; status=%d ",
            event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        MODLOG_DFLT(INFO, "notify_tx event; conn_handle=%d attr_handle=%d "
            "status=%d is_indication=%d",
            event->notify_tx.conn_handle,
            event->notify_tx.attr_handle,
            event->notify_tx.status,
            event->notify_tx.indication);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

         /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(TAG, "PASSKEY_ACTION_EVENT started");
        struct ble_sm_io pkey = { 0 };
        int key = 0;

        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            pkey.action = event->passkey.params.action;
            pkey.passkey = 123456; // This is the passkey to be entered on peer
            ESP_LOGI(TAG, "Enter passkey %" PRIu32 "on the peer side", pkey.passkey);
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            ESP_LOGI(TAG, "Accepting passkey..");
            pkey.action = event->passkey.params.action;
            pkey.numcmp_accept = key;
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_OOB) {
            static uint8_t tem_oob[16] = { 0 };
            pkey.action = event->passkey.params.action;
            for (int i = 0; i < 16; i++) {
                pkey.oob[i] = tem_oob[i];
            }
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
            ESP_LOGI(TAG, "Input not supported passing -> 123456");
            pkey.action = event->passkey.params.action;
            pkey.passkey = 123456;
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
        }
        return 0;
    }
    return 0;
}

static esp_err_t start_nimble_scan(uint32_t seconds) {
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return rc;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform active scan.
     */
    disc_params.passive = 0;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0x50;
    disc_params.window = 0x30;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, seconds * 1000, &disc_params,
        nimble_hid_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
            rc);
    }
    return rc;
}

#else
static esp_err_t init_low_level(uint8_t mode) {
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32
    bt_cfg.mode = mode;
#endif
#if CONFIG_BT_HID_HOST_ENABLED
    if (mode & ESP_BT_MODE_CLASSIC_BT) {
        bt_cfg.bt_max_acl_conn = 3;
        bt_cfg.bt_max_sync_conn = 3;
    } else
#endif
    {
        ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (ret) {
            ESP_LOGE(TAG, "esp_bt_controller_mem_release failed: %d", ret);
            return ret;
        }
    }
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %d", ret);
        return ret;
    }

    ret = esp_bt_controller_enable(mode);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %d", ret);
        return ret;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %d", ret);
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %d", ret);
        return ret;
    }
#if CONFIG_BT_HID_HOST_ENABLED
    if (mode & ESP_BT_MODE_CLASSIC_BT) {
        ret = init_bt_gap();
        if (ret) {
            return ret;
        }
    }
#endif
#if CONFIG_BT_BLE_ENABLED
    if (mode & ESP_BT_MODE_BLE) {
        ret = init_ble_gap();
        if (ret) {
            return ret;
        }
    }
#endif /* CONFIG_BT_BLE_ENABLED */
    return ret;
}
#endif

esp_err_t esp_hid_gap_init(uint8_t mode) {
    esp_err_t ret;
    if (!mode || mode > ESP_BT_MODE_BTDM) {
        ESP_LOGE(TAG, "Invalid mode given!");
        return ESP_FAIL;
    }

    if (bt_hidh_cb_semaphore != NULL) {
        ESP_LOGE(TAG, "Already initialised");
        return ESP_FAIL;
    }

    bt_hidh_cb_semaphore = xSemaphoreCreateBinary();
    if (bt_hidh_cb_semaphore == NULL) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed!");
        return ESP_FAIL;
    }

    ble_hidh_cb_semaphore = xSemaphoreCreateBinary();
    if (ble_hidh_cb_semaphore == NULL) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed!");
        vSemaphoreDelete(bt_hidh_cb_semaphore);
        bt_hidh_cb_semaphore = NULL;
        return ESP_FAIL;
    }

    ret = init_low_level(mode);
    if (ret != ESP_OK) {
        vSemaphoreDelete(bt_hidh_cb_semaphore);
        bt_hidh_cb_semaphore = NULL;
        vSemaphoreDelete(ble_hidh_cb_semaphore);
        ble_hidh_cb_semaphore = NULL;
        return ret;
    }

    return ESP_OK;
}

esp_err_t esp_hid_scan(uint32_t seconds, size_t* num_results, esp_hid_scan_result_t** results) {
    if (num_bt_scan_results || bt_scan_results || num_ble_scan_results || ble_scan_results) {
        ESP_LOGE(TAG, "There are old scan results. Free them first!");
        return ESP_FAIL;
    }

#if CONFIG_BT_BLE_ENABLED
    if (start_ble_scan(seconds) == ESP_OK) {
        WAIT_BLE_CB();
    } else {
        return ESP_FAIL;
    }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_NIMBLE_ENABLED
    if (start_nimble_scan(seconds) == ESP_OK) {
        WAIT_BLE_CB();
    } else {
        return ESP_FAIL;
    }
#endif /* CONFIG_BT_BLE_ENABLED */


#if CONFIG_BT_HID_HOST_ENABLED
    if (start_bt_scan(seconds) == ESP_OK) {
        WAIT_BT_CB();
    } else {
        return ESP_FAIL;
    }
#endif

    * num_results = num_bt_scan_results + num_ble_scan_results;
    *results = bt_scan_results;
    if (num_bt_scan_results) {
        while (bt_scan_results->next != NULL) {
            bt_scan_results = bt_scan_results->next;
        }
        bt_scan_results->next = ble_scan_results;
    } else {
        *results = ble_scan_results;
    }

    num_bt_scan_results = 0;
    bt_scan_results = NULL;
    num_ble_scan_results = 0;
    ble_scan_results = NULL;
    return ESP_OK;
}
