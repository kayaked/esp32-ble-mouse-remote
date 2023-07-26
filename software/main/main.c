/*
Marty Kahuna Tech (MKT) C1
Software Version: PRE-RELEASE

Copyright Jack Engle (kayaked) 2023.
Licensed under the MIT Freeware License

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Date: 7/25/23
 */

// FreeRTOS, ESP General, C general
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

// Flash Memory
#include "nvs_flash.h"

// BlueTooth/LE, GATT, HIDD
#include "esp_bt.h"
#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "hid_dev.h"

// GPIO, ADC
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define HID_TAG "C1_LOG"


static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME "MKTech C1 Mouse"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // Shortened UUID: [12],[13]
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x63, 0x37, 0x00, 0x00,
};

#define GPIO_INPUT_IO_0     04
#define GPIO_INPUT_IO_1     02
#define GPIO_INPUT_IO_2     15
#define GPIO_INPUT_IO_3     32
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3))

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x03c0,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_TAG, param->led_write.data, param->led_write.length);
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

static QueueHandle_t gpio_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void hid_input_task(void *pvParameters)
{
    // ADC Initialization
    adc_oneshot_unit_handle_t handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &handle);

    // ADC Configuration
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(handle, ADC_CHANNEL_6, &config);
    adc_oneshot_config_channel(handle, ADC_CHANNEL_7, &config);

    // GPIO Configuration (TODO..)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
    gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_isr_handler, (void*) GPIO_INPUT_IO_3);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // btn is the mouse button's identifier
    // mouseXY holds raw values from ADC OneShot
    // x and y are the offsets sent to move the mouse by during this cycle
    // packet is true when a mouse update is being sent, and false for no changes
    /*
    Swstate 0:
    - GPIO 4: Left Click
    - GPIO 2: Right Click
    - GPIO 15: Middle Click
    - ADC6/7: Mouse Movement
    Swstate 1:
    - GPIO 4: Rewind
    - GPIO 2: Fast-Forward
    - GPIO 15: Pause/Play
    - ADC6: N/A
    - ADC7: Volume Up/Down
    */
    // 
    int mouseXY[2];
    bool active[3];
    int btn = 0, x, y;
    bool packet, swactive = 0, swstate = 0;
    while(1) {
        // ADC Readings [JoystickX, JoystickY]
        adc_oneshot_read(handle, ADC_CHANNEL_6, &mouseXY[0]);
        adc_oneshot_read(handle, ADC_CHANNEL_7, &mouseXY[1]);
        x = 0;
        y = 0;
        packet = false;

        if(!swstate) {
            if(mouseXY[0] > 1900) {
                x = mouseXY[0]/1000;
                packet = true;
            } else if(mouseXY[0] < 1250) {
                x = ((3100 - mouseXY[0]) * -1)/1000;
                packet = true;
            } else {
                x = 0;
            }
        }

        // Mouse Y Movement Detection
        if(!swstate) {
            if(mouseXY[1] > 1900) {
                y = (mouseXY[1]*-1)/1000;
                packet = true;
            } else if(mouseXY[1] < 1250) {
                y = ((3100 - mouseXY[1]))/1000;
                packet = true;
            } else {
                y = 0;
            }
        } else {
            if(mouseXY[1] > 2100) {
                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
            } else if(mouseXY[1] < 1050) {
                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
                packet = true;
            } else {
                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
                esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
            }
        }

        // Left Click
        if(gpio_get_level(GPIO_INPUT_IO_0) == 1) {
            if(active[0] == false) {
                packet = true;
                btn = 0x001;
                active[0] = true;
                if(swstate && sec_conn) {
                    uint8_t keyLeft = {HID_KEY_LEFT_ARROW};
                    esp_hidd_send_keyboard_value(hid_conn_id, 0, &keyLeft, 1);
                    esp_hidd_send_keyboard_value(hid_conn_id, 0, &keyLeft, 0);
                }
            }
        } else {
            if(active[0] == true) {
                packet = true;
                btn = 0x000;
                active[0] = false;
            }
        }

        // Right Click
        if(gpio_get_level(GPIO_INPUT_IO_1) == 1) {
            if(active[1] == false) {
                packet = true;
                btn = 0x002;
                active[1] = true;
                if(swstate && sec_conn) {
                    uint8_t keyRight = {HID_KEY_RIGHT_ARROW};
                    esp_hidd_send_keyboard_value(hid_conn_id, 0, &keyRight, 1);
                    esp_hidd_send_keyboard_value(hid_conn_id, 0, &keyRight, 0);
                }
            }
        } else {
            if(active[1] == true) {
                packet = true;
                btn = 0x000;
                active[1] = false;
            }
        }

        // Middle Click
        if(gpio_get_level(GPIO_INPUT_IO_2) == 1) {
            if(active[2] == false) {
                packet = true;
                btn = 0x004;
                active[2] = true;
                if(swstate && sec_conn) {
                    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_PAUSE, true);
                    esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_PAUSE, false);
                    //ESP_LOGI("DEBUG", "Play/Pause successfully sent.");
                }
            }
        } else {
            if(active[2] == true) {
                packet = true;
                btn = 0x000;
                active[2] = false;
            }
        }

        if(gpio_get_level(GPIO_INPUT_IO_3) == 0) { // Stick switch is pulled up and active low
            if(swactive == false) {
                if(swstate == false) {
                    swstate = true;

                    // When remote mode is activated, release every mouse button during this cycle
                    // This also resets the variables (mode agnostic) for use with remote mode
                    btn = 0x000;
                    x = 0;
                    y = 0;
                    if(sec_conn) esp_hidd_send_mouse_value(hid_conn_id, btn, x, y);
                } else {
                    swstate = false;

                    // When mouse mode is activated, disable every consumer signal
                    if(sec_conn) {
                        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
                        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
                        esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_PLAY_PAUSE, false);
                    }
                }

                swactive = true;
            }
        } else {
            if(swactive == true) swactive = false;
        }

        if (sec_conn && packet) {
            /*ESP_LOGI("DEBUG", "X: %d | Y: %d | btn: %d", x, y, btn);
            ESP_LOGI("DEBUG", "Send mouse switch");*/
            if(!swstate) {
                ESP_LOGI("DEBUG", "Test");
                esp_hidd_send_mouse_value(hid_conn_id, btn, x, y);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS Flash
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_TAG, "%s enable failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_TAG, "%s init failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_TAG, "%s init failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_TAG, "%s init failed\n", __func__);
    }

    // Register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    xTaskCreate(&hid_input_task, "C1PTask", 2048, NULL, 5, NULL);
}
