#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "motor.h"

#define TAG "SPP_SERVER"
Motor_Typedef motor_left;
Motor_Typedef motor_right;

static SemaphoreHandle_t cmd_mutex;
static char bt_command = 'S';
static int speed_level = 2;

int map_speed(int level)
{
    if (level < 1) level = 1;
    if (level > 9) level = 9;
    return 600 + level * 44;
}

void motor_task(void *arg)
{
    while (1)
    {
        xSemaphoreTake(cmd_mutex, portMAX_DELAY);
        char cmd = bt_command;
        int pwm = map_speed(speed_level);
        xSemaphoreGive(cmd_mutex);

        switch (cmd)
        {
        case 'F':
            motor_forward(&motor_left, pwm);
            motor_forward(&motor_right, pwm);
            break;
        case 'B':
            motor_backward(&motor_left, pwm);
            motor_backward(&motor_right, pwm);
            break;
        case 'L':
            motor_forward(&motor_left, pwm / 4);
            motor_forward(&motor_right, pwm);
            break;
        case 'R':
            motor_forward(&motor_left, pwm);
            motor_forward(&motor_right, pwm / 4);
            break;
        case 'S':                           
            motor_stop(&motor_left);
            motor_stop(&motor_right);
            break;
        default:
            break;
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP INIT");
        esp_bt_gap_set_device_name("ESP32_BT");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP SERVER");
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "Client Connected!...");
        break;

    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(TAG, "Received %d byte", param->data_ind.len);

        if (param->data_ind.len > 0)        
        {
            char d = param->data_ind.data[0];

            if (d >= '1' && d <= '9')
            {
                xSemaphoreTake(cmd_mutex, portMAX_DELAY); 
                speed_level = d - '0';
                xSemaphoreGive(cmd_mutex);
                ESP_LOGI(TAG, "Speed = %d", speed_level);
            }
            else
            {
                xSemaphoreTake(cmd_mutex, portMAX_DELAY);
                bt_command = d;
                xSemaphoreGive(cmd_mutex);
                ESP_LOGI(TAG, "Command = %c", d);
            }
        }
        break;

    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    motor_init(&motor_left, 17, 4, 16, LEDC_CHANNEL_0, LEDC_TIMER_0);
    motor_init(&motor_right, 19, 5, 18, LEDC_CHANNEL_1, LEDC_TIMER_1);
    cmd_mutex = xSemaphoreCreateMutex();

    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "BT CONTROLLER FAILED %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret)
    {
        ESP_LOGE(TAG, "BT CONTROLLER FAILED %s", esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(spp_callback)) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = false,
        .tx_buffer_size = 0,
    };

    ret = esp_spp_enhanced_init(&spp_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "SPP init failed: %s", esp_err_to_name(ret));
    }

    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL); 

    ESP_LOGI(TAG, "System Ready!");
}