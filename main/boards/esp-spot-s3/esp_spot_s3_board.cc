#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "sdkconfig.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_types.h>
#include <driver/spi_common.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "bmi270.h"
#include "i2c_bus.h"

#include <driver/gpio.h>
#include "esp_timer.h"
#include "led/circular_strip.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "esp_spot_s3"

#ifdef IMU_DEBUG
#include "array"
#include "string_view"
#endif // IMU_DEBUG

static bool button_released_ = false;
static bool shutdown_ready_ = false;
static esp_timer_handle_t shutdown_timer_;

class SpotEs8311AudioCodec : public Es8311AudioCodec {
private:

public:
SpotEs8311AudioCodec(i2c_bus_handle_t i2c_bus_handle, i2c_port_t i2c_port, int input_sample_rate, int output_sample_rate,
                        gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din,
                        gpio_num_t pa_pin, uint8_t es8311_addr, bool use_mclk = true)
        : Es8311AudioCodec(i2c_bus_get_internal_bus_handle(i2c_bus_handle), i2c_port, input_sample_rate, output_sample_rate,
                                mclk,  bclk,  ws,  dout,  din,pa_pin,  es8311_addr,  use_mclk = false) {}

    void EnableOutput(bool enable) override {
        if (enable == output_enabled_) {
            return;
        }
        Es8311AudioCodec::EnableOutput(enable);
    }
};

namespace Bmi270Imu {

static bmi270_handle_t bmi_handle_ = nullptr;
static struct bmi2_feat_sensor_data sens_data_ = { .type = BMI2_WRIST_GESTURE };
static TaskHandle_t imu_task_handle_ = nullptr;

static void IRAM_ATTR ImuIsrHandler(void* arg)
{
    vTaskNotifyGiveFromISR(imu_task_handle_, NULL);
}

esp_err_t Initialize(i2c_bus_handle_t i2c_bus, uint8_t addr = BMI270_I2C_ADDRESS) {
    bmi270_i2c_config_t i2c_bmi270_conf = {
        .i2c_handle = i2c_bus,
        .i2c_addr = addr,
    };

    int8_t rslt = bmi270_sensor_create(&i2c_bmi270_conf, &bmi_handle_);
    if (rslt != BMI2_OK || !bmi_handle_) {
        ESP_LOGE(TAG, "BMI270 create failed: %d", rslt);
        return ESP_FAIL;
    }

    uint8_t sens_list[] = {BMI2_ACCEL, BMI2_WRIST_GESTURE};
    rslt = bmi270_sensor_enable(sens_list, 2, bmi_handle_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Sensor enable failed: %d", rslt);
        return ESP_FAIL;
    }

    struct bmi2_sens_config config = {.type = BMI2_WRIST_GESTURE};
    rslt = bmi270_get_sensor_config(&config, 1, bmi_handle_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Get config failed: %d", rslt);
        return ESP_FAIL;
    }
    config.cfg.wrist_gest.wearable_arm = BMI2_ARM_LEFT;
    rslt = bmi270_set_sensor_config(&config, 1, bmi_handle_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Set config failed: %d", rslt);
        return ESP_FAIL;
    }

    struct bmi2_sens_int_config sens_int = {
        .type = BMI2_WRIST_GESTURE,
        .hw_int_pin = BMI2_INT1
    };
    rslt = bmi270_map_feat_int(&sens_int, 1, bmi_handle_);

    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Map interrupt failed: %d", rslt);
        return ESP_FAIL;
    }

    struct bmi2_int_pin_config pin_config = { 0 };
    rslt = bmi2_get_int_pin_config(&pin_config, bmi_handle_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Get int pin config failed: %d", rslt);
        return ESP_FAIL;
    }

    pin_config.pin_type = BMI2_INT1;
    pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
    pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pin_config.int_latch = BMI2_INT_NON_LATCH;

    rslt = bmi2_set_int_pin_config(&pin_config, bmi_handle_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "Set int pin config failed: %d", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Gesture detection initialized");
    return ESP_OK;
}

bmi270_handle_t GetBmiHandle() {
    if(!bmi_handle_) {
        ESP_LOGE(TAG, "BMI Sensor has not been inited.");
    }
    return bmi_handle_;
}

uint8_t GetBmiStatus() {
    if(!bmi_handle_) {
        return 0;
        ESP_LOGW(TAG, "IMU is not inited!");
    }
    uint16_t status = 0;
    bmi2_get_int_status(&status, bmi_handle_);
    if (status & BMI270_WRIST_GEST_STATUS_MASK) {
        bmi270_get_feature_data(&sens_data_, 1, bmi_handle_);
        uint8_t gesture = sens_data_.sens_data.wrist_gesture_output;
#ifdef IMU_DEBUG
        constexpr std::array<std::string_view, 6> GUSTURE_LIST = {
            "unknown", "push_arm_down", "pivot_up",
            "wrist_shake_jiggle", "flick_in", "flick_out"
        };
        ESP_LOGI(TAG, "Detected gesture: %s", GUSTURE_LIST[gesture]);
#endif // IMU_DEBUG
        return gesture;
    }
    return 0;
}

void ImuTask(void *arg) {
    ESP_LOGI(TAG, "ImuTask Started!");
    uint16_t int_status = 0;
    vTaskDelay(pdMS_TO_TICKS(1000));
    while(true) {
        if (!bmi_handle_) {
            ESP_LOGW(TAG, "bmi_handle_ is NULL");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "Waiting for IMU interrupt...");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "IMU interrupt received!");
        if (GetBmiStatus() == 3) {
            auto& app = Application::GetInstance();
            auto device_state = app.GetDeviceState();
            if (device_state == kDeviceStateListening) {
                app.StopListening();
            } else if (device_state == kDeviceStateSpeaking) {
                app.AbortSpeaking(kAbortReasonNone);
            }
        }
    }
    ESP_LOGE(TAG, "ImuTask Stoped!");
    vTaskDelete(NULL);
}

}

class EspSpotS3Bot : public WifiBoard {
private:
    i2c_bus_handle_t i2c_bus_ = nullptr;
    // 为了兼容 BMI270 IMU 驱动，这里使用 i2c_bus 组件的 i2c_bus_handle，
    // 可以使用 `i2c_bus_get_internal_bus_handle(i2c_bus_)`
    // 获取 i2c_master_bus_handle_t 的 i2c_bus
    Button boot_button_;
    Button key_button_;
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_handle;
    bool do_calibration = false;
    bool key_long_pressed = false;
    int64_t last_key_press_time = 0;
    static const int64_t LONG_PRESS_TIMEOUT_US = 5 * 1000000ULL;

    void InitializeI2c() {
        const i2c_config_t i2c_bus_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                .clk_speed = I2C_MASTER_FREQ_HZ
            }
        };
        i2c_bus_ = i2c_bus_create(I2C_NUM_0, &i2c_bus_conf);
        if (!i2c_bus_) {
            ESP_LOGE(TAG, "I2C bus create failed");
            abort();
        }
    }

    void InitializeADC() {
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

        adc_oneshot_chan_cfg_t chan_config = {
            .atten = ADC_ATTEN,
            .bitwidth = ADC_WIDTH,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VBAT_ADC_CHANNEL, &chan_config));

#if CONFIG_ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_handle_t handle = NULL;
        esp_err_t ret = ESP_FAIL;
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN,
            .bitwidth = ADC_WIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            do_calibration = true;
            adc1_cali_handle = handle;
            ESP_LOGI(TAG, "ADC Curve Fitting calibration succeeded");
        }
#endif // CONFIG_ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            // auto& app = Application::GetInstance();
            ResetWifiConfiguration();
        });

        key_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            app.ToggleChatState();
            key_long_pressed = false;
        });

        key_button_.OnLongPress([this]() {
            int64_t now = esp_timer_get_time();
            auto* led = static_cast<CircularStrip*>(this->GetLed());

            if (key_long_pressed) {
                if ((now - last_key_press_time) < LONG_PRESS_TIMEOUT_US) {
                    ESP_LOGW(TAG, "Key button long pressed the second time within 5s, shutting down...");
                    led->SetSingleColor(0, {0, 0, 0});

                    gpio_hold_dis(MCU_VCC_CTL);
                    gpio_set_level(MCU_VCC_CTL, 0);

                } else {
                    last_key_press_time = now;
                    BlinkGreenFor5s();
                }
                key_long_pressed = true;
            } else {
                ESP_LOGW(TAG, "Key button first long press! Waiting second within 5s to shutdown...");
                last_key_press_time = now;
                key_long_pressed = true;

                BlinkGreenFor5s();
            }
        });
    }

    void InitializePowerCtl() {
        InitializeGPIO();

        gpio_set_level(MCU_VCC_CTL, 1);
        gpio_hold_en(MCU_VCC_CTL);

        gpio_set_level(PERP_VCC_CTL, 1);
        gpio_hold_en(PERP_VCC_CTL);
    }

    void InitializeGPIO() {
        gpio_config_t io_conf_1 = {
            .pin_bit_mask = (1ULL << MCU_VCC_CTL),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf_1);

        gpio_config_t io_conf_2 = {
            .pin_bit_mask = (1ULL << PERP_VCC_CTL),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf_2);

        gpio_config_t io_conf_imu_int = {
            .pin_bit_mask = (1ULL << IMU_INT_GPIO),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        gpio_config(&io_conf_imu_int);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(IMU_INT_GPIO, Bmi270Imu::ImuIsrHandler, nullptr);
    }

    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Battery"));
    }

    void BlinkGreenFor5s() {
        auto* led = static_cast<CircularStrip*>(GetLed());
        if (!led) {
            return;
        }

        led->Blink({50, 25, 0}, 100);

        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto* self = static_cast<EspSpotS3Bot*>(arg);
                auto* led = static_cast<CircularStrip*>(self->GetLed());
                if (led) {
                    led->SetSingleColor(0, {0, 0, 0});
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "blinkGreenFor5s_timer"
        };

        esp_timer_handle_t blink_timer = nullptr;
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &blink_timer));
        ESP_ERROR_CHECK(esp_timer_start_once(blink_timer, LONG_PRESS_TIMEOUT_US));
    }

    void StartImuTask() {
        int ret = xTaskCreate(Bmi270Imu::ImuTask, "imu_task", 3096, nullptr, 1, &Bmi270Imu::imu_task_handle_);
    }

public:
    EspSpotS3Bot() : boot_button_(BOOT_BUTTON_GPIO), key_button_(KEY_BUTTON_GPIO, true) {
        InitializePowerCtl();
        InitializeI2c();
        InitializeADC();
        StartImuTask();
        Bmi270Imu::Initialize(i2c_bus_);
        InitializeButtons();
        InitializeIot();
    }

    virtual Led* GetLed() override {
        static CircularStrip led(LED_PIN, 1);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
         static SpotEs8311AudioCodec audio_codec(i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int &level, bool &charging, bool &discharging) {
        if (!adc1_handle) {
            InitializeADC();
        }

        int raw_value = 0;
        int voltage = 0;

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VBAT_ADC_CHANNEL, &raw_value));

        if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, raw_value, &voltage));
            voltage = voltage * 3 / 2; // compensate for voltage divider
            ESP_LOGI(TAG, "Calibrated voltage: %d mV", voltage);
        } else {
            ESP_LOGI(TAG, "Raw ADC value: %d", raw_value);
            voltage = raw_value;
        }

        voltage = voltage < EMPTY_BATTERY_VOLTAGE ? EMPTY_BATTERY_VOLTAGE : voltage;
        voltage = voltage > FULL_BATTERY_VOLTAGE ? FULL_BATTERY_VOLTAGE : voltage;

        // 计算电量百分比
        level = (voltage - EMPTY_BATTERY_VOLTAGE) * 100 / (FULL_BATTERY_VOLTAGE - EMPTY_BATTERY_VOLTAGE);

        charging = gpio_get_level(MCU_VCC_CTL);
        ESP_LOGI(TAG, "Battery Level: %d%%, Charging: %s", level, charging ? "Yes" : "No");
        return true;
    }
};

DECLARE_BOARD(EspSpotS3Bot);
