#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <driver/gpio.h>
#include "led_indicator.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "esp_spot_c5"

static led_indicator_handle_t led_handle = nullptr;  // **全局 LED 句柄**

// 定义 LED 闪烁模式

static const blink_step_t triple_blink[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 500},
    {LED_BLINK_HOLD, LED_STATE_OFF, 500},
    {LED_BLINK_HOLD, LED_STATE_ON, 500},
    {LED_BLINK_HOLD, LED_STATE_OFF, 500},
    {LED_BLINK_HOLD, LED_STATE_ON, 500},
    {LED_BLINK_HOLD, LED_STATE_OFF, 500},
    {LED_BLINK_STOP, 0, 0},
};

static const blink_step_t breath_slow_blink[] = {
    {LED_BLINK_HOLD, LED_STATE_OFF, 0},
    {LED_BLINK_BREATHE, LED_STATE_ON, 1500},
    {LED_BLINK_BREATHE, LED_STATE_OFF, 1500},
    {LED_BLINK_LOOP, 0, 0},
};

blink_step_t const *led_modes[] = {
    [0] = triple_blink,
    [1] = breath_slow_blink,
};


static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle;
static bool do_calibration = false;

static void init_adc() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VBAT_ADC_CHANNEL, &chan_config));

    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
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
#endif
}

class SpotEs8311AudioCodec : public Es8311AudioCodec {
private:

public:
SpotEs8311AudioCodec(void* i2c_master_handle, i2c_port_t i2c_port, int input_sample_rate, int output_sample_rate,
                        gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din,
                        gpio_num_t pa_pin, uint8_t es8311_addr, bool use_mclk = true)
        : Es8311AudioCodec(i2c_master_handle, i2c_port, input_sample_rate, output_sample_rate,
                             mclk,  bclk,  ws,  dout,  din,pa_pin,  es8311_addr,  use_mclk = false) {}

    void EnableOutput(bool enable) override {
        if (enable == output_enabled_) {
            return;
        }
        Es8311AudioCodec::EnableOutput(enable);
    }
};


class EspSpotC5Bot : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Button key_button_;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }


    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                // ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
        key_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Battery"));
    }

    void InitializeGPIO() {
        gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << POWER_CTL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(POWER_CTL_PIN, 1);  // 拉高保持
    }

public:
    Led* GetLed() override;
    virtual bool GetBatteryLevel(int &level, bool &charging);

    EspSpotC5Bot() : boot_button_(BOOT_BUTTON_GPIO), key_button_(KEY_BUTTON_GPIO) {
        InitializeI2c();
        InitializeButtons();
        InitializeIot();
        InitializeGPIO();
    }

    virtual AudioCodec* GetAudioCodec() override {
         static SpotEs8311AudioCodec audio_codec(i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }
};

DECLARE_BOARD(EspSpotC5Bot);


class BoardLed : public Led {
public:
    BoardLed() {
        InitLed();
    }

    void OnStateChanged() override {
        ESP_LOGI("LED", "OnStateChanged triggered!");
        auto& app = Application::GetInstance();
        DeviceState state = app.GetDeviceState();
        SetState(state);
    }

private:
    void InitLed() {
        led_indicator_ledc_config_t ledc_config = {
            .is_active_level_high = false,
            .timer_inited = false,
            .timer_num = LEDC_TIMER_0,
            .gpio_num = LED_PIN,
            .channel = LEDC_CHANNEL_0,
        };

        led_indicator_config_t led_config = {
            .mode = LED_LEDC_MODE,
            .led_indicator_ledc_config = &ledc_config,
            .blink_lists = led_modes,
            .blink_list_num = 2,
        };

        led_handle = led_indicator_create(&led_config);
    }

    void SetState(DeviceState state) {
        for (int i = 0; i < 2; i++) {
            led_indicator_stop(led_handle, i);
        }

        switch (state) {
            case kDeviceStateIdle:
                // ESP_LOGI("LED", "LED Effect: OFF");
                led_indicator_set_on_off(led_handle, false);
                break;
            case kDeviceStateConnecting:
                // ESP_LOGI("LED", "LED Effect: Triple Blink");
                led_indicator_start(led_handle, 0);
                break;
            case kDeviceStateListening:
                // ESP_LOGI("LED", "LED Effect: Breathing Slow");
                led_indicator_start(led_handle, 1);
                break;
            case kDeviceStateSpeaking:
                // ESP_LOGI("LED", "LED Effect: ON");
                led_indicator_set_on_off(led_handle, true);
                break;
            default:
                // ESP_LOGI("LED", "LED Effect: OFF (default)");
                led_indicator_set_on_off(led_handle, false);
                break;
        }
    }
};

Led* EspSpotC5Bot::GetLed() {
    static BoardLed board_led;
    return &board_led;
}


bool EspSpotC5Bot::GetBatteryLevel(int &level, bool &charging) {
    if (!adc1_handle) {
        init_adc();
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

    charging = gpio_get_level(POWER_CTL_PIN);
    ESP_LOGI(TAG, "Battery Level: %d%%, Charging: %s", level, charging ? "Yes" : "No");
    return true;
}
