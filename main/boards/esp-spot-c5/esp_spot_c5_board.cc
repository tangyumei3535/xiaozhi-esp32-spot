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
    }

public:
    Led* GetLed() override;

    EspSpotC5Bot() : boot_button_(BOOT_BUTTON_GPIO), key_button_(KEY_BUTTON_GPIO) {
        InitializeI2c();
        InitializeButtons();
        InitializeIot();
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
                ESP_LOGI("LED", "LED Effect: OFF");
                led_indicator_set_on_off(led_handle, false);
                break;
            case kDeviceStateConnecting:
                ESP_LOGI("LED", "LED Effect: Triple Blink");
                led_indicator_start(led_handle, 0);
                break;
            case kDeviceStateListening:
                ESP_LOGI("LED", "LED Effect: Breathing Slow");
                led_indicator_start(led_handle, 1);
                break;
            case kDeviceStateSpeaking:
                ESP_LOGI("LED", "LED Effect: ON");
                led_indicator_set_on_off(led_handle, true);
                break;
            default:
                ESP_LOGI("LED", "LED Effect: OFF (default)");
                led_indicator_set_on_off(led_handle, false);
                break;
        }
    }
};

Led* EspSpotC5Bot::GetLed() {
    static BoardLed board_led;
    return &board_led;
}

// static BoardLed board_led;