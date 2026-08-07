#include <freertos/FreeRTOS.h>
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/dac_cosine.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include <atomic>
#include <string>


std::atomic<bool> button_pressed_left  = false;
std::atomic<bool> button_pressed_right = false;
std::atomic<bool> output_press         = false;
std::atomic<double> pot_freq           = 0;
std::atomic<bool> allow                = false;
static const char *TAG_S = "SENSOR_VAL";
static const char *TAG_G = "SIGNAL_GEN";
static const char *TAG_P = "POT_VAL";

/*

    ESP.LOGI IS DEBUG ONLY, FOR RELEASE THEY NEED TO BE REMOVED!

*/

void delay(int par) {

    vTaskDelay(pdMS_TO_TICKS(par));

}

enum signal_type {

    SINE = 0,
    SQUARE = 1

};

struct signal_data {

    std::atomic<int>    freq         = 1000; // in Hz
    std::atomic<float>  amplitude    = 0; // in mV
    std::atomic<int8_t> typ_selected = 0; // sine as default

} sd;

void button_left(void *p) {

    while(true) {

        bool current_state = gpio_get_level(GPIO_NUM_27) == 0;  // true when pressed

        if(output_press.load()) {

            if(current_state) {

                int freq = sd.freq.load();
                if(freq < 1500000)
                    sd.freq.store(freq + 1000);
                ESP_LOGI(TAG_S, "freq: %d", sd.freq.load());

            }

        } else {

            button_pressed_left.store(current_state);
            
        }

        delay(180); 

    }

}

void button_right(void *p) {

    while (true) {

        bool current_state = gpio_get_level(GPIO_NUM_32) == 0;

        if (output_press.load()) {

            if (current_state) {

                int freq = sd.freq.load();
                if (freq > 1000)
                    sd.freq.store(freq - 1000);
                ESP_LOGI(TAG_S, "freq: %d", sd.freq.load());

            }

        } else {

            button_pressed_right.store(current_state); 

        }

        delay(180);

    }

}

void output_button(void *p) {

    while(true) {

        if(gpio_get_level(GPIO_NUM_13) == 0) {

            output_press.store(true);
            ESP_LOGI(TAG_S, "Output: ON");

        } else {

            output_press.store(false); 
            ESP_LOGI(TAG_S, "Output: OFF");

        }

        delay(500);

    }

}

// using adc_oneshot as the potentimeter dose not require very fast or accurate reading, 
// continuous reading is unncessary, 100nF capacitor would be required and 
// even then not enough to stabilize a potentimeter reading;
void read_potentimeter(void *p) {

    // potentiometer is 10k
    // ADC SETUP FOR GPIO35 , PIN SETUP AND CALIBRATION;
    adc_oneshot_unit_handle_t adc_handle_io35;
    adc_cali_handle_t cali_handle_io35;

    adc_oneshot_unit_init_cfg_t init_cfg_io35 = {};
    init_cfg_io35.unit_id = ADC_UNIT_1;
    init_cfg_io35.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    init_cfg_io35.ulp_mode = ADC_ULP_MODE_DISABLE;
    adc_oneshot_new_unit(&init_cfg_io35, &adc_handle_io35);

    adc_oneshot_chan_cfg_t chan_cfg_io35 = {};
    chan_cfg_io35.atten = ADC_ATTEN_DB_12; // 0-3.3V
    chan_cfg_io35.bitwidth = ADC_BITWIDTH_12;
    adc_oneshot_config_channel(adc_handle_io35, ADC_CHANNEL_7, &chan_cfg_io35);

    adc_cali_line_fitting_config_t cali_cfg_io35 = {};
    cali_cfg_io35.unit_id = ADC_UNIT_1;
    cali_cfg_io35.atten = ADC_ATTEN_DB_12;
    cali_cfg_io35.bitwidth = ADC_BITWIDTH_12;
    cali_cfg_io35.default_vref = 1100; // fallback value , internal eFuse value will be taken if it exists(most ESP32 do have it);
    
    adc_cali_create_scheme_line_fitting(&cali_cfg_io35, &cali_handle_io35);
    // ADC SETUP FOR GPIO35

    int raw, voltage;
    float last_voltage = 0;
    float voltage_conv;

    while (true)
    {
        adc_oneshot_read(adc_handle_io35, ADC_CHANNEL_7, &raw);
        adc_cali_raw_to_voltage(cali_handle_io35, raw, &voltage);

        voltage_conv = voltage / 1000.0f;
        voltage_conv = ((int)(voltage_conv * 10)) / 10.0f;

        if (voltage_conv != last_voltage) {

            sd.amplitude.store(voltage_conv);
            last_voltage = voltage_conv;

            ESP_LOGI(TAG_P, "Value: %.2f", voltage_conv);
            
        }

        delay(350);
    }

}

// functions that has the task to generate signals based on user input
/*

    - GPIO26 DAC is signal output;
    - amplitude -> from 0.142V to 3.155V;
    - frequency -> from 50kHz to 1.5Mhz, with 100 Hz step;
    - types of signals: sine, square(only at 3.3V);
    - square signal works on ledc for the 50kHz to 1.5Mhz frequency;
    - esp32 analizer can only recive up to 1Mhz in frequency, so 1.5Mhz is over;

*/

static constexpr ledc_mode_t        kSpeedMode  = LEDC_LOW_SPEED_MODE;
static constexpr ledc_channel_t     kChannel    = LEDC_CHANNEL_0;
static constexpr ledc_timer_t       kTimer      = LEDC_TIMER_0;
static constexpr ledc_timer_bit_t   kResolution = LEDC_TIMER_2_BIT;
static constexpr int                kGpio       = 12;
static constexpr uint32_t           kDuty50     = (1u << 2) / 2;

static ledc_channel_config_t ledc_ch  = {};
static bool ledc_running              = false;

void square_stop(void) {
    if (!ledc_running) return;
    ledc_stop(kSpeedMode, kChannel, 0);
    ledc_running = false;
}

static void apply_timer(int32_t freq_Hz) {
    ledc_timer_config_t ledc_timer  = {};
    ledc_timer.speed_mode           = kSpeedMode;
    ledc_timer.timer_num            = kTimer;
    ledc_timer.duty_resolution      = kResolution;
    ledc_timer.freq_hz              = freq_Hz;
    ledc_timer.clk_cfg              = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

void square_start(int32_t freq_Hz) {
    apply_timer(freq_Hz);

    ledc_ch.speed_mode              = kSpeedMode;
    ledc_ch.channel                 = kChannel;
    ledc_ch.timer_sel               = kTimer;
    ledc_ch.hpoint                  = 0;
    ledc_ch.duty                    = kDuty50;
    ledc_ch.gpio_num                = kGpio;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch));

    ledc_set_duty(kSpeedMode, kChannel, kDuty50);
    ledc_update_duty(kSpeedMode, kChannel);

    ledc_running = true;
}

void square_set_freq(int32_t freq_Hz) {
    if (!ledc_running) {
        square_start(freq_Hz);
        return;
    }
    apply_timer(freq_Hz);
    ledc_set_duty(kSpeedMode, kChannel, kDuty50);
    ledc_update_duty(kSpeedMode, kChannel);
}

static dac_cosine_handle_t dac_handle = NULL;
void sine_init(int32_t freq_Hz, dac_cosine_atten_t ampl) {

    dac_cosine_config_t cfg  = {}; 
    cfg.chan_id              = DAC_CHAN_1;
    cfg.freq_hz              = freq_Hz;
    cfg.clk_src              = DAC_COSINE_CLK_SRC_DEFAULT;
    cfg.offset               = 0;
    cfg.phase                = DAC_COSINE_PHASE_0;
    cfg.atten                = ampl;
    cfg.flags.force_set_freq = true;

    // handles errors that can happen in the DAC driver
    ESP_ERROR_CHECK(dac_cosine_new_channel(&cfg, &dac_handle));
    ESP_ERROR_CHECK(dac_cosine_start(dac_handle));

}

dac_cosine_atten_t amplitude_to_atten(float ampl) {

    if      (ampl >= 1.7f)  return DAC_COSINE_ATTEN_DB_0;
    else if (ampl >= 0.9f)  return DAC_COSINE_ATTEN_DB_6;
    else if (ampl >= 0.45f) return DAC_COSINE_ATTEN_DB_12;
    else                    return DAC_COSINE_ATTEN_DB_18;

}

// call at start/frequency/amplitude changes — tears down and rebuilds;
void sine_set_freq(int32_t freq_Hz, dac_cosine_atten_t ampl) {

    if (dac_handle != NULL) {

        dac_cosine_stop(dac_handle);
        dac_cosine_del_channel(dac_handle);
        dac_handle = NULL;

    }

    sine_init(freq_Hz, ampl);

}

static std::atomic<bool> isOn = true;

void signal_loop(void *p) {
    
    int last_freq      = 0;
    float last_ampl    = 0.0f;

    while (true)
    {
        
        if(output_press.load()) {

            int set_freq   = sd.freq.load();
            float set_ampl = sd.amplitude.load();

            if(isOn.load() || last_freq != set_freq || last_ampl != set_ampl) {

                if(sd.typ_selected.load() == SINE) {

                    // square off;
                    square_stop();

                    // sine on;
                    dac_cosine_atten_t atten_ampl = amplitude_to_atten(set_ampl);
                    sine_set_freq(set_freq, atten_ampl);

                    last_freq = set_freq;
                    last_ampl = set_ampl;

                } else {

                    // sine off;
                    if (dac_handle != NULL) {
                        dac_cosine_stop(dac_handle);
                        dac_cosine_del_channel(dac_handle);
                        dac_handle = NULL;
                    }

                    // square on;
                    square_set_freq(set_freq);

                    last_freq = set_freq;

                } 

                isOn.store(false);
            }

        } else {
            
            square_stop();
            if (dac_handle != NULL) {

                dac_cosine_stop(dac_handle);
                dac_cosine_del_channel(dac_handle);
                dac_handle = NULL;

            }

            isOn.store(true);

        };
        
        delay(100);

    }

}

void sig_type_selector(void *p) {

    while (true) {

        if (!output_press.load()) {

            if (gpio_get_level(GPIO_NUM_32) == 0) {

               
                sd.typ_selected.store(0); 
                ESP_LOGI(TAG_S, "SIGNAL_changed = SINE"); 

            } else if (gpio_get_level(GPIO_NUM_27) == 0) {   

                sd.typ_selected.store(1); 
                ESP_LOGI(TAG_S, "SIGNAL_changed = SQUARE"); 

            }            

        }

        delay(130); 

    }

}

extern "C" void app_main(void) {

    ///////////////// --- SETUP --- /////////////////
    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_27, GPIO_PULLUP_ONLY);
    /////////////////////////////////////////////////
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_32, GPIO_PULLUP_ONLY);
    /////////////////////////////////////////////////
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);
    /////////////////////////////////////////////////
    ESP_LOGI(TAG_G, "INIT_0");
    delay(1500);
    ///////////////// --- SETUP --- /////////////////

    // tasks(thread style), usStackDepth -> size of memory alocated, pvParameter -> custom parameters for start,
    // priority(0=lowest, 25=highest), handle(specific action to delte/suspend), and there is alos XTaskCreatePinnedCore,
    // an additional parameter is given to select core for the thread to be connected(for ESP32 2xLuaX6, there is core 0 and 1)
    xTaskCreatePinnedToCore(button_left,             "BTN_left",   4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(button_right,            "BTN_right",  4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(output_button,           "BTN_out",    4096, NULL, 5,  NULL, 0);
    xTaskCreatePinnedToCore(read_potentimeter,       "POT_read",   8192, NULL, 15, NULL, 0);
    xTaskCreatePinnedToCore(sig_type_selector,       "SIG_SEL",    4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(signal_loop,             "SIG_LOOP",   8192, NULL, 20, NULL, 1);

}