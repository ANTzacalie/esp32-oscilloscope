#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "driver/uart.h" 

static constexpr int SAMPLE_COUNT = 256;
static constexpr int ADC_FREQ_HZ  = 1000000;

/*

    - up to 240khz in frequency
    - full voltage supported

*/

extern "C" void app_main(void) {
    
    uart_set_baudrate(UART_NUM_0, 115200);
    esp_log_level_set("*", ESP_LOG_NONE); // silence noise, keep only our printf

    adc_continuous_handle_t adc_handle = NULL;
    adc_continuous_handle_cfg_t init_cfg = {};
    init_cfg.max_store_buf_size  = 8192;
    init_cfg.conv_frame_size     = SAMPLE_COUNT * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&init_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern = {};
    pattern.atten     = ADC_ATTEN_DB_12;
    pattern.channel   = ADC_CHANNEL_7;
    pattern.unit      = ADC_UNIT_1;
    pattern.bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t cont_cfg = {};
    cont_cfg.sample_freq_hz = ADC_FREQ_HZ;
    cont_cfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
    cont_cfg.adc_pattern    = &pattern;
    cont_cfg.pattern_num    = 1;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &cont_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    uint32_t raw_rx_bytes = SAMPLE_COUNT * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
    uint8_t *raw_buf = (uint8_t *)malloc(raw_rx_bytes);
    int frame_count = 0;

    while (true) {

        uint32_t out_len = 0;
        esp_err_t ret = adc_continuous_read(
            adc_handle, 
            raw_buf, 
            raw_rx_bytes, 
            &out_len, pdMS_TO_TICKS(100)
        );

        if (ret == ESP_OK && out_len == raw_rx_bytes) {

            // -- Send frame header with frame number
            printf("FRAME:%d\n", frame_count++);

            // -- Send all x samples as ASCII integers, one per line
            for (int i = 0; i < SAMPLE_COUNT; i++) {

                adc_digi_output_data_t *d = (adc_digi_output_data_t *)&raw_buf[i * SOC_ADC_DIGI_DATA_BYTES_PER_CONV];
                printf("%u\n", (uint16_t)d->type1.data);

            }

            // -- Send frame footer so MATLAB knows the frame is complete
            printf("END\n");

        } else if (ret == ESP_ERR_TIMEOUT) {

            printf("TIMEOUT\n");

        } else {

            printf("ERR:0x%x\n", ret);

        }

    }

}