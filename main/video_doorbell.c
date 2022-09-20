/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Agora Lab, Inc (http://www.agora.io/)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <string.h>

#include "app_config.h"
#include "agora_iot_api.h"
#include "agora_iot_call.h"
#include "agora_iot_device_manager.h"
#include "algorithm_stream.h"
#include "audio_element.h"
#include "audio_hal.h"
#include "audio_mem.h"
#include "audio_pipeline.h"
#include "audio_sys.h"
#include "es7210.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_jpeg_enc.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "filter_resample.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "i2s_stream.h"
#include "input_key_service.h"
#include "nvs_flash.h"
#include "raw_stream.h"
#include "esp32s3/clk.h"
#include "esp_sleep.h"
#include "audio_thread.h"

#define DEFAULT_LISTEN_INTERVAL CONFIG_EXAMPLE_WIFI_LISTEN_INTERVAL

#if CONFIG_EXAMPLE_POWER_SAVE_MIN_MODEM
#define DEFAULT_PS_MODE WIFI_PS_MIN_MODEM
#elif CONFIG_EXAMPLE_POWER_SAVE_MAX_MODEM
#define DEFAULT_PS_MODE WIFI_PS_MAX_MODEM
#elif CONFIG_EXAMPLE_POWER_SAVE_NONE
#define DEFAULT_PS_MODE WIFI_PS_NONE
#else
#define DEFAULT_PS_MODE WIFI_PS_NONE
#endif

#define CAMERA_WIDTH (CONFIG_FRAME_WIDTH)
#define CAMERA_HIGH (CONFIG_FRAME_HIGH)

#define CAM_PIN_PWDN -1 // power down is not used
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK GPIO_NUM_40
#define CAM_PIN_SIOD GPIO_NUM_17
#define CAM_PIN_SIOC GPIO_NUM_18

#define CAM_PIN_D7 GPIO_NUM_39
#define CAM_PIN_D6 GPIO_NUM_41
#define CAM_PIN_D5 GPIO_NUM_42
#define CAM_PIN_D4 GPIO_NUM_12
#define CAM_PIN_D3 GPIO_NUM_3
#define CAM_PIN_D2 GPIO_NUM_14
#define CAM_PIN_D1 GPIO_NUM_47
#define CAM_PIN_D0 GPIO_NUM_13
#define CAM_PIN_VSYNC GPIO_NUM_21
#define CAM_PIN_HREF GPIO_NUM_38
#define CAM_PIN_PCLK GPIO_NUM_11

#define PRIO_TASK_FETCH (21)

#ifdef CONFIG_AUDIO_SAMPLE_RATE_8K
#define I2S_SAMPLE_RATE 8000
#define DEFAULT_PCM_CAPTURE_LEN (320)
#else
#define I2S_SAMPLE_RATE 16000
#define DEFAULT_PCM_CAPTURE_LEN (640)
#endif
#define I2S_CHANNELS 1
#define I2S_BITS 16

#define ESP_READ_BUFFER_SIZE 320

#define DEFAULT_MAX_BITRATE (2000000)

typedef struct {
    bool b_wifi_connected;
    bool b_call_session_started;
    bool b_exit;
} app_t;

static const char *TAG = "Agora";
static const char *certificate_for_test = "eyJzaWduIjoiZkppajRhWVdYeWUvU3I1MnlsRlk3NjRHSUN3UFpPenlnNzRHR2cyWSs2d"
                                          "XFyTkFOc0hPbmJaSWpFM2dxV0Y2dlpTKzhWbmhUU25MMUx0dkFjRXBqUnVaUzBYV2U4bj"
                                          "ZiWFA1Y3AzU1ljV1g4eVdiNFVPcktGU3VmL2MyME1BVlhINzM0OVdqYVprc3ZmeHgrWGV"
                                          "2b1dpb2hZNnF2dWRrZk9pTE9UaU0vWnFFZERTSVZ0RE40ZklFdmpTTmNYSU5GTG1XRFdt"
                                          "eExFcTRObmNNcm1STWR3U1RDRDZXamZqMVErZ0x2VTRrdllVWTJPd3plNGp4TitVQ1ZhY"
                                          "VRsenBNTm9kN0doUVFPcmRGUVk5dDc1SHFLT3ZFelpzM0VaK0VPSTJ4NXNhQmM1S3dJYX"
                                          "ZQMDlNMWxTUzRXWG8xcmE3UTQzbkthRWJRMmkreUR1ZHdXSzZKcWVnPT0iLCJjdXN0b20"
                                          "iOiIyMDIyLTA0LTE5VDIyOjQ2OjA3KzA4OjAwIiwiY3JlZGVudGlhbCI6IjRhNWZhMmVl"
                                          "ZGRkMmIyYzljN2M0MTk5YWFkNDgyMDViOWIwYzliYjE5ZWFlZmRlYTQ5OTZhYTk5ZjZkZ"
                                          "DkyNTIiLCJkdWUiOiIyMDIyMDcxOCJ9";

static app_t g_app = {
    .b_call_session_started = false,
    .b_wifi_connected       = false,
    .b_exit                 = false,
};

static camera_config_t camera_config = {
    .pin_pwdn     = CAM_PIN_PWDN,
    .pin_reset    = CAM_PIN_RESET,
    .pin_xclk     = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href  = CAM_PIN_HREF,
    .pin_pclk  = CAM_PIN_PCLK,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_YUV422, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size   = CONFIG_FRAME_SIZE, // QQVGA-UXGA Do not use sizes above QVGA
    // when not JPEG

    .jpeg_quality = 12, // 0-63 lower number means higher quality
    .fb_count     = 2, // if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    .conv_mode    = YUV422_TO_YUV420,
};

static char g_device_id[32] = { 0 };

static agora_iot_handle_t g_handle = NULL;

static uint32_t image_cnt = 0;
static uint32_t tick_begin = 0;

static audio_element_handle_t raw_read, raw_write, element_algo;
static audio_pipeline_handle_t recorder, player;
static SemaphoreHandle_t g_video_capture_sem  = NULL;
static SemaphoreHandle_t g_audio_capture_sem  = NULL;
static uint8_t g_push_type = 0x00;

static esp_lcd_panel_handle_t g_panel_handle;

static void init_lcd_service(esp_periph_set_handle_t set)
{
  g_panel_handle = audio_board_lcd_init(set, NULL);//lcd_trans_done_cb
}

static esp_err_t es7210_write_reg(i2c_bus_handle_t i2c_handle, uint8_t reg_addr, uint8_t data)
{
    return i2c_bus_write_bytes(i2c_handle, ES7210_AD1_AD0_00, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
}

static void set_es7210_tdm_mode()
{
    i2c_config_t es_i2c_cfg;
    i2c_bus_handle_t i2c_handle = i2c_bus_create(I2C_NUM_0, &es_i2c_cfg);
    es7210_write_reg(i2c_handle, 0x01, 0x20);
    es7210_write_reg(i2c_handle, 0x03, 0x06);
    es7210_write_reg(i2c_handle, 0x04, 0x03);
    es7210_write_reg(i2c_handle, 0x06, 0x04);
    es7210_write_reg(i2c_handle, 0x08, 0x14);
    es7210_write_reg(i2c_handle, 0x0b, 0x01);
    es7210_write_reg(i2c_handle, 0x11, 0x60);
    es7210_write_reg(i2c_handle, 0x12, 0x02);
    es7210_write_reg(i2c_handle, 0x3f, 0x01);
    es7210_write_reg(i2c_handle, 0x43, 0x1e);
    es7210_write_reg(i2c_handle, 0x44, 0x1e);
    es7210_write_reg(i2c_handle, 0x45, 0x18);
    es7210_write_reg(i2c_handle, 0x46, 0x1e);
    es7210_write_reg(i2c_handle, 0x47, 0x08);
    es7210_write_reg(i2c_handle, 0x49, 0x08);
    es7210_write_reg(i2c_handle, 0x4a, 0x08);
}

static void fps_timer_callback(void *arg)
{
    uint32_t cur_tick = xTaskGetTickCount();
    uint32_t duration = cur_tick - tick_begin;
    ESP_LOGW(TAG, "duration %-15u, image cnt %-10u, fps %f", duration, image_cnt,
             (float)image_cnt / (float)(duration / CONFIG_FREERTOS_HZ));

    audio_sys_get_real_time_stats();
    ESP_LOGI(TAG, "MEM Total:%d Bytes, Inter:%d Bytes, Dram:%d Bytes", esp_get_free_heap_size(),
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

static char *get_device_id(void)
{
  uint8_t base_mac_addr[6] = {0};

  ESP_ERROR_CHECK(esp_efuse_mac_get_default(base_mac_addr));

  uint8_t tmp = 0x00;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 2; j++) {
      tmp = (*(base_mac_addr + i) >> 4) * (1 - j) + (*(base_mac_addr + i) & 0x0F) * j;
      if (tmp >= 0 && tmp <= 9) {
        g_device_id[2 * i + j] = tmp + '0';
      } else if (tmp >= 0x0A && tmp <= 0x0F) {
        g_device_id[2 * i + j] = tmp - 0x0A + 'A';
      }
    }
  }

  return g_device_id;
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK) {
        switch ((int)evt->data) {
        case INPUT_KEY_USER_ID_REC: {
            char peer_id[64] = { 0 };
            snprintf(peer_id, sizeof(peer_id), "100000000000000000-%s", CONFIG_USER_ID);
            ESP_LOGW(TAG, "Now ring the bell and call user %s ...\n", peer_id);
            agora_iot_call(g_handle, peer_id, "I'm the guest");
        } break;
        case INPUT_KEY_USER_ID_VOLUP:
            ESP_LOGW(TAG, "Hang up the call.");
            agora_iot_hang_up(g_handle);
            g_app.b_call_session_started = false;
            break;
        case INPUT_KEY_USER_ID_VOLDOWN:
            ESP_LOGW(TAG, "Now exiting the app...");
            g_app.b_exit = true;
            break;
        default:
            ESP_LOGE(TAG, "User Key ID[%d] does not support", (int)evt->data);
            break;
        }
    }

    return ESP_OK;
}

static void start_key_service(esp_periph_set_handle_t set)
{
    ESP_LOGI(TAG, "Initialize Button peripheral with board init");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle                  = set;
    input_cfg.based_cfg.task_stack    = 5 * 1024;
    input_cfg.based_cfg.extern_stack  = true;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);

    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, NULL);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        g_app.b_wifi_connected = true;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/*init wifi as sta and set power save mode*/
static void setup_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
      .sta =
          {
              .ssid            = CONFIG_WIFI_SSID,
              .password        = CONFIG_WIFI_PASSWORD,
              .listen_interval = DEFAULT_LISTEN_INTERVAL,
          },
  };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "esp_wifi_set_ps().");
#ifdef CONFIG_ENABLE_LIGHT_SLEEP
    esp_wifi_set_ps(DEFAULT_PS_MODE);
#else
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif
}

static esp_err_t i2s_data_divided(int16_t *raw_buff, int len, int16_t *buf_aec)
{
    for (int i = 0; i < len / 4; i++) {
        buf_aec[i << 1] = raw_buff[(i << 1) + 1];
        buf_aec[(i << 1) + 1] = raw_buff[i << 1];
    }
    return ESP_OK;
}

int i2s_stream_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx)
{
    size_t bytes_read = 0;

    char *buf_tmp = audio_calloc(1, ESP_READ_BUFFER_SIZE);
    AUDIO_MEM_CHECK(TAG, buf, return 0);
    char *buf_aec = audio_calloc(1, ESP_READ_BUFFER_SIZE);
    AUDIO_MEM_CHECK(TAG, buf_aec, return 0);

    i2s_read(0, buf_tmp, ESP_READ_BUFFER_SIZE, &bytes_read, wait_time);
    if (bytes_read == ESP_READ_BUFFER_SIZE) {
        i2s_data_divided((int16_t *)buf_tmp, ESP_READ_BUFFER_SIZE, (int16_t *)buf_aec);
        memcpy(buf, buf_aec, bytes_read);
    }

    free(buf_tmp);
    free(buf_aec);

    return bytes_read;
}

static esp_err_t recorder_pipeline_open()
{
    audio_element_handle_t i2s_stream_reader;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    recorder = audio_pipeline_init(&pipeline_cfg);
    AUDIO_NULL_CHECK(TAG, recorder, return ESP_FAIL);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type             = AUDIO_STREAM_READER;
    i2s_cfg.uninstall_drv    = false;
#ifdef CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    i2s_cfg.i2s_port  = 1;
#endif
    // i2s_cfg.task_core = 1;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.i2s_config.sample_rate    = I2S_SAMPLE_RATE;
    i2s_cfg.i2s_config.mode           = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
    i2s_cfg.i2s_config.bits_per_sample = 32;
#endif
    i2s_cfg.out_rb_size  = 2 * 1024;
    i2s_cfg.stack_in_ext = true;
    i2s_stream_reader    = i2s_stream_init(&i2s_cfg);
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
    audio_element_set_read_cb(i2s_stream_reader, i2s_stream_read_cb, NULL);
#endif

    algorithm_stream_cfg_t algo_config = ALGORITHM_STREAM_CFG_DEFAULT();
#if defined(CONFIG_ESP_LYRAT_MINI_V1_1_BOARD) || defined(CONFIG_ESP32_S3_KORVO2_V3_BOARD)
    algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE1;
#else
    algo_config.input_type = ALGORITHM_STREAM_INPUT_TYPE2;
#endif
    // algo_config.task_core = 1;
    algo_config.task_stack = 4 * 1024;
    algo_config.algo_mask = ALGORITHM_STREAM_USE_AEC | ALGORITHM_STREAM_USE_NS;
    algo_config.stack_in_ext = true;

    element_algo = algo_stream_init(&algo_config);
    audio_element_set_music_info(element_algo, I2S_SAMPLE_RATE, 1, I2S_BITS);

    audio_pipeline_register(recorder, i2s_stream_reader, "i2s");
    audio_pipeline_register(recorder, element_algo, "algo");

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_READER;
    raw_cfg.out_rb_size = 2 * 1024;
    raw_read = raw_stream_init(&raw_cfg);
    audio_element_set_output_timeout(raw_read, portMAX_DELAY);

    audio_pipeline_register(recorder, raw_read, "raw");

    const char *link_tag[3] = { "i2s", "algo", "raw" };
    audio_pipeline_link(recorder, &link_tag[0], 3);

    ESP_LOGI(TAG, "audio recorder has been created");
    return ESP_OK;
}

static esp_err_t player_pipeline_open()
{
    audio_element_handle_t i2s_stream_writer;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    player = audio_pipeline_init(&pipeline_cfg);
    AUDIO_NULL_CHECK(TAG, player, return ESP_FAIL);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type        = AUDIO_STREAM_WRITER;
    raw_cfg.out_rb_size = 8 * 1024;
    raw_write = raw_stream_init(&raw_cfg);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type                      = AUDIO_STREAM_WRITER;
    i2s_cfg.uninstall_drv             = false;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.i2s_config.mode           = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    i2s_cfg.i2s_config.sample_rate    = I2S_SAMPLE_RATE;
#ifdef CONFIG_ESP32_S3_KORVO2_V3_BOARD
    i2s_cfg.i2s_config.bits_per_sample = 32;
    i2s_cfg.need_expand = true;
#endif
#if !defined(CONFIG_ESP_LYRAT_MINI_V1_1_BOARD) && !defined(CONFIG_ESP32_S3_KORVO2_V3_BOARD)
    i2s_cfg.multi_out_num = 1;
#endif
    i2s_cfg.task_core    = 1;
    i2s_cfg.stack_in_ext = true;
    i2s_stream_writer    = i2s_stream_init(&i2s_cfg);

    audio_pipeline_register(player, raw_write, "raw");
    audio_pipeline_register(player, i2s_stream_writer, "i2s");
    const char *link_tag[3] = { "raw", "i2s" };
    audio_pipeline_link(player, &link_tag[0], 2);

    return ESP_OK;
}

static void setup_audio(void)
{
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
    audio_hal_set_volume(board_handle->audio_hal, 60);

    es7210_mic_select(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2 | ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4);
    set_es7210_tdm_mode();

    recorder_pipeline_open();
    player_pipeline_open();
}

static void init_camera(void)
{
    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
    }
}

static void *init_jpeg_encoder(int quality, int hfm_core, int hfm_priority, jpeg_subsampling_t subsampling)
{
    jpeg_enc_info_t jpeg_enc_info = { 0 };
    jpeg_enc_info.width       = CAMERA_WIDTH;
    jpeg_enc_info.height      = CAMERA_HIGH;
    jpeg_enc_info.src_type    = JPEG_RAW_TYPE_YCbY2YCrY2;  //conv_mode = YUV422_TO_YUV420 
    // jpeg_enc_info.src_type    = JPEG_RAW_TYPE_YCbYCr;
    jpeg_enc_info.subsampling = subsampling;
    jpeg_enc_info.quality     = quality;
    // jpeg_enc_info.task_enable = true;
    jpeg_enc_info.hfm_task_core     = hfm_core;
    jpeg_enc_info.hfm_task_priority = hfm_priority;
    return jpeg_enc_open(&jpeg_enc_info);
}

/**
 * Agora video call related functions
 */
static int send_video_frame(uint8_t *data, uint32_t len)
{
    int rval;

    // API: send video data
    ago_video_frame_t ago_frame = { 0 };
    ago_frame.data_type         = AGO_VIDEO_DATA_TYPE_JPEG;
    ago_frame.is_key_frame      = true;
    ago_frame.video_buffer      = data;
    ago_frame.video_buffer_size = len;
    rval = agora_iot_push_video_frame(g_handle, &ago_frame, g_push_type);
    if (rval < 0) {
        ESP_LOGE(TAG, "Failed to push video frame");
        return -1;
    }

    return 0;
}

static int send_audio_frame(uint8_t *data, uint32_t len)
{
    int rval;

    // API: send audio data
    ago_audio_frame_t ago_frame = { 0 };
    ago_frame.data_type         = AGO_AUDIO_DATA_TYPE_PCM;
    ago_frame.audio_buffer      = data;
    ago_frame.audio_buffer_size = len;
    rval = agora_iot_push_audio_frame(g_handle, &ago_frame, g_push_type);
    if (rval < 0) {
        ESP_LOGE(TAG, "Failed to push audio frame");
        return -1;
    }

    return 0;
}

static void video_capture_and_send_task(void *args)
{
    const int image_buf_len = 30 * 1024;
    int image_len = 0;
    void *jpg_encoder = NULL;

#ifdef CONFIG_ENABLE_RUN_TIME_STATS
    esp_timer_handle_t fps_timer = NULL;

    esp_timer_create_args_t create_args = { .callback = fps_timer_callback, .arg = NULL, .name = "fps timer" };
    esp_timer_create(&create_args, &fps_timer);
    esp_timer_start_periodic(fps_timer, 20 * 1000 * 1000);
    tick_begin = xTaskGetTickCount();
    image_cnt = 0;
#endif

    init_camera();

    jpg_encoder = init_jpeg_encoder(40, 0, 20, JPEG_SUB_SAMPLE_YUV420);
    if (!jpg_encoder) {
        ESP_LOGE(TAG, "Failed to initialize jpeg encoder!");
        goto EXIT;
    }

    uint8_t *image_buf = heap_caps_malloc(image_buf_len, MALLOC_CAP_SPIRAM);
    if (!image_buf) {
        ESP_LOGE(TAG, "Failed to alloc video buffer!");
        goto EXIT;
    }

    while (1) {
        xSemaphoreTake(g_video_capture_sem, portMAX_DELAY);

        while (g_app.b_call_session_started) {
            camera_fb_t *pic = esp_camera_fb_get();
            image_cnt++;

            jpeg_enc_process(jpg_encoder, pic->buf, pic->len, image_buf, image_buf_len, &image_len);

            // ESP_LOGI(TAG, "YUV len %d, JPEG len %d", pic->len, image_len);
            send_video_frame(image_buf, image_len);

            esp_camera_fb_return(pic);
            usleep(30 * 1000);
        }
    }

EXIT:
    if (image_buf) {
        free(image_buf);
    }
    if (jpg_encoder) {
        jpeg_enc_close(jpg_encoder);
    }

#ifdef CONFIG_ENABLE_RUN_TIME_STATS
    esp_timer_stop(fps_timer);
    esp_timer_delete(fps_timer);
#endif
    vTaskDelete(NULL);
}

static void audio_capture_and_send_task(void *threadid)
{
    int read_len = DEFAULT_PCM_CAPTURE_LEN;
    int ret;

    uint8_t *pcm_buf = heap_caps_malloc(read_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm_buf) {
        ESP_LOGE(TAG, "Failed to alloc audio buffer!");
        return;
    }

    audio_pipeline_run(recorder);
    audio_pipeline_run(player);
    while (1) {
        xSemaphoreTake(g_audio_capture_sem, portMAX_DELAY);

        while (g_app.b_call_session_started) {
            ret = raw_stream_read(raw_read, (char *)pcm_buf, read_len);
            if (ret != read_len) {
                ESP_LOGW(TAG, "write error, expect %d, but only %d", read_len, ret);
            }
            send_audio_frame(pcm_buf, DEFAULT_PCM_CAPTURE_LEN);
        }
    }

    audio_pipeline_stop(recorder);
    audio_pipeline_wait_for_stop(recorder);
    audio_pipeline_terminate(recorder);

    audio_pipeline_stop(player);
    audio_pipeline_wait_for_stop(player);
    audio_pipeline_terminate(player);

    free(pcm_buf);
    vTaskDelete(NULL);
}

static void create_capture_task(void)
{
  int rval;

#ifndef CONFIG_AUDIO_ONLY
  g_video_capture_sem = xSemaphoreCreateBinary();
  if (NULL == g_video_capture_sem) {
    ESP_LOGE(TAG, "Unable to create video capture semaphore!");
    return;
  }
  rval = xTaskCreatePinnedToCore(video_capture_and_send_task, "video_task", 3 * 1024, NULL, PRIO_TASK_FETCH, NULL, 1);
  if (rval != pdTRUE) {
    ESP_LOGE(TAG, "Unable to create video capture thread!");
    return;
  }
#endif

  g_audio_capture_sem = xSemaphoreCreateBinary();
  if (NULL == g_audio_capture_sem) {
    ESP_LOGE(TAG, "Unable to create audio capture semaphore!");
    return;
  }
  rval = xTaskCreatePinnedToCore(audio_capture_and_send_task, "audio_task", 3 * 1024, NULL, PRIO_TASK_FETCH, NULL, 1);
  if (rval != pdTRUE) {
    ESP_LOGE(TAG, "Unable to create audio capture thread!");
    return;
  }
}


static void iot_cb_call_request(const char *peer_name, const char *attach_msg)
{
    if (!peer_name) {
        return;
    }

    ESP_LOGI(TAG, "Get call from peer \"%s\", attach message: %s", peer_name, attach_msg ? attach_msg : "null");

    agora_iot_answer(g_handle);
}

static void iot_cb_start_push_frame(uint8_t push_type)
{
    ESP_LOGI(TAG, "Start push audio/video frames");
    g_app.b_call_session_started = true;

    // record push type
    g_push_type |= push_type;

#ifndef CONFIG_AUDIO_ONLY
    xSemaphoreGive(g_video_capture_sem);
#endif
    xSemaphoreGive(g_audio_capture_sem);
}

static void iot_cb_stop_push_frame(uint8_t push_type)
{
    ESP_LOGI(TAG, "Stop push audio/video frames");
    g_app.b_call_session_started = false;

    xSemaphoreGive(g_service_sem);

    // record push type
    g_push_type &= ~push_type;
}

static void iot_cb_call_hung_up(const char *peer_name)
{
    if (!peer_name) {
        ESP_LOGI(TAG, "Get hangup from peer \"%s\"", peer_name);
    }
}

static void iot_cb_call_answered(const char *peer_name)
{
    if (!peer_name) {
        ESP_LOGI(TAG, "Get answer from peer \"%s\"", peer_name);
    }
}

static void iot_cb_call_timeout(const char *peer_name)
{
    if (!peer_name) {
        ESP_LOGI(TAG, "No answer from peer \"%s\"", peer_name);
    }
}

static void iot_cb_receive_video_frame(ago_video_frame_t *frame)
{
}

static void iot_cb_receive_audio_frame(ago_audio_frame_t *frame)
{
    raw_stream_write(raw_write, (char *)frame->audio_buffer, frame->audio_buffer_size);
}

static RTC_DATA_ATTR struct timeval sleep_enter_time;
static void handle_wakeup_cause(void)
{
#ifdef CONFIG_ENABLE_DEEP_SLEEP
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
        case ESP_SLEEP_WAKEUP_GPIO: {
            uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
#endif //SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
#ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
        case ESP_SLEEP_WAKEUP_TOUCHPAD: {
            printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
            break;
        }
#endif // CONFIG_EXAMPLE_TOUCH_WAKEUP
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
            break;
    }
#endif
}

int app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    handle_wakeup_cause();

#ifdef CONFIG_ENABLE_LIGHT_SLEEP
    esp_pm_config_esp32s3_t pm_config = { .max_freq_mhz = CONFIG_EXAMPLE_MAX_CPU_FREQ_MHZ,
                                          .min_freq_mhz = CONFIG_EXAMPLE_MIN_CPU_FREQ_MHZ,
                                          .light_sleep_enable = true };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

#ifdef CONFIG_ENABLE_DEEP_SLEEP
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    const gpio_config_t config = {
        // .pin_bit_mask = ((1ULL << 2) | (1ULL << 4))
        .pin_bit_mask = BIT(DEFAULT_WAKEUP_PIN),
        .mode = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT(DEFAULT_WAKEUP_PIN), DEFAULT_WAKEUP_LEVEL));
    // ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(((1ULL << 2) | (1ULL << 4)) , ESP_GPIO_WAKEUP_GPIO_HIGH));
    printf("Enabling GPIO wakeup on pins GPIO%d\n", DEFAULT_WAKEUP_PIN);
#endif //SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
#endif

    setup_wifi();

    setup_audio();

    create_capture_task();

    // Wait until WiFi is connected
    while (!g_app.b_wifi_connected) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "device_id: %s, cpu_freq: %d", get_device_id(), esp_clk_cpu_freq() / MHZ);

    char *cert        = NULL;
  // 1. activate license
    if (0 != agora_iot_license_activate(CONFIG_AGORA_APP_ID, CONFIG_CUSTOMER_KEY, CONFIG_CUSTOMER_SECRET,
                                        CONFIG_PRODUCT_KEY, get_device_id(), CONFIG_LICENSE_PID, &cert)) {
        ESP_LOGE(TAG, "cannot activate agora license !\n");
        return -1;
    }

    // Initialize Agora IoT SDK
    agora_iot_device_info_t device_info = { 0 };
    if (0 != agora_iot_register_and_bind(CONFIG_MASTER_SERVER_URL, CONFIG_PRODUCT_KEY, get_device_id(),
                                         NULL, NULL, &device_info)) {
        ESP_LOGE(TAG, "Failed to register device to aws");
        return -1;
    }

    agora_iot_config_t cfg = {
        .app_id      = CONFIG_AGORA_APP_ID,
        .product_key = CONFIG_PRODUCT_KEY,
        .client_id   = device_info.client_id,
        .domain      = device_info.domain,
        .root_ca     = CONFIG_AWS_ROOT_CA,
        .client_crt  = device_info.certificate,
        .client_key  = device_info.private_key,
        .enable_rtc  = true,
        .certificate = cert,
        .enable_recv_audio = true,
        .enable_recv_video = false,
        .rtc_cb = {
        .cb_start_push_frame    = iot_cb_start_push_frame,
        .cb_stop_push_frame     = iot_cb_stop_push_frame,
        .cb_receive_audio_frame = iot_cb_receive_audio_frame,
        .cb_receive_video_frame = iot_cb_receive_video_frame,
    #ifdef CONFIG_SEND_H264_FRAMES
        .cb_target_bitrate_changed = iot_cb_target_bitrate_changed,
        .cb_key_frame_requested    = iot_cb_key_frame_requested,
    #endif
        },
        .disable_rtc_log      = true,
        .max_possible_bitrate = DEFAULT_MAX_BITRATE,
        .enable_audio_config  = true,
        .audio_config = {
    #ifdef CONFIG_AUDIO_SAMPLE_RATE_8K
        .audio_codec_type = AGO_AUDIO_CODEC_TYPE_G711U,
    #else
        .audio_codec_type = AGO_AUDIO_CODEC_TYPE_G722,
    #endif
    #if defined(CONFIG_SEND_PCM_DATA)
        .pcm_sample_rate  = I2S_SAMPLE_RATE,
        .pcm_channel_num  = I2S_CHANNELS,
    #endif
        },

        .slave_server_url = CONFIG_SLAVE_SERVER_URL,
        .call_cb = {
        .cb_call_request       = iot_cb_call_request,
        .cb_call_answered      = iot_cb_call_answered,
        .cb_call_hung_up       = iot_cb_call_hung_up,
        .cb_call_local_timeout = iot_cb_call_timeout,
        .cb_call_peer_timeout  = iot_cb_call_timeout,
        },
    };
    g_handle = agora_iot_init(&cfg);
    if (!g_handle) {
        // if failed to initialize, no need to deinitialize
        ESP_LOGE(TAG, "Failed to initialize Agoro IoT");
        return -1;
    }

    ESP_LOGI(TAG, "Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    // Monitor the key event
    start_key_service(set);

    // init_lcd_service(set);

    // Infinite loop
    while (!g_app.b_exit) {
        ESP_LOGI(TAG, "current cpu_freq: %d", esp_clk_cpu_freq() / MHZ);
        if (!g_app.b_call_session_started) {
            ESP_LOGW(TAG, "Please press [REC] key to ring the doorbell ...");
        } else {
            ESP_LOGW(TAG, "Now we're in the call. Please press [VOL+] key to hang up ...");
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);

#ifdef CONFIG_ENABLE_DEEP_SLEEP
        printf("Entering deep sleep\n");
        gettimeofday(&sleep_enter_time, NULL);

        esp_sleep_enable_timer_wakeup(20000000);
        esp_deep_sleep_start();
#endif
    }

    // Deinit Agora IoT SDK
    agora_iot_deinit(g_handle);

    free(cert);

    ESP_LOGW(TAG, "App exited.");

    return 0;
}
