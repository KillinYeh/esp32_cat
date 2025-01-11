#include <esp32-cam-cat-dog_inferencing.h>  // replace with your deployed Edge Impulse library
#include <Arduino.h>
#define CAMERA_MODEL_AI_THINKER
#include "img_converters.h"
#include "image_util.h"
#include "esp_camera.h"
#include "camera_pins.h"

// 按鍵（可用來觸發拍照推論）
#define BTN       4 // button (shared with flash led)

// Edge Impulse 推論需要的全域變數
dl_matrix3du_t *resized_matrix = NULL;
ei_impulse_result_t result = {0};

#define CAM_SERIAL      Serial1
#define CAM_RX_PIN        14   
#define CAM_TX_PIN        12 
#define BAUD_RATE       115200

//------------------------------------------------------------------------------
// 函式宣告
//------------------------------------------------------------------------------
String classify();
void capture_quick();
bool capture();
int raw_feature_get_data(size_t offset, size_t out_len, float *signal_ptr);

//------------------------------------------------------------------------------
// setup
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  CAM_SERIAL.begin(BAUD_RATE, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  while (!CAM_SERIAL) {
    delay(10);
  }
  // 按鍵初始化
  pinMode(BTN, INPUT);

  // 相機設定
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_240X240;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // 相機初始化
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // 若為 OV3660，可嘗試翻轉、調整飽和度等 (依需求可自行刪減)
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);      // 垂直翻轉
    s->set_brightness(s, 1); // 增加亮度
    s->set_saturation(s, 0); // 降低飽和度
  }

  Serial.println("Camera Ready!...(standby, press button to start)");
}

//------------------------------------------------------------------------------
// loop
//------------------------------------------------------------------------------
void loop() 
{
  if (CAM_SERIAL.available()) 
  {   

      String cmd = CAM_SERIAL.readStringUntil('\n');
      cmd.trim();
  
      if (cmd == "CAPTURE") 
      {
        // 執行拍照 + 推論
        Serial.println("CAPTURE");
        String resultStr = classify();
        // 把推論結果透過 UART 回覆
        // S3 會收到這行 (e.g. "cat" or "dog" or "Capture Error")
        Serial.printf("Result: %s\n", resultStr.c_str());
        CAM_SERIAL.println(resultStr);
        CAM_SERIAL.flush();
      }
      else
      {
        // 不認得的指令
        CAM_SERIAL.println("UNKNOWN_CMD");
      }
   }
}

//------------------------------------------------------------------------------
// 貓狗分類的主要流程
//------------------------------------------------------------------------------
String classify() {
  // 先做一次 quick capture 清除 buffer
  capture_quick();

  // 拍照並檢查成功與否
  if (!capture()) {
    return "Capture Error";
  }

  Serial.println("Running inference...");

  // Edge Impulse 處理需要的 signal
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &raw_feature_get_data;

  // 執行推論
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);

  // 釋放 resized_matrix
  dl_matrix3du_free(resized_matrix);
  resized_matrix = NULL;

  // 若發生錯誤
  if (res != 0) {
    Serial.println("run_classifier returned error.");
    return "Inference Error";
  }

  // 依照最高分數來回傳 label
  float scoreMax = 0.0;
  int indexMax = 0;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    float score = result.classification[ix].value;
    Serial.printf("    %s: %f\n", result.classification[ix].label, score);
    if (score > scoreMax) {
      scoreMax = score;
      indexMax = ix;
    }
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  Serial.printf("    anomaly score: %f\n", result.anomaly);
#endif

  // 回傳最可能的分類名稱
  return String(result.classification[indexMax].label);
}

//------------------------------------------------------------------------------
// 擷取影像(快速清除 buffer)
//------------------------------------------------------------------------------
void capture_quick() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    return;
  }
  // 取完立即釋放
  esp_camera_fb_return(fb);
}

//------------------------------------------------------------------------------
// 拍照 + 轉成 Edge Impulse 所需的大小 (RGB888 -> Resize -> 存到 resized_matrix)
//------------------------------------------------------------------------------
bool capture() {
  Serial.println("Capture image...");
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  // 轉為 RGB888
  dl_matrix3du_t *rgb888_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  fmt2rgb888(fb->buf, fb->len, fb->format, rgb888_matrix->item);

  // Resize 到 EI_CLASSIFIER_INPUT_WIDTH x EI_CLASSIFIER_INPUT_HEIGHT
  resized_matrix = dl_matrix3du_alloc(
    1, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3
  );
  image_resize_linear(
    resized_matrix->item,
    rgb888_matrix->item,
    EI_CLASSIFIER_INPUT_WIDTH,
    EI_CLASSIFIER_INPUT_HEIGHT,
    3,
    fb->width,
    fb->height
  );

  // 釋放記憶體
  dl_matrix3du_free(rgb888_matrix);
  esp_camera_fb_return(fb);
  return true;
}

//------------------------------------------------------------------------------
// Edge Impulse 傳入資料的 callback
//------------------------------------------------------------------------------
int raw_feature_get_data(size_t offset, size_t out_len, float *signal_ptr) {
  size_t pixel_ix = offset * 3;
  size_t bytes_left = out_len;
  size_t out_ptr_ix = 0;

  // 一次一次像素取出 R/G/B
  while (bytes_left != 0) {
    uint8_t r = resized_matrix->item[pixel_ix];
    uint8_t g = resized_matrix->item[pixel_ix + 1];
    uint8_t b = resized_matrix->item[pixel_ix + 2];

    float pixel_f = (r << 16) + (g << 8) + b;
    signal_ptr[out_ptr_ix] = pixel_f;

    out_ptr_ix++;
    pixel_ix += 3;
    bytes_left--;
  }

  return 0;
}
