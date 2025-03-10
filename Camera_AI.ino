#include <TuanKiet174-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
 
#define BLYNK_TEMPLATE_ID "TMPL61PGlsrmP"
#define BLYNK_TEMPLATE_NAME "Fire Detection"
#define BLYNK_AUTH_TOKEN "Mã Auth Token"

#include <BlynkSimpleEsp32.h> //Thư viện cho phép bạn kết nối ESP32 với app Blynk
#include <WiFiClient.h> //Cung cấp các hàm để kết nối, gửi và nhận dữ liệu qua mạng
#include <WiFi.h> //Cung cấp các hàm để quản lý kết nối Wi-Fi

//char ssid[] = "Tuan Kiet";
//char pass[] = "Pass wifi";
char ssid[] = "Hi";
char pass[] = "hehehehe";
char auth[] = BLYNK_AUTH_TOKEN;

#define CAMERA_MODEL_AI_THINKER 
#define RELAY_PIN 12 // GPIO for controlling water pump relay

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

static bool debug_nn = false; 
static bool is_initialised = false;
uint8_t *snapshot_buf = nullptr; //points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,

    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// Function prototypes
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

/**
 * @brief      Initialize camera
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1); // flip it back
        s->set_brightness(s, 1); // up the brightness just a bit
        s->set_saturation(s, 0); // lower the saturation
    }

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK) {
        ei_printf("Camera không thành công\n");
        return;
    }
    is_initialised = false;
}

/**
 * @brief      Capture, rescale and crop image
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera chưa được khởi tạo\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Không chụp được ảnh\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if(!converted){
        ei_printf("Chuyển đổi không thành công\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }
    return true;
}

/**
 * @brief      Get data from the camera
 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + 
                               (snapshot_buf[pixel_ix + 1] << 8) + 
                               snapshot_buf[pixel_ix];
        
        // Normalize to 0-1 range
        out_ptr[out_ptr_ix] /= 255.0;
        
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }

    return 0;
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    // Initialize relay pin
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    Serial.println("CAMERA NHẬN DIỆN ĐÁM CHÁY BẰNG AI");
    
    // setup kết nối wifi
    WiFi.begin(ssid, pass); //Khởi động kết nối WiFi với sử dụng SSID và mật khẩu
    while (WiFi.status() != WL_CONNECTED) //lấy trạng thái Wifi - WL_CONNECTED trạng thái đã kết nối
    {
      delay(1000);
      Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to WiFi");

    // kết nối blynk với mã thông báo xác thực, tên wifi và mật khẩu
    Blynk.begin(auth, ssid, pass);
    delay(1000);

    // Allocate snapshot buffer
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * 
                                    EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 
                                    EI_CAMERA_FRAME_BYTE_SIZE);
    
    if (!snapshot_buf) {
        Serial.println("Không phân bổ được bộ đệm ảnh chụp nhanh");
        return;
    }

    // Initialize camera
    if (ei_camera_init() == false) {
        Serial.println("Khởi tạo camera không thành công!");
        free(snapshot_buf);
        return;
    }

    Serial.println("Camera đã được khởi tạo thành công");
    Serial.println("Bắt đầu nhận diện trong 2 giây nữa...");
    delay(2000);
}

void loop() {
    Blynk.run();
    // Check if snapshot buffer is allocated
    if (!snapshot_buf) {
        Serial.println("Snapshot buffer not allocated");
        return;
    }

    // Capture image
    if (ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, 
                          EI_CLASSIFIER_INPUT_HEIGHT, 
                          snapshot_buf) == false) {
        Serial.println("Không chụp được ảnh");
        return;
    }

    // Prepare signal for classifier
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Run classifier
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    
    if (err != EI_IMPULSE_OK) {
        Serial.printf("Không chạy được trình phân loại (%d)\n", err);
        return;
    }

    // Print predictions and check for fire
    int fire_detected = 0;
    Serial.println("Phát hiện: ");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_impulse_result_bounding_box_t prediction = result.bounding_boxes[i];
        Serial.printf("  %s: %.2f\n", 
                      ei_classifier_inferencing_categories[i], 
                      prediction.value);
        Blynk.virtualWrite(V0, prediction.value); //Gửi lên Blynk 

        // Check for fire with high confidence
        if (strcmp(ei_classifier_inferencing_categories[i], "fire") == 0 && 
            prediction.value > 0.8) {
              fire_detected = 1;
        }
    }

    // Activate water pump if fire is detected
    if (fire_detected) {
        Serial.println("ĐÃ PHÁT HIỆN CHÁY! Đang kích hoạt máy bơm nước...");
        Serial.write(fire_detected); //Gửi tín hiệu về mạch chính
        digitalWrite(RELAY_PIN, HIGH);
        delay(5000);  // Run pump for 5 seconds
        digitalWrite(RELAY_PIN, LOW);
    }

    // Small delay between iterations
    delay(1000);
}

// Ensure the model is for camera
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif