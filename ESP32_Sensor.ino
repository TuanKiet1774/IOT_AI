#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ----- Cấu hình WiFi -----
const char* WIFI_SSID = "Hi";
const char* WIFI_PASSWORD = "hehehehe";

// ----- Cấu hình MQTT ThingsBoard -----
const char* THINGSBOARD_SERVER = "thingsboard.cloud";
const char* ACCESS_TOKEN = "TPYOB7bRc8fys228pH9T";

WiFiClient espClient;
PubSubClient client(espClient);

// ----- Cấu hình cảm biến MQ-5 -----
#define MQ5_PIN 34  // Chân ADC của ESP32
const int GAS_THRESHOLD = 1000; // Ngưỡng cảnh báo khí gas

// ----- Cấu hình cảm biến ACS-712 -----
#define ACS712_PIN 35  // Chân đọc dòng điện của ESP32

// ----- Cấu hình cảm biến DHT-22 -----
#define DHT_PIN 32   // Chân dữ liệu của DHT-22
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
    Serial.begin(115200);
    dht.begin();
    
    // Kết nối WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Đang kết nối WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ Đã kết nối WiFi!");

    // Kết nối MQTT ThingsBoard
    client.setServer(THINGSBOARD_SERVER, 1883);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    
    // Đọc giá trị từ MQ-5
    int gasValue = analogRead(MQ5_PIN);
    
    float sum = 0;
    int numSamples = 1000;
    for (int i = 0; i < numSamples; i++) {
        sum += analogRead(ACS712_PIN);
        delay(1);
    }
    float avgVoltage = (sum / numSamples) * (3.3 / 4095.0); // Chuyển ADC -> V
    float current = (avgVoltage - 2.5) / 0.185; // Đổi V -> A với ACS712-5A
    if (current < 0) current = -current; // Đảm bảo giá trị luôn dương


    // Đọc nhiệt độ & độ ẩm từ DHT-22
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // Hiển thị dữ liệu trên Serial Monitor
    Serial.printf("Gas: %d, Current: %.2fA, 🌡 Temperature: %.2f°C, Humidity: %.2f%%\n", 
                  gasValue, current, temperature, humidity);

    // Gửi dữ liệu lên ThingsBoard
    String payload = "{";
    payload += "\"gas\":" + String(gasValue) + ",";
    payload += "\"current\":" + String(current) + ",";
    payload += "\"temperature\":" + String(temperature) + ",";
    payload += "\"humidity\":" + String(humidity);
    payload += "}";

    Serial.print("📤 Gửi dữ liệu: ");
    Serial.println(payload);

    client.publish("v1/devices/me/telemetry", payload.c_str());

    delay(5000);
}

// Hàm kết nối lại MQTT nếu bị mất kết nối
void reconnect() {
    while (!client.connected()) {
        Serial.print("🔄 Đang kết nối MQTT...");
        if (client.connect("ESP32_Client", ACCESS_TOKEN, "")) {
            Serial.println("✅ Kết nối MQTT thành công!");
        } else {
            Serial.print("❌ Thất bại, mã lỗi: ");
            Serial.println(client.state());
            delay(2000);
        }
    }
}