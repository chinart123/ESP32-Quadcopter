// ========================================== 
// 1. CẤU HÌNH VAI TRÒ (CHỌN 1 TRONG 2) 
// ========================================== 
#define ROLE_TRANSMITTER // Bật dòng này để làm mạch Phát (Tay cầm) 
//#define ROLE_RECEIVER // Bật dòng này để làm mạch Nhận (Drone FC) 
 
// ========================================== 
// 2. CẤU HÌNH KIẾN TRÚC (BẬT ĐỂ DÙNG 2 CORE) 
// ========================================== 
#define USE_DUAL_CORE // Bật dòng này để dùng FreeRTOS tách luồng. Tắt đi (//) để chạy 1 Core. 
 
// --- Khai báo chân (Pins) ĐÃ ĐƯỢC CHUYỂN SANG VÙNG AN TOÀN --- 
const int pwmTxPin = 7; // Chân phát PWM (chỉ dùng khi là TX) 
const int pwmRxPin = 6; // Chân nhận PWM (chỉ dùng khi là RX) 
 
TaskHandle_t Core0TaskHandle; // Biến quản lý Task nếu dùng Dual Core 

// --- KHỐI NGẮT PHẦN CỨNG (DÙNG CHO RX) ---
#ifdef ROLE_RECEIVER
  volatile unsigned long pulseStartTime = 0;
  volatile unsigned long pwmValue = 0;

  void IRAM_ATTR pwmInterruptHandler() {
    if (digitalRead(pwmRxPin) == HIGH) {
      pulseStartTime = micros(); 
    } else {
      pwmValue = micros() - pulseStartTime; 
    }
  }
#endif
 
void setup() { 
  Serial.begin(115200); 
  delay(1000); // Chờ Serial ổn định 
 
  // --------------------------------------------------------- 
  // KHỞI TẠO PHẦN CỨNG THEO VAI TRÒ 
  // --------------------------------------------------------- 
  #ifdef ROLE_TRANSMITTER 
    Serial.println(">>> CHẾ ĐỘ: TAY CẦM PHÁT (TX) - ARDUINO API <<<"); 
    // ESP32-S3 chỉ hỗ trợ tối đa 14-bit
    if (ledcAttach(pwmTxPin, 50, 14)) {
      ledcWrite(pwmTxPin, 1229); // Phát độ rộng 1500us ở thang 14-bit
    }
  #elif defined(ROLE_RECEIVER) 
    Serial.println(">>> CHẾ ĐỘ: DRONE NHẬN (RX) - NGẮT ARDUINO <<<"); 
    pinMode(pwmRxPin, INPUT_PULLDOWN); 
    attachInterrupt(digitalPinToInterrupt(pwmRxPin), pwmInterruptHandler, CHANGE);
  #endif 
 
  // --------------------------------------------------------- 
  // KHỞI TẠO KIẾN TRÚC THEO SỐ CORE 
  // --------------------------------------------------------- 
  #ifdef USE_DUAL_CORE 
    Serial.println("Kiến trúc: DUAL CORE (Sử dụng FreeRTOS trên Core 0)"); 
    #ifdef ROLE_TRANSMITTER 
      xTaskCreatePinnedToCore(txTask, "TX_Task", 2048, NULL, 1, &Core0TaskHandle, 0); 
    #elif defined(ROLE_RECEIVER) 
      xTaskCreatePinnedToCore(rxTask, "RX_Task", 2048, NULL, 2, &Core0TaskHandle, 0); 
    #endif 
  #else 
    Serial.println("Kiến trúc: SINGLE CORE (Chạy toàn bộ trên Core 1)"); 
  #endif 
  Serial.println("============================="); 
} 

void loop() { 
  // ============================================================== 
  // VÒNG LẬP CHÍNH CỦA HỆ THỐNG (LUÔN CHẠY TRÊN CORE 1) 
  // ============================================================== 
  #ifdef USE_DUAL_CORE 
    // --- NẾU DÙNG DUAL CORE --- 
    #ifdef ROLE_TRANSMITTER 
      Serial.println("[Core 1] Đang rảnh rỗi chờ xử lý nút bấm/màn hình..."); 
      delay(2000); 
    #elif defined(ROLE_RECEIVER) 
      Serial.println("[Core 1] Đang chạy vòng lặp PID siêu mượt (không bị delay)..."); 
      delay(500); // Giả lập thời gian tính toán thuật toán 
    #endif 

  #else 
    // --- NẾU CHỈ DÙNG 1 CORE (SINGLE CORE) --- 
    #ifdef ROLE_TRANSMITTER 
      Serial.println("[Single Core] TX vẫn đang phát PWM tự động bằng phần cứng..."); 
      delay(2000); 
    #elif defined(ROLE_RECEIVER) 
      // Không còn bị chặn bởi pulseIn. Chỉ cần lấy biến từ Ngắt ra in.
      if (pwmValue > 0) { 
        Serial.print("[Single Core] Nhận được PWM: "); 
        Serial.print(pwmValue); 
        Serial.println(" us"); 
      } 
      delay(20); 
    #endif 
  #endif 
}

// ============================================================= 
// CÁC HÀM XỬ LÝ TRÊN CORE 0 (CHỈ ĐƯỢC DỊCH KHI BẬT DUAL CORE) 
// ============================================================= 
#ifdef USE_DUAL_CORE 

  #ifdef ROLE_TRANSMITTER 
  void txTask(void *pvParam) { 
    for(;;) { 
      // Serial.println("[Core 0] Task phụ của TX..."); 
      vTaskDelay(2000 / portTICK_PERIOD_MS); // Bắt buộc dùng vTaskDelay 
    } 
  } 
  #endif 

  #ifdef ROLE_RECEIVER 
  void rxTask(void *pvParam) { 
    for(;;) { 
      // Nhiệm vụ chờ xung đã do phần cứng (EXTI) lo, Core 0 chỉ việc lấy kết quả
      if (pwmValue > 0) { 
        Serial.print("[Core 0] Nhận được PWM (Không ảnh hưởng Core 1): "); 
        Serial.print(pwmValue); 
        Serial.println(" us"); 
      } 
      vTaskDelay(20 / portTICK_PERIOD_MS); // Nghỉ ngơi nhường CPU cho WiFi/OS 
    } 
  } 
  #endif 

#endif

/* CODE ESP-IDF
// ========================================== 
#define ROLE_TRANSMITTER 
//#define ROLE_RECEIVER  

#define USE_DUAL_CORE 
 
const int pwmTxPin = 7; 
const int pwmRxPin = 6; 
 
TaskHandle_t Core0TaskHandle; 

// KHÁC BIỆT: Bắt buộc gọi thư viện lõi cho TX
#ifdef ROLE_TRANSMITTER
  #include "driver/ledc.h" 
#endif

// --- KHỐI NGẮT PHẦN CỨNG (DÙNG CHO RX) ---
#ifdef ROLE_RECEIVER
  volatile unsigned long pulseStartTime = 0;
  volatile unsigned long pwmValue = 0;

  void IRAM_ATTR pwmInterruptHandler() {
    if (digitalRead(pwmRxPin) == HIGH) {
      pulseStartTime = micros(); 
    } else {
      pwmValue = micros() - pulseStartTime; 
    }
  }
#endif
 
void setup() { 
  Serial.begin(115200); 
  delay(1000); 
 
  // --------------------------------------------------------- 
  #ifdef ROLE_TRANSMITTER 
    Serial.println(">>> CHẾ ĐỘ: TAY CẦM PHÁT (TX) - ESP-IDF NATIVE <<<"); 
    
    // Cấu hình Timer bằng Struct 14-bit
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = pwmTxPin,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 1229, // 1500us
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

  #elif defined(ROLE_RECEIVER) 
    Serial.println(">>> CHẾ ĐỘ: DRONE NHẬN (RX) - NGẮT EXTI <<<"); 
    pinMode(pwmRxPin, INPUT_PULLDOWN); 
    attachInterrupt(pwmRxPin, pwmInterruptHandler, CHANGE);
  #endif 
 
  // --------------------------------------------------------- 
  #ifdef USE_DUAL_CORE 
    #ifdef ROLE_TRANSMITTER 
      xTaskCreatePinnedToCore(txTask, "TX_Task", 2048, NULL, 1, &Core0TaskHandle, 0); 
    #elif defined(ROLE_RECEIVER) 
      xTaskCreatePinnedToCore(rxTask, "RX_Task", 2048, NULL, 2, &Core0TaskHandle, 0); 
    #endif 
  #endif 
} 

// ... Khối void loop() và hàm Core 0 tương tự như DẠNG 1 ...
void loop() { 
  #ifdef USE_DUAL_CORE 
    #ifdef ROLE_TRANSMITTER 
      delay(2000); 
    #elif defined(ROLE_RECEIVER) 
      Serial.println("[Core 1] Đang chạy vòng lặp PID siêu mượt (không bị delay)..."); 
      delay(500); 
    #endif 
  #else 
    #ifdef ROLE_TRANSMITTER 
      delay(2000); 
    #elif defined(ROLE_RECEIVER) 
      if (pwmValue > 0) { 
        Serial.print("[Single Core] Nhận được PWM: "); 
        Serial.println(pwmValue); 
      } 
      delay(20); 
    #endif 
  #endif 
}

#ifdef USE_DUAL_CORE 
  #ifdef ROLE_TRANSMITTER 
  void txTask(void *pvParam) { 
    for(;;) { vTaskDelay(2000 / portTICK_PERIOD_MS); } 
  } 
  #endif 
  #ifdef ROLE_RECEIVER 
  void rxTask(void *pvParam) { 
    for(;;) { 
      if (pwmValue > 0) { 
        Serial.print("[Core 0] Nhận được PWM (Không ảnh hưởng Core 1): "); 
        Serial.println(pwmValue); 
      } 
      vTaskDelay(20 / portTICK_PERIOD_MS); 
    } 
  } 
  #endif 
#endif
*/