// ==========================================
// 1. CẤU HÌNH VAI TRÒ (CHỌN 1 TRONG 2)
// ==========================================
#define ROLE_TRANSMITTER  // Bật dòng này để làm mạch Phát (Tay cầm)
//#define ROLE_RECEIVER     // Bật dòng này để làm mạch Nhận (Drone)

// ==========================================
// 2. CẤU HÌNH KIẾN TRÚC (BẬT ĐỂ DÙNG 2 CORE)
// ==========================================
#define USE_DUAL_CORE     // Bật dòng này để dùng FreeRTOS tách luồng. Tắt đi (//) để chạy 1 Core.

// --- Khai báo chân (Pins) ---
const int pwmTxPin = 4; // Chân phát PWM (chỉ dùng khi là TX)
const int pwmRxPin = 5; // Chân nhận PWM (chỉ dùng khi là RX)

TaskHandle_t Core0TaskHandle; // Biến quản lý Task nếu dùng Dual Core

void setup() {
  Serial.begin(115200);
  delay(1000); // Chờ Serial ổn định

  // ---------------------------------------------------------
  // KHỞI TẠO PHẦN CỨNG THEO VAI TRÒ
  // ---------------------------------------------------------
  #ifdef ROLE_TRANSMITTER
    Serial.println(">>> CHẾ ĐỘ: TAY CẦM PHÁT (TX) <<<");
    ledcSetup(0, 50, 16);       // Tần số 50Hz, 16-bit
    ledcAttachPin(pwmTxPin, 0); // Gắn chân
    ledcWrite(0, 4915);         // Phát độ rộng 1500us
  #elif defined(ROLE_RECEIVER)
    Serial.println(">>> CHẾ ĐỘ: DRONE NHẬN (RX) <<<");
    pinMode(pwmRxPin, INPUT);
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
    Serial.println("Kiến trúc: SINGLE CORE (Chạy tuần tự trên Core 1)");
  #endif
  
  Serial.println("=====================================");
}

void loop() {
  // =========================================================
  // VÒNG LẶP CHÍNH CỦA HỆ THỐNG (LUÔN CHẠY TRÊN CORE 1)
  // =========================================================
  
  #ifdef USE_DUAL_CORE
    // --- NẾU DÙNG DUAL CORE ---
    // Các tác vụ block/chờ đợi đã bị ném sang Core 0. Core 1 rảnh rỗi.
    #ifdef ROLE_TRANSMITTER
      Serial.println("[Core 1] Đang rảnh rỗi chờ xử lý nút bấm/màn hình...");
      delay(1000); 
    #elif defined(ROLE_RECEIVER)
      Serial.println("[Core 1] Đang chạy vòng lặp PID siêu mượt (không bị delay)...");
      delay(10); // Giả lập thời gian tính toán thuật toán
    #endif

  #else
    // --- NẾU CHỈ DÙNG 1 CORE (SINGLE CORE) ---
    // Mọi thứ phải xếp hàng chạy tuần tự tại đây
    #ifdef ROLE_TRANSMITTER
      Serial.println("[Core 1] TX vẫn đang phát PWM tự động bằng phần cứng...");
      delay(1000);
    #elif defined(ROLE_RECEIVER)
      // Lệnh đọc này chặn (block) toàn bộ Core 1, PID phải đợi.
      unsigned long pulse = pulseIn(pwmRxPin, HIGH); 
      if (pulse > 0) {
        Serial.print("[Core 1] Nhận được PWM: ");
        Serial.print(pulse);
        Serial.println(" us");
      }
      delay(20);
    #endif

  #endif
}

// =========================================================
// CÁC HÀM XỬ LÝ TRÊN CORE 0 (CHỈ ĐƯỢC DỊCH KHI BẬT DUAL CORE)
// =========================================================
#ifdef USE_DUAL_CORE

  #ifdef ROLE_TRANSMITTER
  void txTask(void *pvParam) {
    for(;;) {
      Serial.println("[Core 0] Task phụ của TX (Ví dụ: Đọc ADC từ Joystick)...");
      vTaskDelay(500 / portTICK_PERIOD_MS); // Bắt buộc dùng vTaskDelay
    }
  }
  #endif

  #ifdef ROLE_RECEIVER
  void rxTask(void *pvParam) {
    for(;;) {
      // Tác vụ chờ xung (block) được nhốt hoàn toàn ở Core 0
      unsigned long pulse = pulseIn(pwmRxPin, HIGH);
      if (pulse > 0) {
        Serial.print("[Core 0] Nhận được PWM (Không ảnh hưởng Core 1): ");
        Serial.print(pulse);
        Serial.println(" us");
      }
      vTaskDelay(10 / portTICK_PERIOD_MS); // Nghỉ ngơi nhường CPU cho WiFi/OS
    }
  }
  #endif

#endif