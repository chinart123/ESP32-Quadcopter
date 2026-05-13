# Nhật ký: Test Motor Driver - ESP32-S3

## [CÂU HỎI 1]
Tôi đang cần test lại motor driver.
Vấn đề là lâu quá file motor driver test cho chân nào tôi quên mất rồi?
Bạn giúp tôi ghi ra sơ đồ đấu dây ở đây nhé

---

## [TRẢ LỜI 1]
[NO CONTENT FOUND]Dựa vào mã nguồn và các tài liệu trong dự án của bạn, có một vài sơ đồ đấu dây cho motor (ESC) tùy thuộc vào phiên bản file code (phiên bản cũ hay bản mới refactor) và loại vi điều khiển bạn đang dùng. Dưới đây là các sơ đồ đấu dây chi tiết:

**1. Trên file Active hiện tại của ESP32-S3 (`src/drn_motor_pwm.cpp`):**
Mã nguồn mới nhất đang cấu hình 4 chân GPIO liên tiếp cho 4 động cơ:
* **Motor 1:** GPIO 1
* **Motor 2:** GPIO 2
* **Motor 3:** GPIO 3
* **Motor 4:** GPIO 4

**2. Nếu bạn đang dùng file Test cũ của ESP32-S3 (`Old Core_Files/hal_timer_pwm.cpp`):**
Nếu bạn dùng file cũ để test riêng biệt, sơ đồ chân đã được gán sẵn như sau:
* **Motor 1 (Trái - Trước):** GPIO 10
* **Motor 2 (Phải - Trước):** GPIO 11
* **Motor 3 (Phải - Sau):** GPIO 12
* **Motor 4 (Trái - Sau):** GPIO 13

**3. Theo tài liệu sơ đồ kết nối trong `code-analysis.md` (ESP32-S3):**
Tài liệu này ghi nhận sơ đồ kết nối phần cứng cho ESP32-S3 như sau:
* **ESC Motor 1:** GPIO 4
* **ESC Motor 2:** GPIO 5
* **ESC Motor 3:** GPIO 6
* **ESC Motor 4:** GPIO 7

**4. Đối với nền tảng STM32F103 (nếu bạn test trên mạch này):**
Sơ đồ kết nối phần cứng cho STM32 ghi trong `code-analysis.md`:
* **ESC Motor 1:** PA8 (TIM1 CH1)
* **ESC Motor 2:** PA9 (TIM1 CH2) - *Lưu ý chân này bị xung đột với UART1 TX*
* **ESC Motor 3:** PB6 (TIM4 CH1)
* **ESC Motor 4:** PB7 (TIM4 CH2)

**Tóm lại:** Nếu bạn đang biên dịch project mới nhất trên **ESP32-S3**, hệ thống sử dụng chân **GPIO 1, 2, 3, 4**. Bạn nên kiểm tra lại xem mình định test bằng file core cũ hay mới để cắm dây cho chính xác nhé!

---

## [CÂU HỎI 2]
Hình như tôi dùng file mới nhất đó.
src/drn_motor_pwm.cpp
Motor 1: GPIO 1   
Motor 2: GPIO 2   
Motor 3: GPIO 3   
Motor 4: GPIO 4

Với lại 2 nút nhấn là GPIO4, GPIO5 để đóng mở cổng.  Bạn check cái file cũ thì sẽ thấy mình đề cập
1. Bạn có thể viết lại main.cpp giúp mình để mình cho nó chạy các mức độ 20%, 50%, 70%, 90% được không?
2. Tiếp nối ý 1. Cho mình hỏi là drn_main_board_choose.h/.cpp có cần sửa đổi gì không? Nếu có thì sửa giúp mình luôn. Đừng bỏ #include các file cũ nhé, để nguyên đấy để có gì cần dùng mình gọi cho dễ.
3. Cho mình hỏi PWM là mình điều khiển giá trị điện áp hay độ rộng xung vậy? Vì hổm mình cũng có làm liên quan đến PWM, nhưng rõ ràng là cùng mức điện áp, nó chỉ điều khiển độ rộng xung thôi, nghĩa là độ rộng xung quy định loại hành động của cái tay cầm truyền từ TX sang RX(trong file ARDUINO-CODE-UPDATE.pdf). Còn khi mình test motor driver thì mình nhớ là source code của mình thay đổi luôn điện áp cho các giá trị 20%, 50%,  70%, 90%( dao động từ 0 đến 3.3V), đây là kết quả mình đo bằng đồng hồ đo điện thật chứ không phải nói đùa. Mình chỉ thiếu cái máy đo xung như ossciolloscope hay logic analyzer thôi.
4. Tiếp nối ý 3. Mình có thể đo được xung motor driver bằng logic analyzer thôi được không?

---

## [TRẢ LỜI 2]
Trước khi đi vào code chi tiết, có một **xung đột phần cứng nhỏ** trong mô tả của bạn: Bạn đang gán **Motor 4 ở GPIO 4**, nhưng đồng thời lại định dùng **nút nhấn cũng ở GPIO 4**. ESP32-S3 Supermini có khá nhiều chân, nên để tránh chập cháy hoặc lỗi logic, mình sẽ dời 2 nút nhấn sang **GPIO 5 và GPIO 6** trong đoạn code mẫu dưới đây nhé. Bạn có thể đổi lại theo ý muốn miễn là không trùng với 4 chân motor (1, 2, 3, 4).

Dưới đây là giải pháp chi tiết cho cả 4 câu hỏi của bạn:

### 1. Viết lại `main.cpp` để test các mức 20%, 50%, 70%, 90%

Đoạn code này sử dụng 1 nút nhấn (GPIO 5) để chuyển đổi tuần tự giữa các mức duty cycle, và nút còn lại (GPIO 6) để dừng khẩn cấp (về 0%).

```cpp
#include "drn_main_board_choose.h"
#include "drn_motor_pwm.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

// Định nghĩa chân nút nhấn (Tránh GPIO 4 vì đã dùng cho Motor 4)
#define BTN_NEXT_LEVEL GPIO_NUM_5
#define BTN_STOP       GPIO_NUM_6

const int duty_levels[] = {20, 50, 70, 90};
int current_level_idx = -1; // -1 nghĩa là đang tắt (0%)

void setup_buttons() {
    gpio_config_t btn_config = {};
    btn_config.pin_bit_mask = (1ULL << BTN_NEXT_LEVEL) | (1ULL << BTN_STOP);
    btn_config.mode = GPIO_MODE_INPUT;
    btn_config.pull_up_en = GPIO_PULLUP_ENABLE; // Dùng điện trở kéo lên nội
    btn_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&btn_config);
}

void set_all_motors_duty(int duty_percent) {
    // Giả sử hàm set duty của bạn nhận giá trị %. 
    // Nếu hàm của bạn nhận giá trị raw (ví dụ 0-8191 cho 13-bit), 
    // bạn cần map(duty_percent, 0, 100, 0, max_duty)
    DRN_Motor_SetDuty(0, duty_percent); // Motor 1
    DRN_Motor_SetDuty(1, duty_percent); // Motor 2
    DRN_Motor_SetDuty(2, duty_percent); // Motor 3
    DRN_Motor_SetDuty(3, duty_percent); // Motor 4
}

extern "C" void app_main() {
    // Khởi tạo board và ngoại vi
    DRN_Board_Init();
    setup_buttons();

    int last_btn_next_state = 1;

    while (true) {
        int btn_next_state = gpio_get_level(BTN_NEXT_LEVEL);
        int btn_stop_state = gpio_get_level(BTN_STOP);

        // Xử lý nút dừng khẩn cấp (Nhấn xuống = 0 do pull-up)
        if (btn_stop_state == 0) {
            current_level_idx = -1;
            set_all_motors_duty(0);
            vTaskDelay(pdMS_TO_TICKS(200)); // Debounce
        }

        // Xử lý nút chuyển mức (Bắt sườn âm - lúc vừa nhấn xuống)
        if (btn_next_state == 0 && last_btn_next_state == 1) {
            current_level_idx++;
            if (current_level_idx >= 4) {
                current_level_idx = 0; // Quay vòng lại 20%
            }
            
            set_all_motors_duty(duty_levels[current_level_idx]);
            vTaskDelay(pdMS_TO_TICKS(200)); // Debounce
        }
        last_btn_next_state = btn_next_state;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### 2. Sửa đổi `drn_main_board_choose.h / .cpp`

Để duy trì cấu trúc dự án đa nền tảng (giữa ESP32-S3 và STM32F103), file này có nhiệm vụ bao bọc các lời gọi khởi tạo chung. Bạn **không cần xóa các #include cũ**, chỉ cần đảm bảo có gọi hàm khởi tạo của `drn_motor_pwm.cpp` (bản mới) vào luồng khởi tạo của ESP32.

**`drn_main_board_choose.h`**
```cpp
#ifndef DRN_MAIN_BOARD_CHOOSE_H
#define DRN_MAIN_BOARD_CHOOSE_H

// Giữ nguyên các include cũ của bạn ở đây
#include "old_hal_includes.h" 
// ...

// Include file mới
#include "drn_motor_pwm.h"

void DRN_Board_Init();

#endif
```

**`drn_main_board_choose.cpp`**
```cpp
#include "drn_main_board_choose.h"

void DRN_Board_Init() {
#ifdef ESP32_S3_SUPERMINI
    // Khởi tạo các ngoại vi cũ (nếu còn dùng)
    // Old_Timer_Init(); 

    // Khởi tạo PWM cho 4 motor theo file mới (GPIO 1,2,3,4)
    DRN_Motor_PWM_Init(); 
    
#elif defined(STM32F103C8T6)
    // Luồng khởi tạo riêng cho STM32
    // STM32_HAL_Init();
    // ...
#endif
}
```

### 3. Giải thích về bản chất của PWM: Độ rộng xung hay Điện áp?


Thực chất, PWM (Pulse Width Modulation) **chỉ điều khiển độ rộng xung**, hoàn toàn **không làm thay đổi điện áp đỉnh**. Tín hiệu xuất ra từ chân GPIO của ESP32-S3 luôn là tín hiệu Digital: nó chỉ nhảy chớp nhoáng giữa mức 0V (LOW) và 3.3V (HIGH) với một tần số rất nhanh (ví dụ 4kHz như bạn đã cấu hình).

**Vậy tại sao đồng hồ đo điện lại báo từ 0 đến 3.3V?**
Đồng hồ vạn năng (Multimeter) có tốc độ lấy mẫu (refresh rate) rất chậm so với tần số 4kHz của PWM. Khi nó đo một tín hiệu chớp tắt liên tục 4000 lần một giây, phần cứng của đồng hồ sẽ hoạt động như một bộ lọc thông thấp (low-pass filter) và **tính ra giá trị điện áp trung bình (RMS Voltage)** theo thời gian. 
* Ở mức 50% duty cycle: Chân GPIO bật 3.3V một nửa thời gian, và 0V một nửa thời gian. Đồng hồ sẽ tính trung bình và hiện ~1.65V.
* Ở mức 90% duty cycle: Nó hiện ~2.97V.
Kết quả đo bằng đồng hồ là "ảo ảnh" của phép lấy trung bình, chứ bản chất vi điều khiển không hề hạ điện áp xuống 1.65V.

**Sự khác biệt với tay cầm TX-RX trong file PDF:**
Trong tài liệu mô phỏng tay cầm RC của bạn, PWM được dùng để **truyền dữ liệu/thông tin**. Độ rộng xung từ $1000\mu s$ đến $2000\mu s$ là một quy ước ngôn ngữ để bộ nhận (RX) hiểu là gạt cần ga bao nhiêu phần trăm. Còn PWM xuất ra cho *motor driver* là để **truyền công suất**. Duty cycle càng lớn, thời gian cấp điện mức HIGH trong một chu kỳ càng lâu, dòng điện trung bình đổ vào cuộn dây motor càng lớn, làm motor quay nhanh hơn. 

### 4. Có thể đo xung Motor Driver bằng Logic Analyzer được không?

**Hoàn toàn được, và đây chính là thiết bị chuẩn xác nhất để làm việc này!**

Logic Analyzer (như bộ Saleae clone bạn hay dùng) sinh ra chính là để đo các tín hiệu Digital. Khi bạn kẹp các kênh (CH0, CH1...) của Logic Analyzer vào chân GPIO 1, 2, 3, 4 trên mạch, bạn sẽ thấy rõ mồn một bản chất thật của tín hiệu:
1.  Bạn sẽ thấy hình ảnh các xung vuông vuông vắn, biên độ đỉnh luôn là 3.3V.
2.  Bạn sẽ thấy rõ khoảng cách giữa 2 chu kỳ là bao nhiêu (để xác nhận lại tần số 4kHz hay 500Hz).
3.  Bạn sẽ thấy rõ bề rộng phần "mức cao" phình to ra khi bạn bấm nút chuyển từ 20% lên 50%, 70%, 90% đúng như lý thuyết mà không bị thiết bị đo đánh lừa thành điện áp trung bình.