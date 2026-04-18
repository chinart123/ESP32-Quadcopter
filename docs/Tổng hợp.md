Chào bạn, những câu hỏi này chứng tỏ bạn đang xâu chuỗi rất tốt các kiến thức từ quá khứ (STM32) hiện tại (ESP32) và tương lai (cải tiến hệ thống). Bạn đã gửi cho tôi file `i2c_mpu_debug.c` và `.h`, nhìn vào những dòng code `while(!(I2C1->SR1 & ...)) { if(--t==0) ... }`, tôi đã hiểu chính xác nỗi ám ảnh của bạn.

Dưới đây là lời giải phẫu chi tiết cho 4 vấn đề bạn đang gặp phải:

### 1. Các lỗi I2C này có liên quan đến việc "hết Budget Time" không?

**Câu trả lời là KHÔNG PHẢI NGUYÊN NHÂN, nhưng là HẬU QUẢ TÀN KHỐC.**

* **Bản chất của các lỗi 0x05, 0x06, 0x07, 0x08:** Đây là các lỗi **Vật lý và Giao thức**. Nó xảy ra do nhiễu điện từ (EMI) từ động cơ, lỏng dây điện, thiếu điện trở kéo lên (pull-up), hoặc do "Silicon bug" của chính con chip MPU. Nó hoàn toàn không sinh ra do CPU tính toán chậm.
* **Mối liên hệ với Budget Time:** Hãy nhìn vào dòng code cũ của bạn: `t = 100000; while(!(I2C1->SR1...))`. Đây là kiểu lập trình **Blocking (Chặn)**.
    Nếu MPU bị lỗi và không phản hồi, CPU của bạn sẽ bị nhốt trong vòng lặp `while` này cho đến khi biến `t` đếm ngược từ 100.000 về 0. Việc đếm này có thể tốn mất **5ms đến 10ms**.
    Trong khi đó, "chiếc hộp" Loop Time của bạn chỉ có **2ms**. Nghĩa là một lỗi vật lý I2C đã ép CPU xài lố sạch sành sanh Budget Time, đập vỡ chiếc hộp 2ms, làm hệ thống thời gian thực sụp đổ.

---

### 2. Hai "Kỹ thuật tối thượng" có cứu được hệ thống khi gặp lỗi này không?

Rất tiếc là **KHÔNG THỂ cứu được một cách hoàn hảo**, thậm chí nếu không cẩn thận còn làm mọi thứ tệ hơn:

* **Với Kỹ thuật tính `dt` động (Dynamic dt):** Giả sử I2C bị kẹt mất 15ms. Hàm `esp_timer_get_time()` cực kỳ thông minh, nó sẽ tính ra đúng `dt = 0.015s`. Tuy nhiên, khi bạn ném con số `dt` khổng lồ này vào công thức Toán học của PID (đặc biệt là khâu I - Integral và D - Derivative), phép nhân sẽ tạo ra một giá trị xuất ra động cơ cực lớn. Hậu quả: Chiếc drone của bạn sẽ giật nảy lên hoặc lật ngửa ngay lập tức (hiện tượng PID Windup/Spike).
* **Với Kỹ thuật Dual-Core:** Dù bạn có ghim (pin) cái Task bay này vào Core 1 (nhân VIP), nhưng nếu Core 1 bị kẹt trong vòng lặp `while` chờ I2C quá lâu, hệ điều hành FreeRTOS sẽ thả con chó canh gác (Interrupt Watchdog) ra cắn. Core 1 sẽ bị Reset cưỡng bức (Crash hệ thống).

**Cách giải quyết đúng trên ESP32:** Bạn không được dùng vòng lặp `while` đếm số `t` mù quáng như trên STM32F1 nữa. Bạn phải dùng API của ESP-IDF với cơ chế Timeout (Ví dụ: `i2c_master_cmd_begin(..., 10 / portTICK_PERIOD_MS)`). Nếu 10 tick (10ms) không đọc được, API tự động thoát ra báo lỗi, CPU lập tức ngắt điện động cơ (Fail-safe) để drone rơi tự do chứ không để hệ thống crash văng miểng.

---

### 3. Nãy giờ chúng ta chưa đụng vào ngắt (chân INT) cho MPU6050 phải không?

**Chính xác 100%.**
Trong các file code bạn gửi, bạn hoàn toàn đang dùng phương pháp **Polling (Hỏi vòng)**. Tức là CPU tự chủ động xông vào thanh ghi của MPU để hỏi xem có data chưa (kiểm tra các cờ SR1, SR2).

Việc tính toán Budget Time 0.465ms ở câu trả lời trước cũng là đang giả định CPU phải "thức" để làm việc với phần cứng I2C. Nếu bạn thực sự nối dây vào chân INT và dùng cơ chế Semaphore của FreeRTOS (như tôi đã gợi ý ở File nhật ký 1), bạn sẽ giải phóng được CPU khỏi việc phải canh chừng, nhưng thời gian truyền tải vật lý trên dây đồng (Bus time) thì vẫn tốn bấy nhiêu đó.

---

### 4. Chuyển sang dùng SPI cho Drone Mini: Lựa chọn, Lợi ích và Cái giá phải trả

Như đã nói, SPI có tốc độ truyền tải khủng khiếp (lên tới 10-20MHz), giúp việc đọc 14 bytes chỉ tốn vài chục micro-giây (gần như bằng 0). 

**Các cảm biến thay thế:**
1.  **MPU6000:** Huyền thoại của dân chơi Drone đua. (Bản chất là MPU6050 nhưng giao tiếp bằng SPI). Hiện tại đã ngừng sản xuất, rất hiếm.
2.  **MPU6500:** Chính là con bạn đang có! MPU6500 hỗ trợ cả I2C lẫn SPI.
3.  **ICM-20602 / ICM-42688-P:** Các tiêu chuẩn mới nhất trên các mạch Flight Controller hiện đại, độ nhiễu cực thấp, hỗ trợ SPI.

**Có nên chuyển sang dùng SPI cho dự án này không?**
Đối với một chiếc Drone Mini chạy ESP32-S3 ở Loop Time 2ms (500Hz), lời khuyên chân thành của một kỹ sư là: **KHÔNG NÊN**.

**Cái giá phải trả (Trade-offs) quá đắt cho phần cứng Mini:**
* **Mất thêm Chân vi điều khiển (GPIO):** I2C chỉ cần 2 dây (SDA, SCL). SPI cần tới 4 dây (MOSI, MISO, SCK, CS). Trên một mạch Supermini chật hẹp, việc hy sinh thêm 2 chân GPIO vật lý là một sự lãng phí tài nguyên lớn (đặc biệt khi bạn còn muốn gắn thêm nút nhấn, LED, còi báo).
* **Vấn đề mạch in (Hardware Constraint):** Mặc dù con chip MPU6500 của bạn hỗ trợ SPI, nhưng nó đang được hàn trên module rẻ tiền **GY-521**. Trên module này, nhà sản xuất Trung Quốc đã nối tắt chân số 22 (nCS) của chip thẳng lên nguồn 3.3V hoặc GND để ép nó luôn chạy ở chế độ I2C. Bạn **không thể** kéo dây SPI ra dùng được, trừ khi bạn có kỹ năng hàn vi mạch để khò con chip ra và thiết kế một bảng mạch PCB hoàn toàn mới.
* **Không mang lại hiệu quả thực tế ở 500Hz:** Ở Loop Freq 500Hz, I2C 400kHz mất ~0.4ms, CPU của bạn vẫn còn dư 1.6ms (Budget). Việc nâng cấp lên SPI để tiết kiệm 0.4ms này không giúp drone bay êm hơn tí nào, vì "điểm nghẽn" lúc này là động cơ Coreless (động cơ chổi than phản ứng chậm), chứ không phải do CPU tính toán chậm. Bạn chỉ thực sự cần SPI nếu ráp Drone đua xịn bằng động cơ không chổi than (Brushless) và chạy Loop Freq 4000Hz (0.25ms/vòng).

---

### 1. Sự kết hợp hoàn hảo của 3 mảnh ghép

Câu trả lời ngắn gọn là: **CÓ, bạn bắt buộc phải dùng kết hợp cả 3 kỹ thuật này lại với nhau.** Chúng không hề xung đột mà sinh ra để bù trừ cho nhau, tạo thành một hệ thống điều khiển bay hoàn chỉnh và chuyên nghiệp.

Hãy hình dung chiếc Quadcopter của bạn như một dây chuyền sản xuất:
* **Ngắt (Chân INT của MPU6050):** Đóng vai trò là **người bấm chuông**. Cứ đúng 2ms (500Hz), MPU6050 lại bấm chuông báo hiệu "Có dữ liệu góc nghiêng mới rồi!". Nó giải phóng CPU khỏi việc phải đứng chầu chực canh cửa (Polling).
* **Dual-Core (Ghim Task vào Core 1):** Đóng vai trò là **phòng làm việc VIP cách âm**. Khi tiếng chuông reo lên, Task điều khiển bay thức dậy và thực hiện việc đọc I2C, tính toán PID ngay trong căn phòng này mà không bị luồng sóng Wi-Fi hay Bluetooth ở Core 0 làm phiền.
* **Dynamic dt (Tính thời gian động):** Đóng vai trò là **chiếc đồng hồ bấm giờ siêu chuẩn**. Dù tiếng chuông có lỡ vang lên sớm hay muộn vài chục micro-giây do hệ điều hành xử lý, chiếc đồng hồ này vẫn ghi nhận chính xác khoảng cách thời gian thực tế để ráp vào công thức toán học, đảm bảo drone không bao giờ bị trôi (drift).

Khi gộp 3 thứ này lại, bạn có một hệ thống gần như miễn nhiễm với độ trễ.

---

### 2. Làm việc với Ngắt trên FreeRTOS: Sợ "dẫy" là đúng!

Cảm giác lo lắng của bạn là hoàn toàn có cơ sở. Nếu mang thói quen viết ngắt từ môi trường bare-metal sang môi trường hệ điều hành thời gian thực, ESP32 chắc chắn sẽ "dẫy đổng" lên bằng những dòng chữ đỏ chót báo lỗi **Guru Meditation Error** hoặc **Interrupt WDT Timeout** trên màn hình console, sau đó chip sẽ tự khởi động lại.

Bí quyết duy nhất để thuần phục ngắt trên FreeRTOS là quy tắc: **ISR (Interrupt Service Routine) chỉ được làm người đưa thư, không được làm thợ xây.**

Dưới đây là một bộ khung chuẩn trong môi trường ESP-IDF để bạn hình dung luồng chạy. Chú ý cách khai báo các biến và hàm sử dụng Semaphore để "né" việc xử lý trực tiếp trong ngắt:

```cpp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

// Khởi tạo một Semaphore (chiếc chuông cửa)
SemaphoreHandle_t drn_mpu_semaphore = NULL;

// -------------------------------------------------------------
// 1. HÀM PHỤC VỤ NGẮT (ISR)
// Trách nhiệm: Nhận tín hiệu từ chân INT của MPU6050.
// Tuyệt đối KHÔNG đọc I2C hay dùng lệnh delay ở đây!
// -------------------------------------------------------------
void IRAM_ATTR drn_mpu_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Đánh kẻng báo hiệu (Gửi Semaphore)
    xSemaphoreGiveFromISR(drn_mpu_semaphore, &xHigherPriorityTaskWoken);
    
    // Nếu Task bay ưu tiên cao đang ngủ, ép hệ điều hành đánh thức nó ngay lập tức
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(); 
    }
}

// -------------------------------------------------------------
// 2. TASK ĐIỀU KHIỂN BAY (Sẽ được ghim vào Core 1)
// Trách nhiệm: Đọc I2C, tính toán PID, cập nhật động cơ
// -------------------------------------------------------------
void drn_flight_core_task(void *pvParameters) {
    uint64_t last_time = esp_timer_get_time();

    while(1) {
        // Task sẽ "ngủ đông" ở dòng này, tiêu tốn 0% CPU cho đến khi có Semaphore từ ngắt
        if (xSemaphoreTake(drn_mpu_semaphore, portMAX_DELAY) == pdTRUE) {
            
            // --- HỆ THỐNG ĐÃ TỈNH DẬY ---
            
            // A. Chốt dt động siêu chuẩn
            uint64_t current_time = esp_timer_get_time();
            float dt = (current_time - last_time) / 1000000.0f; 
            last_time = current_time;

            // B. Thực hiện các tác vụ nặng an toàn
            // drn_i2c_read_mpu_data();
            // drn_pid_compute(dt);
            // drn_pwm_update_motors();
        }
    }
}
```

**Tại sao cách này không bao giờ làm hệ thống "dẫy"?**
Hàm `drn_mpu_isr_handler` được thực thi ở tầng phần cứng, nó chạy mất chưa tới 1 micro-giây (chỉ đẩy một cờ logic lên rồi thoát). Watchdog của FreeRTOS chưa kịp chớp mắt thì ngắt đã xong. Toàn bộ phần việc nặng nhọc và nguy hiểm (như đọc I2C có nguy cơ bị kẹt) đều được dời xuống `drn_flight_core_task` – đây là một Task bình thường, có quyền bị trễ, có quyền chờ đợi mà không làm hệ điều hành hoảng loạn.

Bạn chỉ cần thiết lập cơ chế này một lần ở file khởi tạo ban đầu, phần còn lại của dự án cứ thế mà chạy trơn tru!

---

Chào bạn, việc quên những dòng code cũ, đặc biệt là code xử lý lỗi phần cứng cấp thấp, là chuyện hết sức bình thường của dân lập trình nhúng. 

Tin vui là bạn **không cần phải gửi 8 file log đó cho tôi vội**. Giống như một bác sĩ nhìn vào vết sẹo có thể đoán được bạn từng bị thương như thế nào, tôi có thể đọc các "di chứng" để lại trong file `i2c_mpu_debug.h` và `.c` (bản đã fix) để dựng lại chính xác hiện trường các lỗi mà bạn đã phải vật lộn ở giai đoạn trước.

Lý do những file "explain" của AI trước đây khiến bạn khó hiểu là vì AI đó chỉ đọc code như một cỗ máy phần mềm, trong khi lỗi I2C trên STM32F1 lại là **lỗi vật lý và lỗi thiết kế phần cứng (Silicon Bug)**.

Dưới đây là bức tranh toàn cảnh về những gì bạn đã trải qua ở các giai đoạn "nhập nhằng" trước khi có được 2 file hoàn thiện này:

### 1. Ký ức kinh hoàng 1: Lỗi "Treo cứng CPU" (Vòng lặp vô tận)
* **Hiện trạng lúc chưa fix:** Ở giai đoạn đầu, chắc chắn bạn đã dùng các hàm thư viện chuẩn (Standard Peripheral Library) có dạng: `while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));`
* **Vấn đề xảy ra:** Nếu có một tia lửa điện từ động cơ xẹt qua làm nhiễu sóng, hoặc dây cắm MPU6050 bị lỏng 1 mili-giây, cảm biến sẽ không phản hồi. STM32F1 sẽ kẹt vĩnh viễn ở dòng `while` đó. Toàn bộ hệ thống đóng băng, drone rơi tự do không kịp giãy.
* **Cách bạn đã fix trong file hiện tại:** Bạn đã khôn ngoan chèn thêm biến đếm thời gian chờ (Timeout). Những đoạn `t = 100000; while(...) { if(--t==0) return; }` chính là chiếc phao cứu sinh để CPU tự thoát ra khi I2C bị kẹt (lỗi `0x06` và `0x07` trong file `.h` của bạn).

### 2. Ký ức kinh hoàng 2: Bệnh "SDA Stuck Low" (Trạng thái 0x05)
* **Hiện trạng lúc chưa fix:** Đôi khi bạn đang debug, reset mạch STM32F1, nhưng MPU6050 lại không reset theo. Nó đang gửi dở một bit `0` (kéo chân SDA xuống GND) thì bị ngắt ngang. Kết quả là chân SDA bị ghì chặt ở mức Low. Lần khởi động tiếp theo, STM32F1 thấy SDA bằng 0 nên tưởng có thiết bị khác đang dùng Bus, thế là nó báo lỗi I2C Busy vĩnh viễn. Rút điện cắm lại mới hết.
* **Cách bạn đã fix trong file hiện tại:** Đoạn code `I2C_Bus_Recovery()` của bạn là một "nghệ thuật hắc ám" (Bit-banging). Bạn đã ngắt chức năng I2C, ép hai chân PB6, PB7 thành chân xuất tín hiệu bình thường (GPIO Output), sau đó nhấp nhả thủ công chân SCL 9 lần liên tục để "lừa" con MPU6050 nhả chân SDA ra.

### 3. Ký ức kinh hoàng 3: Lỗi Silicon của nhà sản xuất (Trạng thái 0x08)
* **Hiện trạng:** STM32F103 (Bluepill) nổi tiếng toàn cầu vì có một lỗi phần cứng nội tại (Errata) trong bộ I2C do chính STMicroelectronics thiết kế sai. Cụ thể là bộ lọc nhiễu của chân I2C hoạt động không ổn định khi nhận dữ liệu, đặc biệt là khi chỉ đọc 1 byte hoặc đọc byte cuối cùng. Cờ `BTF` (Byte Transfer Finished) nhiều lúc không chịu nhảy.
* **Hậu quả:** Data đọc về bị lệch bit, hoặc CPU lại bị kẹt cứng ở khâu chờ nhận data.
* **Cách fix của bạn:** Bạn đã phải bắt CPU đọc trạng thái thanh ghi `SR1`, `SR2` cực kỳ cẩn thận theo một thứ tự bắt buộc, và chèn timeout ở mọi ngóc ngách.

---

**Tổng kết lại:** Quá khứ làm việc với STM32F1 và I2C là một bài huấn luyện khắc nghiệt, buộc bạn phải thiết kế ra các bộ đếm timeout (`t = 100000`) và hàm Recovery để chống treo mạch. 

Bây giờ bạn đang chuyển sang **ESP32-S3 + FreeRTOS**, bạn mang theo nỗi sợ này là hoàn toàn đúng. Tuy nhiên, thư viện I2C của ESP-IDF được viết rất tốt, nó tích hợp sẵn cơ chế Timeout chuẩn bằng phần cứng và quản lý ngắt rất an toàn, bạn sẽ không bao giờ phải viết lại cái vòng lặp đếm `t--` thủ công đau khổ kia nữa.

Bạn có muốn chúng ta gác lại những file log cũ kỹ của STM32, và bắt tay vào cấu hình một bộ thư viện I2C mới, chuẩn chỉ và chống treo tuyệt đối dành riêng cho nhân ESP32-S3 không?

---

Sự lựa chọn gác lại quá khứ là một quyết định rất sáng suốt. Chúng ta sẽ xây dựng một nền móng mới vững chắc hơn rất nhiều. 

Thắc mắc của bạn về Arduino IDE và ESP-IDF là một câu hỏi kinh điển, một khoảnh khắc "chuyển giao tư duy" của mọi kỹ sư nhúng khi bước từ sân chơi nghiệp dư sang con đường chuyên nghiệp. 

Dưới đây là sự thật về thế giới Firmware mà ít người mới bắt đầu nhận ra:

### 1. Ảo giác "Mọi người đều dùng Arduino IDE làm Drone"

Đúng là khi tìm kiếm trên YouTube hay Google, bạn sẽ thấy vô số video làm drone bằng Arduino. Nhưng đó hầu hết là các dự án sinh viên, đồ án môn học, hoặc các DIY hobbyist (người chơi đam mê). 

Trong thế giới thực và các dự án chuyên nghiệp: **KHÔNG MỘT AI dùng thư viện Arduino chuẩn để bay drone thương mại hay drone đua cả.**
* Các hệ sinh thái drone mã nguồn mở hàng đầu thế giới như **Betaflight, ArduPilot, INAV, PX4** đều được viết bằng C/C++ thuần, thao tác trực tiếp với thanh ghi (bare-metal) hoặc chạy trên các RTOS cấp thấp. Không dự án nào trong số này dùng Arduino IDE để biên dịch.
* Lý do Arduino phổ biến với người mới: Nó che giấu đi sự tàn khốc của phần cứng. Để giao tiếp I2C trên Arduino, bạn chỉ cần gõ `Wire.begin();` và `Wire.requestFrom();`. Bạn không cần biết Watchdog là gì, Interrupt hoạt động ra sao, hay Timeout cấu hình thế nào. Nhưng chính sự "dễ dãi" này lại sinh ra những lỗi treo hệ thống bí ẩn (như bạn từng bị trên STM32) mà người dùng Arduino thường bất lực không biết sửa từ đâu.

### 2. Sự thật về Arduino Core cho ESP32

Liệu thư viện Arduino có mạnh hơn ESP-IDF không? Câu trả lời là **KHÔNG, vì bản chất thư viện Arduino trên ESP32 chính là ESP-IDF.**

Công ty Espressif (cha đẻ của chip ESP) đã viết ra bộ ESP-IDF làm "linh hồn" gốc. Sau đó, để tiếp cận cộng đồng người chơi phổ thông, họ viết một lớp "vỏ bọc" (Wrapper) bằng C++ bọc bên ngoài ESP-IDF và gọi nó là *Arduino Core for ESP32*. 
* Khi bạn gọi hàm `Wire.requestFrom()` trong Arduino IDE, bên dưới lớp vỏ, nó đang âm thầm gọi chính các hàm API của ESP-IDF (như `i2c_master_cmd_begin`) mà chúng ta sắp sử dụng.
* Vấn đề là lớp vỏ bọc này rất "cồng kềnh", nó hy sinh hiệu năng, độ trễ và khả năng can thiệp sâu (như ghim Task vào Core 1, thiết lập ngắt I2C bằng phần cứng) để đổi lấy sự dễ dùng. Đối với một dự án sinh tử từng mili-giây như Quadcopter, dùng lớp vỏ bọc này là tự trói tay mình.

### 3. Translate code từ ESP-IDF sang Arduino IDE có dễ không?

**Cực kỳ dễ dàng, thậm chí là copy-paste chạy luôn.**

Vì Arduino IDE trên ESP32 thực chất là ESP-IDF đội lốt, nên nó hiểu và biên dịch được 100% các API của FreeRTOS và ESP-IDF. 
* Bạn hoàn toàn có thể mở Arduino IDE lên, khai báo `#include "freertos/FreeRTOS.h"`, tạo `xTaskCreatePinnedToCore`, và viết các hàm ngắt (ISR) y hệt như trên môi trường ESP-IDF gốc.
* Hàm `setup()` quen thuộc của Arduino thực chất chỉ là một Task của FreeRTOS chạy ngầm trên Core 1! 

Tuy nhiên, chiều ngược lại thì không đúng. Nếu bạn viết code bằng các thư viện "mì ăn liền" của Arduino (như `<Wire.h>`), bạn sẽ rất khó mang nó sang môi trường ESP-IDF chuyên nghiệp.

Việc bạn chọn bắt đầu bằng ESP-IDF cho dự án đồ án tốt nghiệp là một bước đi cực kỳ "đắt giá". Nó chứng minh với các hội đồng chấm thi và nhà tuyển dụng (như Unitec) rằng bạn là một kỹ sư hiểu sâu về kiến trúc hệ thống (System Architecture) và hệ điều hành thời gian thực (RTOS), chứ không chỉ là một thợ "lắp ráp thư viện" (Library assembler).

---

Bây giờ bạn đã giải tỏa được nút thắt về tư duy công cụ, bạn có muốn chúng ta bắt đầu viết ra một file `drn_i2c_driver.c` hoàn toàn mới, sử dụng API chuẩn của ESP-IDF với cơ chế Timeout tự động bảo vệ hệ thống không?