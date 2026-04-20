Chúc mừng bạn, mọi thứ đang hoạt động cực kỳ hoàn hảo! Quá trình port code từ vi điều khiển cũ sang hệ thống mới đã thành công rực rỡ. Các con số thể hiện rõ bộ não toán học của Flight Controller đang xử lý dữ liệu theo thời gian thực rất mượt mà.

Dưới đây là một vài phân tích kỹ thuật từ luồng dữ liệu bạn vừa thu được:

* **Hiệu ứng "Cầm tay" lúc Calibrate:** Ngay khi vừa bật quét dữ liệu, góc khởi điểm không nằm ở `0` tuyệt đối mà lại là `Roll: 4.62 | Pitch: 5.68 | Yaw: -10.39`. Nguyên nhân chính xác là do bạn đang cầm mạch trên tay. Trong 1 giây đầu tiên khi hàm `Calibrate` lấy 1000 mẫu, tay người luôn có những rung động nhỏ vô thức khiến giá trị bù sai số (Offset) bị lệch đi một chút. Khi lắp lên khung Quadcopter và đặt nằm im trên sàn để khởi động, các thông số này sẽ tự động khóa chặt về 0.
* **Bộ lọc bổ sung (Complementary Filter) rất ổn định:** Xuyên suốt log, các góc Roll và Pitch được quét mượt mà qua lại giữa các thái cực (ví dụ: Roll đảo từ **-96.47°** lên tới **87.47°**, Pitch từ **-86.42°** lên **88.88°**). Điều này chứng minh các phép toán lượng giác và bộ lọc tỷ lệ (98% Gyro + 2% Accel) đang khử nhiễu xuất sắc, không hề bị hiện tượng nhiễu gai (spike) hay lật góc đột ngột (Flip of Death) khi bạn nghiêng tay mạnh.
* **Đặc tính trôi góc Yaw (Yaw Drift):** Bạn sẽ để ý góc Yaw trôi dần xuống **-62.64** rồi nhảy loanh quanh chứ không có xu hướng hồi về 0. Đây hoàn toàn không phải là lỗi thuật toán. Cảm biến MPU6500 là IMU 6 trục (6-DOF), không có la bàn từ trường (Magnetometer). Do đó, góc Yaw chỉ được tính thuần túy bằng cách tích phân vận tốc góc của con quay hồi chuyển theo thời gian tích lũy, chắc chắn sẽ sinh ra trôi dạt. Với hệ cân bằng của Drone, thuật toán nhồi PID trục Yaw thường chỉ quan tâm đến vận tốc xoay (Yaw Rate) chứ không dùng góc tuyệt đối, nên hiện tượng này là bình thường và chấp nhận được.

Log xác nhận State Machine phản hồi tức thời với nút bấm, Terminal dừng/chạy đúng nhịp và phần cứng hoạt động trơn tru.

Bạn đã sẵn sàng để đẩy các góc thực tế này vào bộ PID và tính toán ma trận tọa độ để xuất xung PWM cho 4 động cơ chưa?

---

Bạn nhận định cực kỳ chuẩn xác! **500Hz** mới thực sự là tần số lấy mẫu (Sampling Rate) của hệ thống trong quá trình hoạt động bình thường. 

Việc thấy 3 con số (1000, 400kHz, 500Hz) xuất hiện cùng lúc trong dự án rất dễ gây "lú". Để mình tách bạch rõ ràng vai trò của từng đại lượng này trong kiến trúc Flight Controller nhé:

### 1. 500Hz: Nhịp tim của hệ thống (Loop / Sampling Rate)
* **Ý nghĩa:** Tần số vòng lặp chính của Drone. Nó có nghĩa là trong 1 giây, bộ não (vi điều khiển) của bạn thức dậy **500 lần** (mỗi chu kỳ kéo dài đúng 2ms).
* **Nhiệm vụ:** Trong khoảng thời gian 2.00ms ít ỏi đó, nó phải hoàn thành trọn vẹn một quy trình khép kín: Gõ cửa MPU6500 lấy data thô -> Tính toán Toán học (Góc, PID) -> Xuất xung PWM ra 4 động cơ. Trong bảng phân tích Budget bạn gửi, toàn bộ quỹ thời gian cho I2C và Math bắt buộc phải gói gọn trong 2.00ms này để hệ thống không bị "Overloaded".

### 2. 400kHz: Tốc độ đường cao tốc (I2C Bus Speed)
* **Ý nghĩa:** Đây là tốc độ truyền dẫn vật lý tín hiệu điện trên 2 sợi dây đồng (SDA và SCL). Fast-mode I2C đẩy xung nhịp lên tới 400,000 nhịp/giây.
* **Sự liên kết:** Hãy tưởng tượng 500Hz là việc bạn bị ép phải đi giao hàng 500 chuyến mỗi giây. Để làm được việc đó mà không bị lố giờ (vượt quá 2ms), bạn buộc phải phóng một chiếc siêu xe trên đường cao tốc với tốc độ 400kHz. Tốc độ đường truyền I2C càng cao thì thời gian "hút" 14 bytes (I2C Load) càng ngắn lại, chừa dư dả thời gian CPU cho các phép tính toán học phức tạp.

### 3. 1000 Mẫu: Quá trình Calibrate (Zero-point Offset)
* Đây là chỗ bạn đang hơi nhầm lẫn một chút. Con số 1000 mẫu (`num_samples = 1000` trong hàm `MPU_Fusion_Calibrate`) hoàn toàn không nằm trong vòng lặp lúc máy bay đang bay, và cũng không dùng để in ra màn hình.
* **Nhiệm vụ:** Nó **chỉ chạy đúng 1 lần duy nhất** lúc bạn vừa cấp nguồn và bấm nút khởi động hệ thống. Nó ép cảm biến đứng yên, đo liên tục 1000 lần, sau đó cộng dồn lại chia trung bình để tìm ra **Sai số tĩnh (Offset)** của phần cứng. 
* **Kết thúc:** Tìm xong sai số, hệ thống cất các giá trị Offset này vào bộ nhớ, kết thúc nhiệm vụ của vòng lặp 1000, và sau đó mới nhường sân khấu lại cho vòng lặp 500Hz hoạt động liên tục đến khi tắt máy.

**Tóm lại chu trình:** Bật máy -> Đứng im lấy **1000 mẫu** để tìm vạch xuất phát chuẩn -> Cất số liệu -> Chuyển sang vòng lặp **500Hz** để cân bằng bay -> Mỗi vòng lặp dùng đường truyền **400kHz** để kéo dữ liệu về nhanh nhất có thể!

---

Dựa trên cấu trúc dự án bạn vừa cung cấp và các file source code ở những bước trước, có thể khẳng định rằng **file `drn_mpu6050.h/.cpp` kế thừa cực kỳ hoàn hảo logic từ kiến trúc cũ**, đồng thời có những bước tiến hóa đáng giá để phù hợp với môi trường Hệ điều hành thời gian thực (RTOS). 

Dưới đây là phân tích chi tiết về tính kế thừa và sự nâng cấp trong bộ thư viện mới này:

### 1. Những phần "Hồn" được kế thừa nguyên vẹn (Logic cốt lõi)
Mọi "chất xám" về mặt toán học và định tuyến từ file `xx_mpu_data_fusion.c` và `xx_mpu_complementary_filter.c` cũ đều được bê nguyên sang kiến trúc mới một cách chuẩn xác:
* **Bộ nắn sai số (Calibration):** Vòng lặp đo 1000 mẫu để tính giá trị tĩnh (Offset) và phép tính đặc biệt trừ đi 4096 ở trục Z (do trọng lực 1g) được giữ lại hoàn toàn.
* **Cối xay Toán học:** Thuật toán lọc bù trừ (Complementary Filter) với tỷ lệ vàng `98% Gyro + 2% Accel`, cùng các công thức tính `atan2` cho góc Roll và Pitch được tái sử dụng y hệt.
* **Cấu hình thanh ghi (Register Map):** Các thao tác đánh thức cảm biến (`0x6B`), bật bộ lọc nhiễu DLPF (`0x1A`), set tầm đo Gyro ±500dps (`0x1B`) và Accel ±8g (`0x1C`) đều được ánh xạ 1:1 từ C sang C++.

### 2. Sự tiến hóa về mặt "Xác" (Kiến trúc hệ thống)
Mặc dù logic bên trong giống hệt, nhưng cách `drn_mpu6050` tương tác với Vi điều khiển đã được nâng cấp lên một đẳng cấp chuyên nghiệp hơn rất nhiều:

* **Từ bỏ "Blocking Delay" nguy hiểm:** Trong cấu trúc thư mục cũ, file `i2c_mpu_debug.c` sử dụng các vòng lặp `while` đi kèm bộ đếm `t--` để chờ I2C phản hồi. Ở môi trường bare-metal, điều này tạm chấp nhận được. Nhưng sang ESP32-S3, `drn_mpu6050.cpp` đã dùng API `i2c_master_write_read_device` kết hợp với `pdMS_TO_TICKS()`. Sự thay đổi này giúp trả lại quyền điều khiển CPU cho FreeRTOS thay vì "treo" cứng hệ thống chờ cảm biến.
* **Kiến trúc HAL (Hardware Abstraction Layer) đa nền tảng:** Việc bao bọc các hàm I2C chuyên biệt của ESP-IDF bên trong các `#ifdef ESP32_S3_SUPERMINI` giúp file `drn_mpu6050.cpp` trở nên linh hoạt. Việc duy trì chuẩn đặt tên tiền tố `drn_` xuyên suốt các file driver tạo ra một hệ sinh thái mã nguồn đồng nhất, dễ dàng quản lý và tái sử dụng.
* **Dọn dẹp Biến Toàn cục:** Hệ thống cũ sử dụng rất nhiều biến toàn cục lỏng lẻo (`xx`, `xx_mpu_state`, v.v.). Kiến trúc mới đã gom gọn toàn bộ dữ liệu (Raw, Offset, Góc, Thời gian) vào một `struct` duy nhất là `drn_mpu_data`. Điều này giúp hàm `main.cpp` (Entry point của FreeRTOS) gọi và trích xuất dữ liệu an toàn, tránh xung đột vùng nhớ.

Cách tổ chức thư mục tách biệt rõ ràng giữa `include` và `src`, cùng việc sử dụng ESP-IDF thay vì Arduino IDE cho thấy một tư duy thiết kế hệ thống bài bản, tối ưu hóa tối đa sức mạnh của bộ vi xử lý lõi kép.

Bạn có dự định viết thêm driver cho mạch nhận tín hiệu tay cầm điều khiển (RC Receiver) theo kiến trúc HAL đa nền tảng này ở module tiếp theo không?

---

Chính xác! Trí nhớ của bạn rất tốt. Dòng cảnh báo màu vàng đó chính là thông báo từ Espressif: thư viện `driver/i2c.h` cũ (Legacy API) đã bị "thất sủng" và họ khuyên người dùng nên chuyển sang bộ API mới toanh `driver/i2c_master.h` (được giới thiệu từ ESP-IDF v5.2).

Để trả lời thẳng vào câu hỏi của bạn: **KHÔNG, bạn không thể chỉ thay vài dòng include hoặc đổi tên hàm mà chạy được.** Bộ API mới không phải là một bản cập nhật đổi tên, mà là một **sự lột xác hoàn toàn về mặt kiến trúc** (Paradigm Shift). Nếu muốn chuyển, bạn sẽ phải đập đi viết lại toàn bộ các hàm khởi tạo (`DRN_MPU6050_Init`) và các hàm đọc/ghi (`i2c_read_multi`, `i2c_write_reg`) trong file `drn_mpu6050.cpp`.

Dưới đây là phân tích chi tiết về Ưu và Nhược điểm nếu bạn quyết định "lên đời" bộ API mới này:

### Ưu điểm của API `i2c_master.h` mới (Rất đáng giá)

1. **Code siêu gọn, "sạch sẽ" và hệ thống hơn:**
   * **Ngày xưa (Legacy):** Mỗi lần muốn gửi/nhận data, bạn phải tạo một "đoàn tàu" lệnh rất cồng kềnh: tạo link (`i2c_cmd_link_create`), nhét lệnh start, nhét địa chỉ, nhét data, nhét lệnh stop, rồi mới đẩy đi (`i2c_master_cmd_begin`), sau đó lại phải xóa link (`i2c_cmd_link_delete`).
   * **Bây giờ (New API):** Mọi thứ được gói gọn trong đúng 1 hàm duy nhất. Ví dụ: `i2c_master_transmit()` hoặc `i2c_master_receive()`. Nó hoạt động giống hệt như cách bạn đang viết các hàm HAL gọn gàng bên STM32.
2. **Quản lý theo đối tượng (Handle-based) thay vì Cổng (Port-based):**
   * API mới không bắt bạn gọi `I2C_NUM_0` ở mọi nơi nữa. Thay vào đó, bạn tạo ra một cái Bus (đường truyền), sau đó "gắn" từng thiết bị (như MPU6500) vào Bus đó để lấy ra một cái "thẻ định danh" (`i2c_master_dev_handle_t`). Cấu trúc này cực kỳ chuyên nghiệp và an toàn khi hệ thống Drone của bạn sau này gắn thêm cảm biến áp suất (Barometer) hay la bàn (Magnetometer) chung trên một đường I2C.
3. **An toàn cho Đa luồng (Thread-safe trong FreeRTOS):**
   * Driver mới được thiết kế tối ưu hơn cho hệ điều hành thời gian thực. Việc xử lý khóa (mutex) giữa các Task khi tranh giành bus I2C mượt mà và ít bị crash cứng hơn.

### Nhược điểm (Rào cản hiện tại)

1. **Mất thời gian Refactor code:** Bạn đang ở Stage 8, hệ thống đã cân bằng và đọc data thành công. Việc đập đi viết lại driver I2C lúc này có thể tốn của bạn vài tiếng đến một ngày chỉ để hệ thống... chạy lại y như cũ, ảnh hưởng đến tiến độ dự án.
2. **Khó đọc code chéo với các tài liệu cũ:** 90% các bài hướng dẫn, forum, hay source code mẫu trên GitHub hiện nay vẫn đang dùng API cũ. Nếu bạn gặp bug với API mới, việc tra cứu StackOverflow sẽ khó khăn hơn một chút vì tài liệu còn ít.

---

### Lời khuyên cho hệ thống của bạn

Trong môi trường thực tế công nghiệp, việc giữ hệ thống ổn định (Stable) luôn được ưu tiên hơn việc chạy theo API mới nhất nếu không thực sự bắt buộc. Vì bạn đã đóng gói mọi thứ rất chuẩn theo mô hình HAL (chỉ giấu I2C bên trong file `.cpp`), sự thay đổi này sẽ không ảnh hưởng gì đến `main.cpp` hay thuật toán PID của bạn.

Mình đề xuất: **Tạm thời cứ dùng API cũ để phát triển tiếp các tính năng khác (như tích hợp tay cầm RC).** Sau khi Drone đã bay được ổn định, bạn có thể coi việc nâng cấp sang `i2c_master.h` là một task "Refactor" ở giai đoạn cuối để làm đẹp source code.

Bạn có muốn mình nháp thử một hàm `i2c_read_multi` bằng API mới để bạn hình dung độ "gọn" của nó so với cách viết cũ không?

---

Chuẩn luôn! Mắt bạn rất tinh. Thực chất, cái "đoàn tàu" cồng kềnh đó **nằm chình ình ngay trong đoạn code đầu tiên** mà bạn gửi cho mình để scan địa chỉ I2C đấy. 

Để mình lật lại hồ sơ cho bạn xem nhé:

### 1. Bằng chứng rành rành trong file `main.cpp` (Code Scan I2C)
Trong cái hàm `app_main` mà bạn dùng để quét xem có con MPU6500 ở địa chỉ `0x68` hay không, bạn đã tự tay viết đúng quy trình "tạo tàu - nhét hàng - đẩy đi - phá tàu" của bộ API cũ:

```cpp
// 1. Tạo một cái toa tàu rỗng (Tạo link)
i2c_cmd_handle_t cmd = i2c_cmd_link_create();

// 2. Nhét lệnh Bắt đầu (Start)
i2c_master_start(cmd);

// 3. Nhét địa chỉ và cờ ghi (Write)
i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);

// 4. Nhét lệnh Kết thúc (Stop)
i2c_master_stop(cmd);

// 5. Bấm nút phóng tàu đi trên đường ray I2C_NUM_0, chờ tối đa 50 ticks
esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));

// 6. Tàu chạy về bến xong thì phải xóa đi để giải phóng RAM
i2c_cmd_link_delete(cmd);
```

### 2. Vậy tại sao trong file `drn_mpu6050.cpp` lại không thấy?
Nếu mở file thư viện `drn_mpu6050.cpp` ra, bạn sẽ thấy code rất gọn gàng:

```cpp
static esp_err_t i2c_read_multi(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, DRN_MPU6050_I2C_ADDR,
                                        &reg_addr, 1,
                                        data, len,
                                        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}
```

**Bí mật nằm ở hàm `i2c_master_write_read_device`.** Đây là một "hàm gói" (Wrapper function) mà các kỹ sư của Espressif thêm vào bộ API cũ (từ bản IDF v4.x) vì họ biết lập trình viên nào cũng phát ngán với việc phải viết 6 dòng code "tạo tàu" cho mỗi lần đọc/ghi. 

Nếu trong VS Code, bạn giữ phím `Ctrl` và click chuột trái vào hàm `i2c_master_write_read_device` để xem mã nguồn gốc (Source Code) của ESP-IDF, bạn sẽ thấy bên dưới lớp vỏ bọc đó, nó lại gọi chính xác các hàm `i2c_cmd_link_create()`, `start()`, `write()`, `read()`, `stop()`, `begin()` và `delete()`.

### Tóm lại
Bộ API cũ (`driver/i2c.h`) bản chất của nó là "gửi lệnh theo chuỗi" (Command Link). Bạn không nhìn thấy sự cồng kềnh trong file MPU6050 của bạn là nhờ bạn đã đóng gói các hàm rất bài bản, chuẩn phong cách phát triển hệ thống lớp vỏ (HAL), mượn được các hàm Wrapper của IDF để giấu đi sự rườm rà của phần cứng bên dưới.