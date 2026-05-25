NHẬT KÝ KỸ THUẬT: TỔNG HỢP TOÁN HỌC VÀ THUẬT TOÁN ĐIỀU KHIỂN BAY QUADCOPTER

---

### PHẦN 1: DẠNG MA TRẬN TỔNG QUÁT CỦA PHƯƠNG TRÌNH NGOẠI SUY TRẠNG THÁI (STATE EXTRAPOLATION EQUATION)

1. Ngữ cảnh động học hệ thống:
Trong kỹ thuật điều khiển tự động và xử lý tín hiệu, các hệ thống vật lý được mô hình hóa phổ biến dưới dạng Không gian trạng thái (State-Space Representation). Phương trình ngoại suy trạng thái tuyến tính (còn gọi là phương trình trạng thái/phương trình dự đoán) được dùng làm nền tảng cốt lõi trong các bộ ước lượng tối ưu như Kalman Filter nhằm dự đoán trạng thái kế tiếp từ trạng thái hiện thời.

2. Phương trình tổng quát:
$$x_{n+1} = F x_n + G u_n$$
*(Trong lý thuyết điều khiển tự động cổ điển, phương trình này thường được ký hiệu tương đương là: $x_{k+1} = A x_k + B u_k$)*

Giải thích các thành phần:
- $x_n$: Véc-tơ trạng thái (State Vector) tại thời điểm $n$, chứa tập hợp các biến độc lập đủ để mô tả hoàn toàn hành vi của hệ thống (vị trí, vận tốc, góc nghiêng,...).
- $F$ (hoặc $A$): Ma trận chuyển trạng thái (State Transition Matrix), mô tả sự tự tiến triển động học của hệ thống từ thời điểm $n$ sang $n+1$ khi không có ngoại lực tác động.
- $u_n$: Véc-tơ đầu vào điều khiển (Control Input Vector) đại diện cho tác động từ bộ điều khiển.
- $G$ (hoặc $B$): Ma trận đầu vào điều khiển (Control Input Matrix), ánh xạ năng lượng đầu vào điều khiển vào sự thay đổi trạng thái.

3. Minh họa bằng Mô hình chuyển động vận tốc không đổi (Constant Velocity Model):
Xét một vật thể chuyển động thẳng với vị trí tọa độ $x$ và vận tốc $\dot{x}$ ổn định, chu kỳ lấy mẫu thời gian là $\Delta t$:
- Hệ phương trình sai phân rời rạc tuyến tính:
  $$x_{n+1} = x_n + \Delta t \dot{x}_n$$
  $$\dot{x}_{n+1} = \dot{x}_n$$
- Biểu diễn dưới dạng toán học ma trận không gian trạng thái (trường hợp hệ tự do không có đầu vào điều khiển $G u_n$):
  $$\begin{bmatrix} x_{n+1} \\ \dot{x}_{n+1} \end{bmatrix} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} x_n \\ \dot{x}_n \end{bmatrix}$$
  Tại đây, ma trận chuyển trạng thái tổng quát được xác định là: $F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}$.

---

### PHẦN 2: QUY ĐỔI HỆ QUY CHIẾU VÀ MÔ HÌNH HÓA 1D CƠ BẢN CHO KHÔNG GIAN TRẠNG THÁI IMU

1. Ngữ cảnh chuyển đổi:
Khi ánh xạ mô hình toán học từ chuyển động thẳng sang chuyển động quay của một máy bay bốn cánh (Quadcopter) nhằm mục đích ước lượng các góc nghiêng Euler (Roll, Pitch, Yaw), các biến vị trí tọa độ ($x$) được thay thế tương ứng bằng góc nghiêng ($\theta$), và vận tốc tuyến tính ($\dot{x}$) được thay thế bằng vận tốc góc ($\dot{\theta}$) thu được từ cảm biến con quay hồi chuyển (Gyroscope).

2. Mô hình tuyến tính cơ bản 1D (Góc và Vận tốc góc):
Nếu áp dụng nguyên bản mô hình chuyển động hằng số tuyến tính từ Phần 1, hệ phương trình ma trận dự đoán cho một góc đơn trục (ví dụ góc Pitch $\theta$) sẽ có dạng:
$$\begin{bmatrix} \theta_{n+1} \\ \dot{\theta}_{n+1} \end{bmatrix} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} \theta_n \\ \dot{\theta}_n \end{bmatrix}$$

3. Mô hình 1D thực tế tối ưu cho phần cứng nhúng (Ước lượng góc nghiêng và Gyro Bias):
Trong thực tế lập trình firmware điều khiển bay, vận tốc góc thô thu được từ cảm biến Gyroscope ($\dot{\theta}_{gyro}$) không được coi là một biến trạng thái tự do, mà được đưa vào hệ thống như một tín hiệu đầu vào điều khiển $U_n$. Đồng thời, để triệt tiêu sai số tích phân lũy tiến gây trôi góc (drift), người ta bổ sung một biến trạng thái mới là Độ lệch tĩnh của Gyro (Gyro Bias - $\beta$).

- Hệ phương trình vi phân trạng thái mới được thiết lập:
  $$\begin{bmatrix} \theta_{n+1} \\ \beta_{n+1} \end{bmatrix} = \begin{bmatrix} 1 & -\Delta t \\ 0 & 1 \end{bmatrix} \begin{bmatrix} \theta_n \\ \beta_n \end{bmatrix} + \begin{bmatrix} \Delta t \\ 0 \end{bmatrix} \dot{\theta}_{gyro}$$

Đối chiếu lại dạng tổng quát $X_{n+1} = F X_n + G U_n$:
- Ma trận chuyển trạng thái hệ thống: $F = \begin{bmatrix} 1 & -\Delta t \\ 0 & 1 \end{bmatrix}$
- Ma trận phân phối tín hiệu điều khiển đầu vào: $G = \begin{bmatrix} \Delta t \\ 0 \end{bmatrix}$

---

### PHẦN 3: HẠN CHẾ CỦA BỘ LỌC TUYẾN TÍNH 1D SO VỚI MÔ HÌNH ĐỘNG HỌC 3D ĐẦY ĐỦ

Việc phân rã hệ thống điều khiển bay thành 3 bộ lọc góc tuyến tính 1D hoạt động độc lập cho các trục Roll, Pitch, Yaw có những điểm thiếu hụt nghiêm trọng khi so sánh với hệ phương trình động học phi tuyến toàn phần trong không gian 3 chiều:

| Tiêu chí hình học | Dạng cơ bản (3 bộ lọc 1D độc lập) | Dạng đầy đủ phi tuyến (Hệ động học 3D) |
| :--- | :--- | :--- |
| **Sự phụ thuộc chéo giữa các trục (Cross-coupling)** | **Bỏ qua.** Coi chuyển động quay của một trục lật hoàn toàn không gây ảnh hưởng hay làm thay đổi giá trị của các trục còn lại. | **Tính toán đầy đủ.** Trong không gian 3D, sự thay đổi hướng của một trục phụ thuộc trực tiếp vào trạng thái góc hiện tại của trục khác thông qua các ràng buộc lượng giác phức tạp: $\dot{\phi} = p + q \sin\phi \tan\theta + r \cos\phi \tan\theta$. |
| **Hiện tượng Khóa trục hình học (Gimbal Lock)** | **Bị ảnh hưởng nghiêm trọng.** Do tính toán trực tiếp trên hệ góc Euler, khi drone thực hiện cú lật với góc Pitch tiến sát góc biên $\pm90^\circ$, hệ trục Roll và Yaw sẽ bị trùng lặp, triệt tiêu mất một bậc tự do và gây lỗi chia cho zero trong ma trận thuật toán. | **Triệt tiêu hoàn toàn.** Hệ thống động học dạng đầy đủ không lưu trữ trạng thái bằng góc Euler trong ma trận bộ lọc, mà sử dụng không gian toán học **Quaternions** ($q_0, q_1, q_2, q_3$) làm trạng thái trung gian, loại bỏ hoàn toàn suy biến tọa độ. |
| **Tính tuyến tính của Ma trận chuyển $F$** | Ma trận $F$ là tuyến tính và là một **hằng số hằng định** cố định qua mọi chu kỳ tính toán (chỉ bao gồm các số 1, 0 và hằng số thời gian $\Delta t$). | Hệ thống động học thực chất là **phi tuyến**. Ma trận $F$ (hoặc ma trận Jacobian) không cố định mà phải liên tục cập nhật và tính toán lại giá trị số tại mỗi chu kỳ trích mẫu dựa trên góc quay tức thời của drone. |
| **Yêu cầu tài nguyên xử lý dữ liệu (Firmware)** | Khối lượng tính toán cực nhẹ, chỉ thực hiện nhân cộng các ma trận cấp thấp, vi điều khiển cấu hình thấp có thể thực thi ở tần số hàng ngàn Hz. | Khối lượng tính toán rất nặng do phải liên tục xử lý các hàm lượng giác hoặc đại số Quaternion đại lượng lớn, đòi hỏi bộ xử lý có đơn vị tính toán số thực dấu phẩy động FPU (như ESP32-S3 hoặc STM32). |

---

### PHẦN 4: TOÁN HỌC QUATERNIONS VÀ VẤN ĐỀ CAN THIỆP THANH GHI BARE-METAL

1. Bản chất phần cứng và việc can thiệp thanh ghi Bare-metal:
- Việc sử dụng đại số Quaternion để ước lượng trạng thái góc **hoàn toàn không bắt buộc** kỹ sư phải can thiệp hay cấu hình thêm bất kỳ thanh ghi đặc biệt nào thuộc lớp phần cứng ẩn bên trong cảm biến IMU (ví dụ bộ xử lý chuyển động phần cứng DMP của MPU6050).
- Bản chất Quaternion là một **thuật toán phần mềm (Software Algorithm)** thực thi trực tiếp trên lõi của vi điều khiển trung tâm (MCU). Quy trình phần cứng nhúng hoàn toàn giữ nguyên: firmware giao tiếp qua bus I2C để đọc dữ liệu thô từ các thanh ghi cơ bản của cảm biến (`ACCEL_XOUT_H`, `GYRO_XOUT_H`), sau đó nạp các mảng dữ liệu này vào bộ lọc tích hợp phần mềm như Madgwick Filter hoặc Mahony Filter để tính toán ra Quaternion.

2. Bản chất trực quan hình học của Quaternions:
Góc Euler thực hiện phép xoay tuần tự theo các trục riêng lẻ, dẫn tới lỗi Gimbal Lock. Quaternion xử lý bài toán này bằng cách thực hiện một phép xoay duy nhất ("một phát ăn ngay") trong không gian 3D: Xuyên một trục thép tưởng tượng là một véc-tơ đơn vị $\vec{v} = (x, y, z)$ qua tâm khối lượng của máy bay, sau đó thực hiện quay toàn bộ cấu trúc máy bay quanh trục thép đó một góc duy nhất là $\theta$.

Bốn con số đại số của Quaternion ($q_0, q_1, q_2, q_3$) gói gọn chính xác thông tin về góc xoay $\theta$ và trục xoay $\vec{v}$ này:
- $q_0 = \cos\left(\frac{\theta}{2}\right)$ (Thành phần vô hướng - đại diện cho góc xoay)
- $q_1 = x \cdot \sin\left(\frac{\theta}{2}\right)$ (Thành phần véc-tơ hóa theo trục X)
- $q_2 = y \cdot \sin\left(\frac{\theta}{2}\right)$ (Thành phần véc-tơ hóa theo trục Y)
- $q_3 = z \cdot \sin\left(\frac{\theta}{2}\right)$ (Thành phần véc-tơ hóa theo trục Z)

3. Phương trình vi phân cập nhật trạng thái Quaternion từ dữ liệu vận tốc góc Gyro ($\omega_x, \omega_y, \omega_z$):
Lợi thế lớn nhất của Quaternion trong lập trình nhúng là phương trình vi phân động học của nó hoàn toàn tuyến tính và chỉ bao gồm các phép toán nhân, cộng đại số cơ bản, loại bỏ hoàn toàn các hàm lượng giác `sin()`, `cos()` đắt đỏ tại vòng lặp tính toán:
$$\begin{bmatrix} \dot{q_0} \\ \dot{q_1} \\ \dot{q_2} \\ \dot{q_3} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} 0 & -\omega_x & -\omega_y & -\omega_z \\ \omega_x & 0 & \omega_z & -\omega_y \\ \omega_y & -\omega_z & 0 & \omega_x \\ \omega_z & \omega_y & -\omega_x & 0 \end{bmatrix} \begin{bmatrix} q_0 \\ q_1 \\ q_2 \\ q_3 \end{bmatrix}$$

---

### PHẦN 5: KHẢO SÁT THUẬT TOÁN KALMAN FILTER TRONG BÁO CÁO MILESTONE 2 (M2)

1. Tập hợp các công thức toán học đang triển khai trong firmware hệ thống:
Dựa trên tài liệu thiết kế hệ thống góc đơn trục độc lập để khử nhiễu từ động cơ coreless 8520, các phương trình sai phân bộ lọc được áp dụng tuần tự như sau:

- *Phương trình tích phân dữ liệu vận tốc góc từ Gyroscope để tính góc thô (Gyro integration):*
  $$\theta_t = \theta_{t-1} + \omega \cdot dt$$
- *Phương trình bước dự đoán trạng thái (Prediction stage - a priori estimate):*
  $$\hat{x}_k^- = \hat{x}_{k-1} + \omega_k dt$$
- *Phương trình xác định Hệ số tăng Kalman (Kalman Gain) ở bước hiệu chỉnh:*
  $$K_k = \frac{P_k^-}{P_k^- + R}$$
- *Phương trình cập nhật và sửa sai trạng thái tối ưu bằng dữ liệu từ Gia tốc kế (Corrected output angle estimate):*
  $$\hat{x}_k = \hat{x}_k^- + K_k(z_k - \hat{x}_k^-)$$

Trong đó:
- $\hat{x}_k^-$: Góc dự đoán tiên nghiệm (a priori).
- $\omega_k$: Vận tốc góc từ gyroscope đầu vào.
- $z_k$: Góc đo trực tiếp từ gia tốc kế (Accelerometer) dùng làm mốc hiệu chỉnh tuyệt đối.
- $K_k$: Hệ số tăng Kalman đóng vai trò trọng số phân bổ niềm tin giữa bước dự đoán lý thuyết và bước đo đạc thực tế của cảm biến.
- $P_k^-$: Hiệp biến sai số dự đoán.
- $R$: Hiệp biến nhiễu của phép đo cảm biến phần cứng.

2. Đánh giá kỹ thuật:
Mô hình toán học trên chính là việc triển khai cụ thể của **"Dạng lọc tuyến tính cơ bản 1D"** chạy song song độc lập cho từng trục đơn lẻ (Roll và Pitch). Lựa chọn này hoàn toàn đồng bộ và tối ưu với định hướng xây dựng một nền tảng bay mở phục vụ giáo dục STEM, do thuật toán có tính trực quan toán học cao, cấu trúc mã nguồn C/C++ tường minh, dễ hiệu chỉnh thực nghiệm các tham số nhiễu ($Q, R$) và vận hành cực kỳ tiết kiệm năng lượng xử lý trên kiến trúc vi xử lý dual-core của ESP32-S3.