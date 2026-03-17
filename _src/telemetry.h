#ifndef TELEMETRY_H
#define TELEMETRY_H

// Khởi tạo và phát sóng Bluetooth (NimBLE)
void BLE_Init(void);

// Hàm gửi dữ liệu văn bản qua Bluetooth tới App
void BLE_Send_Data(const char* data);

#endif