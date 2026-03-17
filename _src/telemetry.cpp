#include "telemetry.h"
#include "esp_log.h"
#include <string.h>

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h" // Cấp phát bộ nhớ cho gói tin

static const char *TAG = "BLE_TELEMETRY";
static uint8_t own_addr_type;

// Biến lưu mã kết nối của điện thoại (Handle)
static uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// ==========================================================
// 1. ĐỊNH NGHĨA UUID CHUẨN NORDIC UART (NUS)
// ==========================================================
const ble_uuid128_t gatt_svr_svc_nus_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

const ble_uuid128_t gatt_svr_chr_nus_rx_uuid = 
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

const ble_uuid128_t gatt_svr_chr_nus_tx_uuid = 
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

uint16_t nus_tx_handle; 

// ==========================================================
// 2. HÀM NGẮT (CALLBACK) KHI NHẬN DỮ LIỆU TỪ APP
// ==========================================================
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char buf[50];
        int len = ctxt->om->om_len;
        if(len > 49) len = 49; 
        
        memcpy(buf, ctxt->om->om_data, len);
        buf[len] = '\0';
        ESP_LOGI(TAG, "=> Dien thoai vua gui lenh: %s", buf);
    }
    return 0;
}

// ==========================================================
// 3. XÂY DỰNG BẢNG DỊCH VỤ (GATT TABLE)
// ==========================================================
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_nus_uuid.u, 
        .includes = NULL,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_nus_rx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .arg = NULL,
                .descriptors = NULL,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .min_key_size = 0,
                .val_handle = NULL,
            },
            {
                .uuid = &gatt_svr_chr_nus_tx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .arg = NULL,
                .descriptors = NULL,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = 0,
                .val_handle = &nus_tx_handle,
            },
            { 0 }
        }
    },
    { 0 }
};

// ==========================================================
// 4. SỰ KIỆN GAP (KẾT NỐI / NGẮT KẾT NỐI)
// ==========================================================
static void ble_app_advertise(void); // Khai báo trước để dùng trong hàm dưới

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ble_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Da ket noi voi dien thoai! Handle: %d", ble_conn_handle);
            } else {
                ble_app_advertise(); // Lỗi thì phát sóng lại
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Dien thoai da ngat ket noi!");
            ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise(); // Bật lại quảng bá để kết nối lại
            break;
    }
    return 0;
}

// ==========================================================
// 5. CẤU HÌNH GÓI TIN PHÁT SÓNG
// ==========================================================
static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *device_name;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    device_name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    fields.uuids128 = (ble_uuid128_t*)&gatt_svr_svc_nus_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; 
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; 

    // Đăng ký ble_gap_event vào đây
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    ESP_LOGI(TAG, "Dang phat song BLE voi ten: %s", device_name);
}

static void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type); 
    ble_app_advertise();           
}

void ble_host_task(void *param) {
    ESP_LOGI(TAG, "Khoi dong NimBLE Task...");
    nimble_port_run(); 
    nimble_port_freertos_deinit();
}

// ==========================================================
// 6. CÁC HÀM GỌI TỪ MAIN
// ==========================================================
void BLE_Init(void) {
    ESP_LOGI(TAG, "Dang nap Driver Bluetooth NimBLE...");
    nimble_port_init();
    
    ble_svc_gap_device_name_set("Drone_S3");
    ble_svc_gap_init();
    
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}

void BLE_Send_Data(const char* data) {
    if (ble_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return; // Chưa có ai kết nối
    }
    
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, strlen(data));
    if (om == NULL) {
        return; // Hết bộ nhớ đệm
    }
    
    // Bắn gói tin đi qua ống TX (nus_tx_handle)
    ble_gatts_notify_custom(ble_conn_handle, nus_tx_handle, om);
}