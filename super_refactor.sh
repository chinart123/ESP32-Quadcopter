#!/bin/bash
# ============================================================
# super_refactor.sh — Drone Project Naming Convention Refactor
# Version 2 — Fixed: double-rename bug, missing pointer patterns, macOS compat
#
# Cách dùng:
#   bash super_refactor.sh --dry-run    # Xem trước, không thay đổi
#   bash super_refactor.sh --apply      # Thực sự đổi tên
#   bash super_refactor.sh --report     # Chỉ báo cáo xung đột
#
# QUAN TRỌNG: Chạy trên branch riêng trước!
#   git checkout -b refactor/naming-convention
#   bash super_refactor.sh --dry-run
#   bash super_refactor.sh --apply
#   git diff  # Kiểm tra kỹ trước khi commit
# ============================================================

set -e

MODE="dry-run"
CHANGED=0
SKIPPED=0

# ============================================================
# Platform detection — FIX: macOS dùng sed -i '' thay vì sed -i
# ============================================================
if sed --version 2>/dev/null | grep -q GNU; then
    SED_INPLACE="sed -i"        # Linux / Git Bash (GNU sed)
else
    SED_INPLACE="sed -i ''"     # macOS (BSD sed)
fi

# ============================================================
# Parse argument
# ============================================================
case "${1:-}" in
  --apply)    MODE="apply" ;;
  --dry-run)  MODE="dry-run" ;;
  --report)   MODE="report" ;;
  *)
    echo "Usage: $0 --dry-run | --apply | --report"
    exit 1
    ;;
esac

# ============================================================
# FIX: Git safety check — cảnh báo nếu đang ở main/master
# ============================================================
if git rev-parse --git-dir > /dev/null 2>&1; then
    BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
    if [ "$BRANCH" = "main" ] || [ "$BRANCH" = "master" ]; then
        echo "⚠️  CẢNH BÁO: Đang ở branch '$BRANCH'!"
        echo "   Khuyến nghị: git checkout -b refactor/naming-convention"
        if [ "$MODE" = "apply" ]; then
            echo "   Nhấn Enter để tiếp tục hoặc Ctrl+C để thoát..."
            read -r
        fi
    fi
    # FIX: Auto-backup trước khi apply
    if [ "$MODE" = "apply" ]; then
        echo "📦 Tạo backup commit trước khi refactor..."
        git add -A 2>/dev/null || true
        git commit -m "backup: before naming convention refactor" 2>/dev/null || \
            echo "   (Không có thay đổi mới để commit — OK)"
        echo ""
    fi
fi

echo "============================================"
echo " Drone Refactor Script v2 — mode: $MODE"
echo "============================================"
echo ""

# ============================================================
# Helper: rename với WORD BOUNDARY — FIX double-rename bug
# Dùng cho tên không kèm ký tự đặc biệt như () ở cuối
# ============================================================
rename_word() {
  local OLD="$1"
  local NEW="$2"
  local DESCRIPTION="$3"

  # FIX: dùng -w (whole word) để tránh match substring
  local FILES
  FILES=$(grep -rlw --include="*.c" --include="*.cpp" --include="*.h" \
          "$OLD" src/ include/ main/ 2>/dev/null || true)

  if [ -z "$FILES" ]; then
    return 0
  fi

  echo "  [RENAME] $DESCRIPTION"
  echo "           $OLD → $NEW"

  while IFS= read -r f; do
    local COUNT
    COUNT=$(grep -cw "$OLD" "$f" 2>/dev/null || echo 0)
    if [ "$COUNT" -gt 0 ]; then
      echo "    → $f ($COUNT occurrence(s))"
      if [ "$MODE" = "apply" ]; then
        # FIX: \b word boundary trong sed — tránh drn_ → drn_drn_
        $SED_INPLACE "s/\b${OLD}\b/${NEW}/g" "$f"
        CHANGED=$((CHANGED + 1))
      else
        SKIPPED=$((SKIPPED + 1))
      fi
    fi
  done <<< "$FILES"
  echo ""
}

# ============================================================
# Helper: rename với pattern chính xác (có ký tự đặc biệt như `()`)
# Dùng khi pattern đủ cụ thể, không cần word boundary
# ============================================================
rename_exact() {
  local OLD="$1"
  local NEW="$2"
  local DESCRIPTION="$3"

  local FILES
  FILES=$(grep -rl --include="*.c" --include="*.cpp" --include="*.h" \
          "$OLD" src/ include/ main/ 2>/dev/null || true)

  if [ -z "$FILES" ]; then
    return 0
  fi

  echo "  [RENAME] $DESCRIPTION"
  echo "           $OLD → $NEW"

  while IFS= read -r f; do
    local COUNT
    COUNT=$(grep -c "$OLD" "$f" 2>/dev/null || echo 0)
    if [ "$COUNT" -gt 0 ]; then
      echo "    → $f ($COUNT occurrence(s))"
      if [ "$MODE" = "apply" ]; then
        $SED_INPLACE "s/${OLD}/${NEW}/g" "$f"
        CHANGED=$((CHANGED + 1))
      else
        SKIPPED=$((SKIPPED + 1))
      fi
    fi
  done <<< "$FILES"
  echo ""
}

check_exists() {
  local PATTERN="$1"
  local DESC="$2"
  local FILES
  FILES=$(grep -rl --include="*.c" --include="*.cpp" --include="*.h" \
          "$PATTERN" src/ include/ main/ 2>/dev/null || true)
  if [ -n "$FILES" ]; then
    echo "  [FOUND] $DESC"
    echo "$FILES" | while read -r f; do
      echo "          $f"
    done
    echo ""
  fi
}

# ============================================================
# SECTION 1: Consolidate drn_ prefix
# ============================================================
echo "--- Section 1: Consolidate drn_ prefix ---"
echo ""

# FIX: dùng rename_word (có \b) vì "main_board_choose_Init" là
# substring của "drn_main_board_choose_Init" → sẽ bị double-rename
# nếu dùng rename_exact
rename_word \
  "main_board_choose_Init" \
  "drn_main_board_choose_Init" \
  "Board init — thêm drn_ prefix"

rename_word \
  "main_board_choose_Delay_ms" \
  "drn_main_board_choose_Delay_ms" \
  "Board delay — thêm drn_ prefix"

# Button functions — dùng rename_exact vì có "(" làm anchor đủ cụ thể
rename_exact \
  "button_state_hardware_scan()" \
  "DRN_Button_State_Hardware_Scan()" \
  "Button scan — uppercase DRN_"

rename_exact \
  "button_fsm_process(" \
  "DRN_Button_FSM_Process(" \
  "Button FSM — uppercase DRN_"

# FIX: Button_Context — gộp 2 rule cũ thành 1 rename_word
# Cũ: "Button_Context " và "Button_Context;" — bỏ sót "Button_Context*"
# Mới: \b bắt tất cả: Button_Context*, Button_Context&, Button_Context;
rename_word \
  "Button_Context" \
  "DRN_Button_Context" \
  "Button struct — DRN_ thắng (bắt cả pointer Button_Context*)"

rename_word \
  "ButtonEvent_TypeDef" \
  "DRN_ButtonEvent_t" \
  "Button event enum — DRN_ thắng"

# ============================================================
# SECTION 2: IMU naming sync
# ============================================================
echo "--- Section 2: IMU naming sync ---"
echo ""

# Dùng rename_exact vì có "()" làm anchor — không có double-rename risk
rename_exact \
  "MPU_Fusion_Init()" \
  "DRN_MPU6050_Init()" \
  "IMU init — STM32 rename"

rename_exact \
  "MPU_Fusion_Read_Burst()" \
  "DRN_MPU6050_Read_Raw()" \
  "IMU read — STM32 rename"

rename_exact \
  "MPU_Fusion_Compute()" \
  "DRN_MPU6050_Compute()" \
  "IMU compute — STM32 rename"

rename_exact \
  "Filter_Machine_Init()" \
  "DRN_MPU6050_Init()" \
  "Filter init → IMU init"

rename_exact \
  "Filter_Machine_Calibrate()" \
  "DRN_MPU6050_Calibrate()" \
  "Filter calibrate → IMU calibrate"

# MANUAL — không thể tự động vì cần thêm tham số
echo "  [MANUAL REQUIRED] Filter_Machine_Run() → DRN_MPU6050_Run_Task(current_tick)"
echo "  Lý do: DRN_MPU6050_Run_Task cần tham số uint32_t — sed không thể tự điền"
check_exists "Filter_Machine_Run" "Cần đổi thủ công — thêm tham số current_tick"

# ============================================================
# SECTION 3: HAL exposed — MANUAL REVIEW
# ============================================================
echo "--- Section 3: Hide HAL behind DRN_ layer ---"
echo ""

echo "  [MANUAL REVIEW REQUIRED]"
check_exists "HAL_TIM3_PWM_SetDuty" "HAL exposed — wrap vào DRN_Timer_PWM_SetDuty()"
check_exists "HAL_TIM3_PWM_Init"    "HAL exposed — wrap vào DRN_Motor_PWM_Init()"

# ============================================================
# SECTION 4: PID naming sync
# ============================================================
echo "--- Section 4: PID naming sync ---"
echo ""

rename_exact \
  "pid_compute(" \
  "Calculate_Single_PID(" \
  "PID compute — sync với STM32 naming"

# FIX: PID_t — dùng rename_word thay vì "PID_t " để bắt cả PID_t*
rename_word \
  "PID_t" \
  "PID_Controller_t" \
  "PID struct — PID_Controller_t thắng (bắt cả PID_t*)"

# ============================================================
# SECTION 5: Global variable conflicts — MANUAL
# ============================================================
echo "--- Section 5: Global variable conflicts ---"
echo ""

echo "  [MANUAL REVIEW REQUIRED]"
check_exists "drn_mpu_data" "Second IMU global — merge về Drone_IMU"
echo "  Action: Xóa drn_mpu_data, chỉ dùng Drone_IMU"
echo ""

echo "  [WARNING] micros() return type khác nhau:"
echo "    STM32: uint16 (overflow sau 65ms!)"
echo "    ESP32: uint32"
echo "    → Kiểm tra thủ công khi dùng cross-platform"
echo ""

# ============================================================
# SECTION 6: Duplicate motor init — MANUAL
# ============================================================
echo "--- Section 6: Duplicate motor init ---"
echo ""

echo "  [MANUAL REVIEW REQUIRED]"
check_exists "DRN_Timer_PWM_Init" "Duplicate — merge vào DRN_Motor_PWM_Init()"
echo "  Action: Gộp DRN_Timer_PWM_Init() vào DRN_Motor_PWM_Init()"
echo ""

# ============================================================
# Summary
# ============================================================
echo "============================================"
echo " Summary"
echo "============================================"
if [ "$MODE" = "apply" ]; then
  echo " Files changed: $CHANGED"
  echo " Next steps:"
  echo "   1. Compile để kiểm tra lỗi"
  echo "   2. Fix các [MANUAL REQUIRED] bên trên"
  echo "   3. git diff để xem toàn bộ thay đổi"
  echo "   4. git add -p để stage từng chunk"
elif [ "$MODE" = "dry-run" ]; then
  echo " DRY RUN — không có file nào bị thay đổi"
  echo " Potential changes: $SKIPPED location(s)"
  echo " Run với --apply để thực sự đổi"
else
  echo " REPORT only — không có thay đổi"
fi
echo ""
echo " ⚠️  PA6 đã cháy Gate — KHÔNG dùng PA6 cho PWM. Chỉ dùng PA7 = TIM3_CH2"
echo "============================================"