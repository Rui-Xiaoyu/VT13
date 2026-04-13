#pragma once

/* clang-format off */
/* === MODULE MANIFEST ===
module_name: VT13
module_description: VT13 link receiver parsing
constructor_args:
  - CMD: '@cmd'
  - task_stack_depth_uart: 1536
  - thread_priority_uart: LibXR::Thread::Priority::HIGH
required_hardware: vt13 dma uart
depends:
  - qdu-future/CMD
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "CMD.hpp"
#include "app_framework.hpp"
#include "crc.hpp"
#include "uart.hpp"

#define VT13_FRAME_HEAD_0 (0xA9u)
#define VT13_FRAME_HEAD_1 (0x53u)

/**
 * @brief VT13遥控器通道值范围定义
 */
#define VT13_CH_VALUE_MIN (364u)  /* 通道最小值 */
#define VT13_CH_VALUE_MID (1024u) /* 通道中间值 */
#define VT13_CH_VALUE_MAX (1684u) /* 通道最大值 */

/**
 * @class VT13
 * @brief VT13链路接收机数据解析类
 */
class VT13 : public LibXR::Application {
 public:
  static constexpr std::size_t VT13_FRAME_SIZE = 21;
  static constexpr std::size_t VT13_PAYLOAD_SIZE_FOR_CRC = 19;

  /**
   * @brief 控制源枚举
   */
  enum class ControlSource : uint8_t {
    VT13_CTRL_SOURCE_SW = 0x00,
    VT13_CTRL_SOURCE_MOUSE = 0x01,
  };

  /**
   * @brief VT13拨杆位置与扩展事件枚举
   */
  enum class SwitchPos : uint16_t {
    /* 切换开关和自定义按键 */
    VT13_KEY_RELEASE_L = 0x100, /* 自定义左键松开 */
    VT13_KEY_PRESSED_L,         /* 自定义左键按下 */
    VT13_KEY_RELEASE_R,         /* 自定义右键松开 */
    VT13_KEY_PRESSED_R,         /* 自定义右键按下 */
    VT13_KEY_RELEASE_PAUSED,    /* 暂停键松开 */
    VT13_KEY_PRESSED_PAUSED,    /* 暂停键按下 */
    VT13_KEY_RELEASE_TRIG,      /* 扳机松开 */
    VT13_KEY_PRESSED_TRIG,      /* 扳机按下 */

    VT13_SW_POS_C = 0x00, /* C档(上) */
    VT13_SW_POS_N = 0x01, /* N档(中) */
    VT13_SW_POS_S = 0x02, /* S档(下) */
    VT13_SW_POS_NUM = 3,  /* 档位数量 */

    /* 拨轮触碰事件：上拨短触/上拨长触/下拨触发 */
    VT13_DIAL_UP_SHORT = 0x120, /* 拨轮上拨短触 */
    VT13_DIAL_UP_LONG,          /* 拨轮上拨长触 */
    VT13_DIAL_DOWN_TOUCH,       /* 拨轮下拨触发 */

    /* 按键动作切换结果：可直接绑定业务语义 */
    VT13_KEY_PAUSE_TOGGLE_ON = 0x130, /* 暂停键切换后ON */
    VT13_KEY_PAUSE_TOGGLE_OFF,        /* 暂停键切换后OFF */
    VT13_KEY_CUSTOM_L_TOGGLE_ON,      /* 自定义左键切换后ON */
    VT13_KEY_CUSTOM_L_TOGGLE_OFF,     /* 自定义左键切换后OFF */
    VT13_KEY_CUSTOM_R_TOGGLE_ON,      /* 自定义右键切换后ON */
    VT13_KEY_CUSTOM_R_TOGGLE_OFF,     /* 自定义右键切换后OFF */
  };

  /**
   * @brief 键盘与鼠标事件编码枚举
   */
  enum class Key : uint8_t {
    KEY_W = static_cast<uint8_t>(SwitchPos::VT13_SW_POS_NUM),
    KEY_S,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,

    KEY_L_PRESS,
    KEY_R_PRESS,
    KEY_M_PRESS,
    KEY_L_RELEASE,
    KEY_R_RELEASE,
    KEY_M_RELEASE,
    KEY_NUM,
  };

  /**
   * @brief VT13协议解包后的原始数据
   */
  typedef struct {
    uint16_t ch_r_x;
    uint16_t ch_r_y;
    uint16_t ch_l_x;
    uint16_t ch_l_y;
    uint8_t sw;
    uint8_t pause;
    uint8_t key_l;
    uint8_t key_r;
    uint16_t dial;
    uint8_t trig;
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
    uint8_t press_m;
    uint16_t key;
  } Data;

  /**
   * @brief 计算Shift组合键事件编码
   * @param key 基础按键
   * @return Shift组合后的事件值
   */
  constexpr uint32_t ShiftWith(Key key) {
    return static_cast<uint8_t>(key) + 1 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 计算Ctrl组合键事件编码
   * @param key 基础按键
   * @return Ctrl组合后的事件值
   */
  constexpr uint32_t CtrlWith(Key key) {
    return static_cast<uint8_t>(key) + 2 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 计算Shift+Ctrl组合键事件编码
   * @param key 基础按键
   * @return Shift+Ctrl组合后的事件值
   */
  constexpr uint32_t ShiftCtrlWith(Key key) {
    return static_cast<uint8_t>(key) + 3 * static_cast<uint8_t>(Key::KEY_NUM);
  }

  /**
   * @brief 获取键盘位图中的原始bit掩码
   * @param key 键位枚举
   * @return 对应bit值，不在W~B范围则返回0
   */
  constexpr uint32_t RawValue(Key key) {
    const auto KEY_U = static_cast<uint8_t>(key);
    const auto KEY_W_U = static_cast<uint8_t>(Key::KEY_W);
    const auto KEY_B_U = static_cast<uint8_t>(Key::KEY_B);
    if (KEY_U < KEY_W_U || KEY_U > KEY_B_U) {
      return 0;
    }
    return 1u << (KEY_U - KEY_W_U);
  }

  /**
   * @brief VT13构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param cmd 控制命令对象引用
   * @param task_stack_depth_uart UART任务栈深度
   * @param thread_priority_uart UART线程优先级
   */
  VT13(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, CMD& cmd,
       uint32_t task_stack_depth_uart,
       LibXR::Thread::Priority thread_priority_uart =
           LibXR::Thread::Priority::HIGH)
      : cmd_(&cmd),
        uart_(hw.Find<LibXR::UART>("uart_ext_controller")),
        sem_(0),
        op_(sem_, 64) {
    uart_->SetConfig({921600, LibXR::UART::Parity::NO_PARITY, 8, 1});
    /* 创建UART线程 */
    thread_uart_.Create(this, ThreadVT13, "uart_vt13", task_stack_depth_uart,
                        thread_priority_uart);
    app.Register(*this);
  }

  /**
   * @brief 获取VT13事件对象
   * @return LibXR::Event& 事件对象引用
   */
  LibXR::Event& GetEvent() { return vt13_event_; }

  /**
   * @brief 监控回调
   */
  void OnMonitor() override {}

  /**
   * @brief VT13 UART读取线程
   * @param vt13 VT13实例指针
   */
  static void ThreadVT13(VT13* vt13) {
    vt13->uart_->read_port_->Reset();

    constexpr std::size_t RX_BUFFER_SIZE = 64;
    uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
    uint8_t frame_buffer[VT13_FRAME_SIZE] = {0};
    std::size_t frame_pos = 0;
    CMD::Data rc_data;

    while (1) {
      if (vt13->uart_->Read({rx_buffer, RX_BUFFER_SIZE}, vt13->op_) ==
          ErrorCode::OK) {
        for (std::size_t idx = 0; idx < RX_BUFFER_SIZE; ++idx) {
          uint8_t rx_byte = rx_buffer[idx];

          /* 字节流状态机：先同步2字节帧头，再累积到完整21字节帧 */
          if (frame_pos == 0) {
            if (rx_byte != VT13_FRAME_HEAD_0) {
              continue;
            }
            frame_buffer[frame_pos++] = rx_byte;
          } else if (frame_pos == 1) {
            if (rx_byte == VT13_FRAME_HEAD_1) {
              frame_buffer[frame_pos++] = rx_byte;
            } else {
              frame_pos = 0;
              /* 若当前字节本身也是首字节，则直接作为下一帧起点继续对齐 */
              if (rx_byte == VT13_FRAME_HEAD_0) {
                frame_buffer[frame_pos++] = rx_byte;
              }
            }
          } else {
            frame_buffer[frame_pos++] = rx_byte;
            if (frame_pos < VT13_FRAME_SIZE) {
              continue;
            }

            frame_pos = 0;
            if (vt13->ParseRC(frame_buffer, rc_data) == ErrorCode::OK) {
              /* 仅在完整且通过校验的帧上更新时间戳并下发控制数据 */
              vt13->last_time_ = LibXR::Timebase::GetMilliseconds();
              vt13->cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_VT13, rc_data);
            }
          }
        }
      }
      vt13->CheckoutOffline();
      LibXR::Thread::Sleep(2);
    }
  }

  /**
   * @brief 解析VT13原始帧并生成CMD控制数据
   * @param raw_data 21字节原始缓冲
   * @param output_data 解析后的CMD数据
   * @return ErrorCode::OK 解析成功；其他值表示校验或数据范围异常
   */
  ErrorCode ParseRC(const uint8_t* raw_data, CMD::Data& output_data) {
    if (!raw_data) {
      return ErrorCode::PTR_NULL;
    }

    /* 固定帧头校验 */
    if (raw_data[0] != VT13_FRAME_HEAD_0 || raw_data[1] != VT13_FRAME_HEAD_1) {
      return ErrorCode::CHECK_ERR;
    }

    /* CRC覆盖前19字节，帧尾2字节为校验值 */
    const bool CRC_OK = VerifyCRC(raw_data);
    if (!CRC_OK) {
      return ErrorCode::CHECK_ERR;
    }

    Data curr_rc{};

    curr_rc.ch_r_x = static_cast<uint16_t>(ExtractBits(raw_data, 16, 11));
    curr_rc.ch_r_y = static_cast<uint16_t>(ExtractBits(raw_data, 27, 11));
    curr_rc.ch_l_x = static_cast<uint16_t>(ExtractBits(raw_data, 38, 11));
    curr_rc.ch_l_y = static_cast<uint16_t>(ExtractBits(raw_data, 49, 11));
    curr_rc.sw = static_cast<uint8_t>(ExtractBits(raw_data, 60, 2));
    curr_rc.pause = static_cast<uint8_t>(ExtractBits(raw_data, 62, 1));
    curr_rc.key_l = static_cast<uint8_t>(ExtractBits(raw_data, 63, 1));
    curr_rc.key_r = static_cast<uint8_t>(ExtractBits(raw_data, 64, 1));
    curr_rc.dial = static_cast<uint16_t>(ExtractBits(raw_data, 65, 11));
    curr_rc.trig = static_cast<uint8_t>(ExtractBits(raw_data, 76, 1));
    curr_rc.x = static_cast<int16_t>(ExtractBits(raw_data, 80, 16));
    curr_rc.y = static_cast<int16_t>(ExtractBits(raw_data, 96, 16));
    curr_rc.z = static_cast<int16_t>(ExtractBits(raw_data, 112, 16));
    curr_rc.press_l = static_cast<uint8_t>(ExtractBits(raw_data, 128, 2));
    curr_rc.press_r = static_cast<uint8_t>(ExtractBits(raw_data, 130, 2));
    curr_rc.press_m = static_cast<uint8_t>(ExtractBits(raw_data, 132, 2));
    curr_rc.key = static_cast<uint16_t>(ExtractBits(raw_data, 136, 16));

    const bool RANGE_OK = !(
        curr_rc.ch_r_x < VT13_CH_VALUE_MIN ||
        curr_rc.ch_r_x > VT13_CH_VALUE_MAX ||
        curr_rc.ch_r_y < VT13_CH_VALUE_MIN ||
        curr_rc.ch_r_y > VT13_CH_VALUE_MAX ||
        curr_rc.ch_l_x < VT13_CH_VALUE_MIN ||
        curr_rc.ch_l_x > VT13_CH_VALUE_MAX ||
        curr_rc.ch_l_y < VT13_CH_VALUE_MIN ||
        curr_rc.ch_l_y > VT13_CH_VALUE_MAX ||
        curr_rc.dial < VT13_CH_VALUE_MIN || curr_rc.dial > VT13_CH_VALUE_MAX ||
        curr_rc.sw > static_cast<uint8_t>(SwitchPos::VT13_SW_POS_S));

    if (!RANGE_OK) {
      return ErrorCode::CHECK_ERR;
    }

    output_data = CMD::Data();

    /* 断链恢复后的首帧仅用于建立边沿基线，避免误触发事件 */
    if (this->offline_latched_) {
      this->last_data_ = curr_rc;
      this->offline_latched_ = false;
    }

    /* 检测挡位切换开关 */
    if (curr_rc.sw != this->last_data_.sw) {
      this->vt13_event_.Active(static_cast<uint32_t>(SwitchPos::VT13_SW_POS_C) +
                               curr_rc.sw);
    }

    /* 检测其余自定义按键 */
    if (curr_rc.key_l && !this->last_data_.key_l) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_KEY_PRESSED_L));

      this->fric_enable_ = !this->fric_enable_;
      if (this->fric_enable_) {
        this->vt13_event_.Active(
            static_cast<uint32_t>(SwitchPos::VT13_KEY_CUSTOM_L_TOGGLE_ON));
      } else {
        this->vt13_event_.Active(
            static_cast<uint32_t>(SwitchPos::VT13_KEY_CUSTOM_L_TOGGLE_OFF));
      }
    }

    if (curr_rc.key_r && !this->last_data_.key_r) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_KEY_PRESSED_R));

      this->ai_mode_enable_ = !this->ai_mode_enable_;
      if (this->ai_mode_enable_) {
        this->vt13_event_.Active(
            static_cast<uint32_t>(SwitchPos::VT13_KEY_CUSTOM_R_TOGGLE_ON));
      } else {
        this->vt13_event_.Active(
            static_cast<uint32_t>(SwitchPos::VT13_KEY_CUSTOM_R_TOGGLE_OFF));
      }
    }

    if (curr_rc.pause && !this->last_data_.pause) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_KEY_PRESSED_PAUSED));

      if (this->ai_mode_enable_) {
        this->gimbal_enable_ = false;
      } else {
        this->gimbal_enable_ = !this->gimbal_enable_;
      }

      if (this->gimbal_enable_) {
        this->vt13_event_.Active(
            static_cast<uint32_t>(SwitchPos::VT13_KEY_PAUSE_TOGGLE_ON));
      } else {
        this->vt13_event_.Active(
            static_cast<uint32_t>(SwitchPos::VT13_KEY_PAUSE_TOGGLE_OFF));
      }
    }

    if (curr_rc.trig && !this->last_data_.trig) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_KEY_PRESSED_TRIG));
    }

    if (!curr_rc.key_l && this->last_data_.key_l) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_KEY_RELEASE_L));
    }
    if (!curr_rc.key_r && this->last_data_.key_r) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_KEY_RELEASE_R));
    }
    if (!curr_rc.pause && this->last_data_.pause) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_KEY_RELEASE_PAUSED));
    }
    if (!curr_rc.trig && this->last_data_.trig) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_KEY_RELEASE_TRIG));
    }

    this->ActiveDialTouchEvent(curr_rc.dial);

    /* 检测Shift/Ctrl */
    uint32_t tmp = 0;
    if (curr_rc.key & RawValue(Key::KEY_SHIFT)) {
      tmp += static_cast<uint32_t>(Key::KEY_NUM);
    }
    if (curr_rc.key & RawValue(Key::KEY_CTRL)) {
      tmp += 2 * static_cast<uint32_t>(Key::KEY_NUM);
    }

    for (int i = 0; i < 16; i++) {
      if ((curr_rc.key & (1u << i)) && !(this->last_data_.key & (1u << i))) {
        this->vt13_event_.Active(static_cast<uint32_t>(Key::KEY_W) + i + tmp);
      }
    }

    const uint16_t COMBO_SW = RawValue(Key::KEY_SHIFT) |
                              RawValue(Key::KEY_CTRL) | RawValue(Key::KEY_Q);
    const uint16_t COMBO_MOUSE = RawValue(Key::KEY_SHIFT) |
                                 RawValue(Key::KEY_CTRL) | RawValue(Key::KEY_E);

    /* Shift+Ctrl+Q/E 控制源切换 */
    if ((curr_rc.key & COMBO_SW) == COMBO_SW) {
      this->ctrl_source_ = ControlSource::VT13_CTRL_SOURCE_SW;
    }
    if ((curr_rc.key & COMBO_MOUSE) == COMBO_MOUSE) {
      this->ctrl_source_ = ControlSource::VT13_CTRL_SOURCE_MOUSE;
    }

    constexpr float FULL_RANGE =
        static_cast<float>(VT13_CH_VALUE_MAX - VT13_CH_VALUE_MIN);
    constexpr float INV_FULL_RANGE = 1.0f / FULL_RANGE;
    constexpr float MOUSE_SCALER = 1000.0f / 32768.0f;

    if (this->ctrl_source_ == ControlSource::VT13_CTRL_SOURCE_MOUSE) {
      if (curr_rc.press_l && !this->last_data_.press_l) {
        this->vt13_event_.Active(static_cast<uint32_t>(Key::KEY_L_PRESS));
      }
      if (curr_rc.press_r && !this->last_data_.press_r) {
        this->vt13_event_.Active(static_cast<uint32_t>(Key::KEY_R_PRESS));
      }
      if (curr_rc.press_m && !this->last_data_.press_m) {
        this->vt13_event_.Active(static_cast<uint32_t>(Key::KEY_M_PRESS));
      }
      if (!curr_rc.press_l && this->last_data_.press_l) {
        this->vt13_event_.Active(static_cast<uint32_t>(Key::KEY_L_RELEASE));
      }
      if (!curr_rc.press_r && this->last_data_.press_r) {
        this->vt13_event_.Active(static_cast<uint32_t>(Key::KEY_R_RELEASE));
      }
      if (!curr_rc.press_m && this->last_data_.press_m) {
        this->vt13_event_.Active(static_cast<uint32_t>(Key::KEY_M_RELEASE));
      }

      if (curr_rc.key & RawValue(Key::KEY_A)) {
        output_data.chassis.x -= 1.0f;
      }
      if (curr_rc.key & RawValue(Key::KEY_D)) {
        output_data.chassis.x += 1.0f;
      }
      if (curr_rc.key & RawValue(Key::KEY_S)) {
        output_data.chassis.y -= 1.0f;
      }
      if (curr_rc.key & RawValue(Key::KEY_W)) {
        output_data.chassis.y += 1.0f;
      }

      if (curr_rc.key & RawValue(Key::KEY_SHIFT)) {
        output_data.chassis.self_define = CMD::ChasStat::BOOST;
      } else {
        output_data.chassis.self_define = CMD::ChasStat::NONE;
      }
      output_data.chassis.z = 0.0f;

      output_data.gimbal.pit = static_cast<float>(curr_rc.y) * MOUSE_SCALER;
      output_data.gimbal.yaw = -static_cast<float>(curr_rc.x) * MOUSE_SCALER;
      output_data.gimbal.rol = 0.0f;

      if (curr_rc.press_l != 0) {
        output_data.launcher.isfire = true;
      } else {
        output_data.launcher.isfire = false;
      }
    } else {
      /* 遥控器模式 */
      output_data.chassis.x =
          2.0f * (static_cast<float>(curr_rc.ch_l_y) - VT13_CH_VALUE_MID) *
          INV_FULL_RANGE;
      output_data.chassis.y =
          2.0f * (static_cast<float>(curr_rc.ch_l_x) - VT13_CH_VALUE_MID) *
          INV_FULL_RANGE;
      output_data.chassis.z =
          2.0f * (static_cast<float>(curr_rc.ch_r_x) - VT13_CH_VALUE_MID) *
          INV_FULL_RANGE;
      output_data.chassis.self_define = CMD::ChasStat::NONE;

      output_data.gimbal.yaw =
          2.0f * (static_cast<float>(curr_rc.ch_r_x) - VT13_CH_VALUE_MID) *
          INV_FULL_RANGE;
      output_data.gimbal.pit =
          2.0f * (static_cast<float>(curr_rc.ch_r_y) - VT13_CH_VALUE_MID) *
          INV_FULL_RANGE;
      output_data.gimbal.rol = 0.0f;

      if (curr_rc.trig != 0) {
        output_data.launcher.isfire = true;
      } else {
        output_data.launcher.isfire = false;
      }
    }

    output_data.chassis_online = true;
    output_data.gimbal_online = true;
    output_data.ctrl_source = CMD::ControlSource::CTRL_SOURCE_RC;

    this->last_data_ = curr_rc;

    return ErrorCode::OK;
  }

  /**
   * @brief 离线安全输出
   * @details 将控制量归零并标记离线，防止链路中断时保持危险状态
   */
  void Offline() {
    this->cmd_data_.chassis.x = 0;
    this->cmd_data_.chassis.y = 0;
    this->cmd_data_.chassis.z = 0;
    this->cmd_data_.chassis.self_define = CMD::ChasStat::NONE;

    this->cmd_data_.gimbal.yaw = 0;
    this->cmd_data_.gimbal.pit = 0;
    this->cmd_data_.gimbal.rol = 0;

    this->cmd_data_.launcher.isfire = false;

    this->cmd_data_.chassis_online = false;
    this->cmd_data_.gimbal_online = false;

    this->gimbal_enable_ = false;
    this->fric_enable_ = false;
    this->ai_mode_enable_ = false;
    /* 清除拨轮触碰中间态，避免离线恢复后误触发短触/长触事件 */
    this->dial_up_active_ = false;
    this->dial_up_long_triggered_ = false;

    this->last_data_ = Data{};
    this->offline_latched_ = true;

    this->cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_VT13, this->cmd_data_);
  }

 private:
  CMD* cmd_; /* CMD模块指针 */
  ControlSource ctrl_source_ = ControlSource::VT13_CTRL_SOURCE_SW;

  bool gimbal_enable_ = false;
  bool fric_enable_ = false;
  bool ai_mode_enable_ = false;

  Data last_data_{};     /* 上一帧数据 */
  CMD::Data cmd_data_{}; /* 命令数据 */

  /* 离线后只发送一次Offline，待下一次有效帧到来再解除 */
  bool offline_latched_ = false;

  bool dial_up_active_ = false;
  bool dial_up_long_triggered_ = false;
  LibXR::MillisecondTimestamp dial_up_touch_start_{};

  LibXR::UART* uart_;                       /* UART接口指针 */
  LibXR::Event vt13_event_;                 /* 事件处理器 */
  LibXR::Thread thread_uart_;               /* UART线程 */
  LibXR::Semaphore sem_;                    /* 读操作信号量 */
  LibXR::ReadOperation op_;                 /* 读操作（阻塞型） */
  LibXR::MillisecondTimestamp last_time_{}; /* 上次接收时间 */

  /*--------------------------工具函数-------------------------------------------------*/

  /**
   * @brief 从任意位偏移提取指定位宽数据
   * @param raw_data 原始字节流
   * @param bit_offset 起始bit偏移
   * @param bit_len 提取bit长度
   * @return 提取后的无符号值
   */
  static uint32_t ExtractBits(const uint8_t* raw_data, uint16_t bit_offset,
                              uint8_t bit_len) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < bit_len; ++i) {
      const uint16_t CURRENT_BIT = bit_offset + i;
      const uint8_t BIT = static_cast<uint8_t>(
          (raw_data[CURRENT_BIT / 8] >> (CURRENT_BIT % 8)) & 0x01u);
      value |= static_cast<uint32_t>(BIT) << i;
    }
    return value;
  }

  /**
   * @brief 读取小端16位整数
   */
  static uint16_t ReadLe16(const uint8_t* raw_data, std::size_t offset) {
    return static_cast<uint16_t>(raw_data[offset]) |
           (static_cast<uint16_t>(raw_data[offset + 1]) << 8);
  }

  /**
   * @brief 校验VT13帧CRC
   * @param raw_data 21字节原始帧
   * @return true 校验通过
   * @return false 校验失败
   */
  bool VerifyCRC(const uint8_t* raw_data) {
    constexpr std::size_t CRC_OFFSET = VT13_PAYLOAD_SIZE_FOR_CRC;
    /* 按官方协议：CRC覆盖前19字节，帧尾2字节为小端CRC16 */
    const uint16_t calc_crc =
        LibXR::CRC16::Calculate(raw_data, VT13_PAYLOAD_SIZE_FOR_CRC);
    const uint16_t frame_crc_le = ReadLe16(raw_data, CRC_OFFSET);
    return calc_crc == frame_crc_le;
  }

  /**
   * @brief 拨轮触碰事件
   * @details
   * 上拨短触发/上拨长触发/下拨触发。
   */
  void ActiveDialTouchEvent(uint16_t curr_dial) {
    constexpr uint16_t DIAL_UP_THRESHOLD = VT13_CH_VALUE_MID + 180u;
    constexpr uint16_t DIAL_DOWN_THRESHOLD = VT13_CH_VALUE_MID - 180u;
    constexpr uint32_t DIAL_LONG_TOUCH_MS = 500u;

    const bool DIAL_UP = curr_dial >= DIAL_UP_THRESHOLD;
    const bool LAST_DIAL_UP = this->last_data_.dial >= DIAL_UP_THRESHOLD;
    const bool DIAL_DOWN = curr_dial <= DIAL_DOWN_THRESHOLD;
    const bool LAST_DIAL_DOWN = this->last_data_.dial <= DIAL_DOWN_THRESHOLD;
    const auto CURRENT_TIME = LibXR::Timebase::GetMilliseconds();

    if (DIAL_UP && !LAST_DIAL_UP) {
      this->dial_up_active_ = true;
      this->dial_up_long_triggered_ = false;
      this->dial_up_touch_start_ = CURRENT_TIME;
    }

    if (this->dial_up_active_ && DIAL_UP && !this->dial_up_long_triggered_) {
      const uint32_t TOUCH_MS =
          (CURRENT_TIME - this->dial_up_touch_start_).ToMillisecond();
      if (TOUCH_MS >= DIAL_LONG_TOUCH_MS) {
        this->vt13_event_.Active(
            static_cast<uint32_t>(SwitchPos::VT13_DIAL_UP_LONG));
        this->dial_up_long_triggered_ = true;
      }
    }

    if (!DIAL_UP && LAST_DIAL_UP && this->dial_up_active_) {
      if (!this->dial_up_long_triggered_) {
        this->vt13_event_.Active(
            static_cast<uint32_t>(SwitchPos::VT13_DIAL_UP_SHORT));
      }
      this->dial_up_active_ = false;
      this->dial_up_long_triggered_ = false;
    }

    if (DIAL_DOWN && !LAST_DIAL_DOWN) {
      this->vt13_event_.Active(
          static_cast<uint32_t>(SwitchPos::VT13_DIAL_DOWN_TOUCH));
    }
  }

  /**
   * @brief 在线状态巡检
   * @details 连续100ms未收到有效帧即判定离线
   */
  void CheckoutOffline() {
    auto current_time = LibXR::Timebase::GetMilliseconds();
    if ((current_time - this->last_time_).ToMillisecond() > 100) {
      /*离线窗口内只触发一次Offline，避免重复下发相同离线数据 */
      if (!this->offline_latched_) {
        this->Offline();
      }
    }
  }
};
