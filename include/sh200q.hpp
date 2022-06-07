#pragma once
#include <ili9341.hpp>
#include <Wire.h>
namespace arduino {

enum struct sh200q_acc_scale {
    scale_4g = 0,
    scale_8g,
    scale_16g
  };

  enum struct sh200q_gyro_scale {
    scale_2000dps = 0,
    scale_1000dps,
    scale_500dps,
    scale_250dps,
    scale_125dps
  };

class sh200q {
    constexpr static const int8_t address = 0x6C;
    constexpr static const uint8_t op_whoami = 0x30;
    constexpr static const uint8_t op_acc_config = 0x0E;
    constexpr static const uint8_t op_gyro_config = 0x0F;
    constexpr static const uint8_t op_gyro_dlpf = 0x11;
    constexpr static const uint8_t op_fifi_config = 0x12;
    constexpr static const uint8_t op_acc_range = 0x16;
    constexpr static const uint8_t op_gyro_range = 0x2B;
    constexpr static const uint8_t op_output_acc = 0x00;
    constexpr static const uint8_t op_output_gyro = 0x06;
    constexpr static const uint8_t op_output_temp = 0x0C;
    constexpr static const uint8_t op_reg_set1 = 0xBA;
    constexpr static const uint8_t op_reg_set2 = 0xCA;
    constexpr static const uint8_t op_adc_reset = 0xC2;
    constexpr static const uint8_t op_soft_reset = 0x7F;
    constexpr static const uint8_t op_reset = 0x75;
    constexpr static const double rta = 57.324841;
    constexpr static const double atr = 0.0174533;
    constexpr static const double gyro_gr = 0.0010653;
    TwoWire& m_i2c;
    bool m_initialized;
    sh200q_acc_scale m_acc_scale;
    sh200q_gyro_scale m_gyro_scale;
    float m_ares;
    float m_gres;
    void adc_reset();
    void update_gres();
    void update_ares();
public:
    inline sh200q(TwoWire& i2c) : m_i2c(i2c),m_initialized(false) {
    }
    inline bool initialized() const { return m_initialized; }
    bool initialize();
    inline sh200q_acc_scale acc_scale() const { return m_acc_scale; }
    void acc_scale(sh200q_acc_scale value);
    inline sh200q_gyro_scale gyro_scale() const { return m_gyro_scale; }
    void gyro_scale(sh200q_gyro_scale value);
    void acc(float* out_x, float* out_y, float* out_z);
    void gyro(float* out_x, float* out_y, float* out_z);
    void ahrs(float* out_pitch,float* out_yaw, float* out_roll);
    float temp();
};
}