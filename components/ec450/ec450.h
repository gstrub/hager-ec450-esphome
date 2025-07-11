#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace ec450 {

#define EC450_MAX_MSG_LENGTH 0x30

class EC450 : public uart::UARTDevice, public Component {
  public:
    void setup() override;
    void loop() override;
    void dump_config() override;

    void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
    void set_current_sensor(uint8_t i, sensor::Sensor *current_sensor) { if (i >= 5) return; current_sensor_[i] = current_sensor; }
    void set_power_sensor(uint8_t i, sensor::Sensor *power_sensor) { if (i >= 5) return; power_sensor_[i] = power_sensor; }
    void set_energy_sensor(uint8_t i, sensor::Sensor *energy_sensor) { if (i >= 5) return; energy_sensor_[i] = energy_sensor; }

  protected:
    uint8_t message_buffer_[EC450_MAX_MSG_LENGTH + 2];
    uint32_t last_transmission_ms_{0};

    uint64_t energy_total_100uWh_[6] {0, 0, 0, 0, 0, 0};  // In 1e-4 Wh
    uint32_t power_W_[6] {0, 0, 0, 0, 0, 0};
    float current_A_[6] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float voltage_V_{0.0f};

    void parse_message_();
    void parse_voltage_();
    void parse_power_();
    void parse_current_();
    void parse_energy_();
    uint32_t parse_uint32_(uint8_t index);
    uint16_t parse_uint16_(uint8_t index);
    void debug_print_hex_();

    sensor::Sensor *voltage_sensor_{nullptr};
    sensor::Sensor *current_sensor_[5]{nullptr, nullptr, nullptr, nullptr, nullptr};
    sensor::Sensor *power_sensor_[5]{nullptr, nullptr, nullptr, nullptr, nullptr};
    sensor::Sensor *energy_sensor_[5]{nullptr, nullptr, nullptr, nullptr, nullptr};
};


}  // namespace ec450
}  // namespace esphome