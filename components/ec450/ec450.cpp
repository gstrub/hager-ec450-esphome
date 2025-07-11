#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "ec450.h"
#include <cstring>
// Code inspired from the CSE7766 component
// https://github.com/esphome/esphome/blob/dev/esphome/components/cse7766/cse7766.cpp#L11

// For storing energy, see https://github.com/esphome/esphome/blob/dev/esphome/components/globals/globals_component.h

namespace esphome {
namespace ec450 {

#define EC450_BUSY 0xB5
#define EC450_EOF 0xC0

#define EC450_MSG_ID_VOLTAGE 0x0D
#define EC450_MSG_ID_CURRENT 0x0E
#define EC450_MSG_ID_POWER 0x0F
#define EC450_MSG_ID_ENERGY_DELTA 0x10

static const char *TAG = "ec450.component";

void EC450::setup() {

}

void EC450::loop() {
    uint32_t now_ms = App.get_loop_component_start_time();
    if (now_ms - this->last_transmission_ms_ > 50) {
      // Last transmission was too long ago, reset parser
    }

    if (this->available() == 0) {
      return;
    }

    this->last_transmission_ms_ = now_ms;
    while (this->available() != 0) {
      uint8_t length;
      this->peek_byte(&length);

      if (length == 0 || length == EC450_BUSY || length >= EC450_MAX_MSG_LENGTH) {
        // Drop "busy" bytes
        this->read_byte(&length);
        if (length != EC450_BUSY)
          ESP_LOGI(TAG, "Dropped byte %02X", length);
        continue;
      }
        
      if (this->available() < length + 2) {
        // Not enough bytes yet, continue later
        break;
      }

      // Now we have a full message in the RX buffer, copy it to our internal structure and check it
      this->read_array(this->message_buffer_, length + 2);

      // Check that the message indeed ends with a EOF byte
      if (this->message_buffer_[length + 1] != EC450_EOF) {
        ESP_LOGW(TAG, "Incorrect EOF %02X", this->message_buffer_[length + 1]);
        debug_print_hex_();
        continue;
      }
      
      // Validate message checksum
      uint8_t checksum = 0;
      for (uint8_t i = 1; i < length; i++) {
        checksum += this->message_buffer_[i];
      }
      if (checksum != this->message_buffer_[length]) {
        ESP_LOGW(TAG, "Invalid checksum %02X (expect: %02X)", checksum, this->message_buffer_[length]);
        debug_print_hex_();
        continue;
      }

      // Made it up to here ? parse message !
      this->parse_message_();

    }
}

void EC450::parse_message_() {
  uint8_t msg_id = this->message_buffer_[1];
  switch (msg_id) {
    case EC450_MSG_ID_VOLTAGE: 
    this->parse_voltage_();
    break;

    case EC450_MSG_ID_CURRENT:
    this->parse_current_();
    break;

    case EC450_MSG_ID_POWER:
    this->parse_power_();
    break;

    case EC450_MSG_ID_ENERGY_DELTA:
    this->parse_energy_();
    break;

    default:
    debug_print_hex_();
  }
}

void EC450::parse_voltage_() {
  voltage_V_ = (float)(this->parse_uint16_(2)) / 100.0f;
  if (this->voltage_sensor_ != nullptr) {
    this->voltage_sensor_->publish_state(voltage_V_);
  }
  ESP_LOGI(TAG, "Voltage : %0.2fV", voltage_V_);
}

void EC450::parse_power_() {
  for (int i=0; i < 6; i++) {
    power_W_[i] = this->parse_uint32_(2 + 4*i);
    if (i > 0 && this->power_sensor_[i-1] != nullptr) {
      this->power_sensor_[i-1]->publish_state(power_W_[i]);
    }
  } 
  ESP_LOGI(TAG, "Power : %dW %dW %dW %dW %dW %dW", power_W_[0], power_W_[1], power_W_[2], power_W_[3], power_W_[4], power_W_[5]);
}

void EC450::parse_current_() {
  for (int i=0; i < 6; i++) {
    current_A_[i] = (float)(this->parse_uint16_(2 + 2*i)) / 100.0f;
    if (i > 0 && this->current_sensor_[i-1] != nullptr) {
      this->current_sensor_[i-1]->publish_state(current_A_[i]);
    }
  }  
  ESP_LOGI(TAG, "Current : %0.2fA %0.2fA %0.2fA %0.2fA %0.2fA %0.2fA", current_A_[0], current_A_[1], current_A_[2], current_A_[3], current_A_[4], current_A_[5]);
}

void EC450::parse_energy_() {
  uint32_t energy_delta[6];
  for (int i=0; i < 6; i++) {
    energy_delta[i] = this->parse_uint32_(2 + 4*i);
    energy_total_100uWh_[i] += energy_delta[i];
    if (i > 0 && this->energy_sensor_[i-1] != nullptr) {
      this->energy_sensor_[i-1]->publish_state((float)(energy_total_100uWh_[i]) / 10000.0f);
    }
  } 
  ESP_LOGI(TAG, "Energy Delta : %.4fWh %.4fWh %.4fWh %.4fWh %.4fWh %.4fWh", 
    (float)(energy_delta[0])/10000.0f, 
    (float)(energy_delta[1])/10000.0f, 
    (float)(energy_delta[2])/10000.0f, 
    (float)(energy_delta[3])/10000.0f, 
    (float)(energy_delta[4])/10000.0f, 
    (float)(energy_delta[5])/10000.0f);
  ESP_LOGI(TAG, "Energy : %.4fWh %.4fWh %.4fWh %.4fWh %.4fWh %.4fWh", 
    (float)(energy_total_100uWh_[0])/10000.0f, 
    (float)(energy_total_100uWh_[1])/10000.0f, 
    (float)(energy_total_100uWh_[2])/10000.0f, 
    (float)(energy_total_100uWh_[3])/10000.0f, 
    (float)(energy_total_100uWh_[4])/10000.0f, 
    (float)(energy_total_100uWh_[5])/10000.0f);
}

uint32_t EC450::parse_uint32_(uint8_t index) {
  return (this->message_buffer_[index] << 24) + 
         (this->message_buffer_[index + 1] << 16) + 
         (this->message_buffer_[index + 2] << 8) + 
         (this->message_buffer_[index + 3] << 0);
}
uint16_t EC450::parse_uint16_(uint8_t index) {
  return (this->message_buffer_[index] << 8) + this->message_buffer_[index + 1];
}

void EC450::debug_print_hex_() {
  std::string res;

  size_t len = this->message_buffer_[0] + 2;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (i > 0) {
      res += " ";
    }
    sprintf(buf, "%02X", this->message_buffer_[i]);
    res += buf;
  }
  ESP_LOGD(TAG, "%s", res.c_str());
  delay(10);
}

void EC450::dump_config(){
    ESP_LOGCONFIG(TAG, "EC450");
    this->check_uart_settings(19200, 1, uart::UART_CONFIG_PARITY_NONE);
}

}  // namespace ec450
}  // namespace esphome