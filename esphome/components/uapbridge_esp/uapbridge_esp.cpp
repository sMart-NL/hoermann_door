#include "uapbridge_esp.h"

namespace esphome {
namespace uapbridge_esp {

static const char *const TAG = "uapbridge_esp";

void UAPBridge_esp::loop() {
  this->loop_fast();
  this->loop_slow();

  if(this->data_has_changed) {
    ESP_LOGD(TAG, "UAPBridge_esp::loop() - received Data has changed.");
    this->clear_data_changed_flag();
    this->state_callback_.call();
  }
}

void UAPBridge_esp::loop_fast() {
  this->receive();

  if(millis() - this->last_call < CYCLE_TIME) return;
  this->last_call = millis();

  if(this->send_time != 0 && millis() >= this->send_time) {
    ESP_LOGVV(TAG, "loop: transmitting");
    this->transmit();
    this->send_time = 0;
  }
}

void UAPBridge_esp::loop_slow() {
  // --- Broadcast timeout ---
  if(millis() - last_broadcast_seen > 5000) {
    got_valid_broadcast = false;
  }

  if(millis() - this->last_call_slow < CYCLE_TIME_SLOW) return;
  this->last_call_slow = millis();

  if(this->ignore_next_event) {
    this->ignore_next_event = false;
  } else {
    hoermann_state_t new_state = hoermann_state_stopped;

    if(this->broadcast_status & hoermann_state_open) new_state = hoermann_state_open;
    else if(this->broadcast_status & hoermann_state_closed) new_state = hoermann_state_closed;
    else if((this->broadcast_status & (hoermann_state_direction | hoermann_state_moving)) == hoermann_state_opening) new_state = hoermann_state_opening;
    else if((this->broadcast_status & (hoermann_state_direction | hoermann_state_moving)) == hoermann_state_closing) new_state = hoermann_state_closing;
    else if(this->broadcast_status & hoermann_state_venting) new_state = hoermann_state_venting;

    if(new_state != this->state) this->handle_state_change(new_state);

    this->update_boolean_state("relay", this->relay_enabled, (this->broadcast_status & hoermann_state_opt_relay));
    this->update_boolean_state("light", this->light_enabled, (this->broadcast_status & hoermann_state_light_relay));
    this->update_boolean_state("vent", this->venting_enabled, (this->broadcast_status & hoermann_state_venting));
    this->update_boolean_state("err", this->error_state, (this->broadcast_status & hoermann_state_error));
    this->update_boolean_state("prewarn", this->prewarn_state, (this->broadcast_status & hoermann_state_prewarn));

    // --- Auto Error Correction ---
    if(auto_correction_) {
      if((this->broadcast_status & hoermann_state_error) == hoermann_state_error) {
        ESP_LOGD(TAG, "autocorrection started");
        if(new_state == hoermann_state_open) this->set_command(true, hoermann_action_open);
        else if(new_state == hoermann_state_closed) this->set_command(true, hoermann_action_close);
        else if(new_state == hoermann_state_stopped) this->auto_correction_in_progress = false;
        this->auto_correction_in_progress = true;
      }
      if(this->auto_correction_in_progress && (this->broadcast_status & hoermann_state_light_relay)) {
        this->set_command(true, hoermann_action_toggle_light);
        this->auto_correction_in_progress = false;
      }
    }

    // --- Recovery Stop ---
    if(recovery_mode && state == hoermann_state_stopped && got_valid_broadcast) {
      if(millis() - last_stop_sent > 1500) {
        action_stop();
        recovery_mode = false;
        got_valid_broadcast = false;
        ESP_LOGI(TAG, "✓ Auto-STOP recovery complete");
      }
    }
  }
}

void UAPBridge_esp::receive() {
  bool newData = false;
  while(this->uart_->available() > 0) {
    for(uint8_t i = 0; i < 4; i++) rx_data[i] = rx_data[i+1];
    if(this->uart_->read_byte(&rx_data[4])) byte_cnt++;
    newData = true;
  }

  if(newData) {
    bool crc_error = (calc_crc8(rx_data,4) != rx_data[4]);
    // framing_error check VERWIJDERD voor ESPHome 2026.1.1 compatibiliteit

    if(crc_error && !recovery_mode) {
      recovery_mode = true;
      last_stop_sent = millis();
      ESP_LOGW(TAG, "RS485 error → Recovery active");
    }

    // --- Broadcast detectie ---
    if(rx_data[0] == BROADCAST_ADDR && rx_data[1] == CMD_SLAVE_STATUS_REQUEST) {
      got_valid_broadcast = true;
      last_broadcast_seen = millis();
    }

    // --- ESP32 Slave 0x81 Responses ---
    if(rx_data[0] == MASTER_ADDR) {
      if(rx_data[1] == CMD_SLAVE_SCAN) {
        make_scan_response_msg();
      } else if(rx_data[1] == CMD_SLAVE_STATUS_REQUEST) {
        make_status_response_msg();
      }
    }
  }
}

void UAPBridge_esp::make_scan_response_msg() {
  tx_data[0] = ESP32_SLAVE_ADDR;  // 0x81 ← JOUW WIJZIGING!
  tx_data[1] = ESP32_TYPE;        // 0x21
  tx_data[2] = calc_crc8(tx_data, 2);
  tx_length = 3;
  send_time = millis() + 10;
  ESP_LOGD(TAG, "ESP32 scan response queued (0x81)");
}

void UAPBridge_esp::make_status_response_msg() {
  tx_data[0] = ESP32_SLAVE_ADDR;  // 0x81 ← JOUW WIJZIGING!
  tx_data[1] = 0x00;
  tx_data[2] = calc_crc8(tx_data, 2);
  tx_length = 3;
  send_time = millis() + 10;
  ESP_LOGD(TAG, "ESP32 status response queued (0x81)");
}

// Transmit functie (ongewijzigd van origineel)
void UAPBridge_esp::transmit() {
  ESP_LOGVV(TAG, "Transmit: %s", print_data(this->tx_data, 0, this->tx_length));
  if (this->rts_pin_ != nullptr) {
    this->rts_pin_->digital_write(true);
  }
  // Generate Sync break
  this->uart_->set_baud_rate(9600);
  this->uart_->set_data_bits(7);
  this->uart_->set_parity(esphome::uart::UARTParityOptions::UART_CONFIG_PARITY_NONE);
  this->uart_->set_stop_bits(1);
  this->uart_->load_settings(false);
  this->uart_->write_byte(0x00);
  this->uart_->flush();

  // Transmit
  this->uart_->set_baud_rate(19200);
  this->uart_->set_data_bits(8);
  this->uart_->set_parity(esphome::uart::UARTParityOptions::UART_CONFIG_PARITY_NONE);
  this->uart_->set_stop_bits(1);
  this->uart_->load_settings(false);
  this->uart_->write_array(this->tx_data, this->tx_length);
  this->uart_->flush();

  if (this->rts_pin_ != nullptr) {
    this->rts_pin_->digital_write(false);
  }

  ESP_LOGVV(TAG, "TX duration: %dms", millis() - this->send_time);
}

void UAPBridge_esp::set_command(bool cond, const hoermann_action_t command) {
  if (cond) {
    if (this->next_action != hoermann_action_none) {
      ESP_LOGW(TAG, "Last Command was not yet fetched by modbus! -- action cached %d", this->next_action);
    } else {
      this->next_action = command;
      this->ignore_next_event = true;
    }
  }
}

void UAPBridge_esp::action_open() {
  ESP_LOGD(TAG, "Action: open called");
  this->set_command(this->state != hoermann_state_open, hoermann_action_open);
}

void UAPBridge_esp::action_close() {
  ESP_LOGD(TAG, "Action: close called");
  this->set_command(this->state != hoermann_state_closed, hoermann_action_close);
}

void UAPBridge_esp::action_stop() {
  ESP_LOGD(TAG, "Action: stop called");
  this->set_command((this->state == hoermann_state_opening || this->state == hoermann_state_closing), hoermann_action_stop);
}

void UAPBridge_esp::action_venting() {
  ESP_LOGD(TAG, "Action: venting called");
  this->set_command(this->state != hoermann_state_venting, hoermann_action_venting);
}

void UAPBridge_esp::action_toggle_light() {
  ESP_LOGD(TAG, "Action: toggle light called");
  this->set_command(true, hoermann_action_toggle_light);
}

void UAPBridge_esp::action_impulse() {
  ESP_LOGD(TAG, "Action: impulse called");
  this->set_command(true, hoermann_action_impulse);
}

UAPBridge_esp::hoermann_state_t UAPBridge_esp::get_state() {
  return this->state;
}

std::string UAPBridge_esp::get_state_string() {
  return this->state_string;
}

void UAPBridge_esp::set_venting(bool state) {
  if (state) {
    this->action_venting();
  } else {
    this->action_close();
  }
  ESP_LOGD(TAG, "Venting state set to %s", state ? "ON" : "OFF");
}

void UAPBridge_esp::set_light(bool state) {
  this->set_command((this->light_enabled != state), hoermann_action_toggle_light);
  ESP_LOGD(TAG, "Light state set to %s", state ? "ON" : "OFF");
}

uint8_t UAPBridge_esp::calc_crc8(uint8_t *p_data, uint8_t length) {
  uint8_t i;
  uint8_t data;
  uint8_t crc = 0xF3;
  
  for(i = 0; i < length; i++) {
    data = *p_data ^ crc;
    p_data++;
    crc = crc_table[data];
  }
  return crc;
}

char* UAPBridge_esp::print_data(uint8_t *p_data, uint8_t from, uint8_t to) {
  char temp[4];
  static char output[30];

  sprintf(output, "%5lu: ", millis() & 0xFFFFul);
  for (uint8_t i = from; i < to; i++) {
    sprintf(temp, "%02X ", p_data[i]);
    strcat(output, temp);
  }
  this->byte_cnt = 0;
  return &output[0];
}

void UAPBridge_esp::handle_state_change(hoermann_state_t new_state) {
  this->state = new_state;
  ESP_LOGV(TAG, "State changed from %s to %d", this->state_string.c_str(), new_state);
  switch (new_state) {
    case hoermann_state_open:
      this->state_string = "Open";
      break;
    case hoermann_state_closed:
      this->state_string = "Closed";
      break;
    case hoermann_state_opening:
      this->state_string = "Opening";
      break;
    case hoermann_state_closing:
      this->state_string = "Closing";
      break;
    case hoermann_state_venting:
      this->state_string = "Venting";
      break;
    case hoermann_state_stopped:
      this->state_string = "Stopped";
      break;
    default:
      this->state_string = "Error";
      break;
  }
  this->data_has_changed = true;
}

void UAPBridge_esp::update_boolean_state(const char * name, bool &current_state, bool new_state) {
  ESP_LOGV(TAG, "update_boolean_state: %s from %s to %s", name, current_state ? "true" : "false", new_state ? "true" : "false");
  if (current_state != new_state) {
    current_state = new_state;
    this->data_has_changed = true;
  }
}

} // namespace uapbridge_esp
} // namespace esphome
