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
    if(auto_correction) {
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
    bool framing_error = this->uart_->has_framing_error();

    if((crc_error || framing_error) && !recovery_mode) {
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

    // --- Originele UAP1 receive handling blijft intact ---
  }
}

// --- ESP32 Slave Functions (send_time queued) ---
void UAPBridge_esp::make_scan_response_msg() {
  tx_data[0] = ESP32_SLAVE_ADDR;  // 0x81
  tx_data[1] = ESP32_TYPE;        // 0x21
  tx_data[2] = calc_crc8(tx_data, 2);
  tx_length = 3;
  send_time = millis() + 10;      // queue response
  ESP_LOGD(TAG, "ESP32 scan response queued (0x81)");
}

void UAPBridge_esp::make_status_response_msg() {
  tx_data[0] = ESP32_SLAVE_ADDR;  // 0x81
  tx_data[1] = 0x00;              // status byte (idle/placeholder)
  tx_data[2] = calc_crc8(tx_data, 2);
  tx_length = 3;
  send_time = millis() + 10;      // queue response
  ESP_LOGD(TAG, "ESP32 status response queued (0x81)");
}

} // namespace uapbridge_esp
} // namespace esphome
