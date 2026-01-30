#include "uapbridge_esp.h"

namespace esphome {
namespace uapbridge_esp {
static const char *const TAG = "uapbridge_esp";

// Alle originele functies blijven EXACT hetzelfde...
void UAPBridge_esp::loop() {
  this->loop_fast();
  this->loop_slow();
  if (this->data_has_changed) {
    ESP_LOGD(TAG, "UAPBridge_esp::loop() - received Data has changed.");
    this->clear_data_changed_flag();
    this->state_callback_.call();
  }
}

// ... (loop_fast, loop_slow, transmit, set_command, actions, etc blijven ORIGINEEL)

void UAPBridge_esp::receive() {
  uint8_t   length  = 0;
  uint8_t   counter = 0;
  bool   newData = false;
  
  while (this->available() > 0) {
    // ORIGINELE shift logic
    for (uint8_t i = 0; i < 4; i++) {
      this->rx_data[i] = this->rx_data[i+1];
    }
    if(this->read_byte(&this->rx_data[4])){
      this->byte_cnt++;
    }
    newData = true;
  }
  
  if (newData) {
    ESP_LOGVV(TAG, "new data received");
    
    // 🔥 MOD 1: ESP32 Slave Scan (0x81 ipv 0x28)
    if (this->rx_data[0] == ESP32_SLAVE_ADDR) {  // ← ENKEL DIT GEWIJZIGD!
      length = this->rx_data[1] & 0x0F;
      if (this->rx_data[2] == CMD_SLAVE_SCAN && this->rx_data[3] == UAP1_ADDR_MASTER && length == 2 && calc_crc8(this->rx_data, length + 3) == 0x00) {
        ESP_LOGVV(TAG, "SlaveScan: %s", print_data(this->rx_data, 0, 5));
        ESP_LOGV(TAG, "->      SlaveScan"); 
        counter = (this->rx_data[1] & 0xF0) + 0x10;
        this->tx_data[0] = UAP1_ADDR_MASTER;
        this->tx_data[1] = 0x02 | counter;
        this->tx_data[2] = UAP1_TYPE;
        this->tx_data[3] = ESP32_SLAVE_ADDR;  // ← ENKEL DIT GEWIJZIGD!
        this->tx_data[4] = calc_crc8(this->tx_data, 4);
        this->tx_length = 5;
        this->send_time = millis();
      }
    }
    
    // Broadcast status (ORIGINEEL)
    if (this->rx_data[0] == BROADCAST_ADDR) {
      length = this->rx_data[1] & 0x0F;
      if (length == 2 && calc_crc8(this->rx_data, length + 3) == 0x00) {
        ESP_LOGVV(TAG, "Broadcast: %s", print_data(this->rx_data, 0, 5));
        ESP_LOGV(TAG, "->      Broadcast");
        this->broadcast_status = this->rx_data[2];
        this->broadcast_status |= (uint16_t)this->rx_data[3] << 8;
      }
    }
    
    // 🔥 MOD 2: ESP32 Status Request (0x81 ipv 0x28)  
    if (this->rx_data[1] == ESP32_SLAVE_ADDR) {  // ← ENKEL DIT GEWIJZIGD!
      length = this->rx_data[2] & 0x0F;
      if (this->rx_data[3] == CMD_SLAVE_STATUS_REQUEST && length == 1 && calc_crc8(&this->rx_data[1], length + 3) == 0x00) {
        ESP_LOGVV(TAG, "Slave status request: %s", print_data(this->rx_data, 1, 5));
        ESP_LOGV(TAG, "->      Slave status request");
        counter = (this->rx_data[2] & 0xF0) + 0x10;
        this->tx_data[0] = UAP1_ADDR_MASTER;
        this->tx_data[1] = 0x03 | counter;
        this->tx_data[2] = CMD_SLAVE_STATUS_RESPONSE;
        this->tx_data[3] = (uint8_t)(this->next_action & 0xFF);
        this->tx_data[4] = (uint8_t)((this->next_action >> 8) & 0xFF);
        this->next_action = hoermann_action_none;
        this->tx_data[5] = calc_crc8(this->tx_data, 5);
        this->tx_length = 6;
        this->send_time = millis();
      }
    }
    
    // Rest ORIGINEEL...
    if (!this->valid_broadcast && (this->rx_data[3] != 0 || this->rx_data[4] != 0)) {
      this->valid_broadcast = true;
      this->data_has_changed = true;
    }
  }
}

// transmit() ORIGINEEL (GEEN wijzigingen!)
void UAPBridge_esp::transmit() {
  ESP_LOGVV(TAG, "Transmit: %s", print_data(this->tx_data, 0, this->tx_length));
  if (this->rts_pin_ != nullptr) {
    this->rts_pin_->digital_write(true);
  }
  this->parent_->set_baud_rate(9600);
  this->parent_->set_data_bits(7);
  this->parent_->set_parity(esphome::uart::UARTParityOptions::UART_CONFIG_PARITY_NONE);  // ← ORIGINEEL
  this->parent_->set_stop_bits(1);
  this->parent_->load_settings(false);
  this->write_byte(0x00);
  this->flush();

  this->parent_->set_baud_rate(19200);
  this->parent_->set_data_bits(8);
  this->parent_->set_parity(esphome::uart::UARTParityOptions::UART_CONFIG_PARITY_NONE);  // ← ORIGINEEL
  this->parent_->set_stop_bits(1);
  this->parent_->load_settings(false);
  this->write_array(this->tx_data, this->tx_length);
  this->flush();

  if (this->rts_pin_ != nullptr) {
    this->rts_pin_->digital_write(false);
  }
  ESP_LOGVV(TAG, "TX duration: %dms", millis() - this->send_time);
}

// Alle andere functies ORIGINEEL (set_command, actions, calc_crc8, etc)...
