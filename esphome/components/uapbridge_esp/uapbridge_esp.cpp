#include "uapbridge_esp.h"

namespace esphome {
namespace uapbridge_esp {

static const char *const TAG = "uapbridge_esp";

/* ============================================================
   LOOP
============================================================ */

void UAPBridge_esp::loop()
{
  loop_fast();
  loop_slow();

  if (data_has_changed)
  {
    clear_data_changed_flag();
    state_callback_.call();
  }
}

/* ============================================================
   FAST LOOP
============================================================ */

void UAPBridge_esp::loop_fast()
{
  receive();

  if (millis() - last_call < CYCLE_TIME)
    return;

  last_call = millis();

  if (send_time != 0 && millis() >= send_time)
  {
    transmit();
    send_time = 0;
  }
}

/* ============================================================
   SLOW LOOP
============================================================ */

void UAPBridge_esp::loop_slow()
{
  uint32_t now = millis();

  if (now - last_call_slow < CYCLE_TIME_SLOW)
    return;

  last_call_slow = now;

  if (!auto_correction)
    return;

  bool error_now = broadcast_status & hoermann_state_error;

  if (!error_now)
  {
    auto_correction_executed = false;
    return;
  }

  if (!last_error_state)
    error_start_time = now;

  last_error_state = error_now;

  if (now - error_start_time < 3000)
    return;

  if (auto_correction_executed)
    return;

  if (state == hoermann_state_open)
    set_command(true, hoermann_action_open);
  else if (state == hoermann_state_closed)
    set_command(true, hoermann_action_close);

  auto_correction_executed = true;
}

/* ============================================================
   RECEIVE
============================================================ */

void UAPBridge_esp::receive()
{
  bool newData = false;

  while (available() > 0)
  {
    for (uint8_t i = 0; i < 4; i++)
      rx_data[i] = rx_data[i + 1];

    if (read_byte(&rx_data[4]))
      byte_cnt++;

    newData = true;
  }

  if (!newData)
    return;

  if (rx_data[0] == BROADCAST_ADDR)
  {
    uint8_t length = rx_data[1] & 0x0F;

    if (length == 2 &&
        calc_crc8(rx_data, length + 3) == 0x00)
    {
      broadcast_status =
        rx_data[2] |
        ((uint16_t)rx_data[3] << 8);
    }
  }
}

/* ============================================================
   TRANSMIT
============================================================ */

void UAPBridge_esp::transmit()
{
  if (rts_pin_ != nullptr)
    rts_pin_->digital_write(true);

  parent_->set_baud_rate(9600);
  parent_->set_data_bits(7);
  parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  parent_->set_stop_bits(1);
  parent_->load_settings(false);

  write_byte(0x00);
  flush();

  parent_->set_baud_rate(19200);
  parent_->set_data_bits(8);
  parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  parent_->set_stop_bits(1);
  parent_->load_settings(false);

  write_array(tx_data, tx_length);
  flush();

  if (rts_pin_ != nullptr)
    rts_pin_->digital_write(false);
}

/* ============================================================
   STATE HELPERS (LINKER SAFE)
============================================================ */

void UAPBridge_esp::set_light(bool state)
{
  if (light_enabled == state)
    return;

  set_command(true, hoermann_action_toggle_light);
}

void UAPBridge_esp::set_venting(bool enable)
{
  if (enable)
    set_command(true, hoermann_action_venting);
  else
    set_command(true, hoermann_action_close);
}

void UAPBridge_esp::action_impulse()
{
  set_command(true, hoermann_action_impulse);
}

hoermann_state_t UAPBridge_esp::get_state()
{
  return state;
}

std::string UAPBridge_esp::get_state_string()
{
  return state_string;
}

/* ============================================================
   STATE MACHINE
============================================================ */

void UAPBridge_esp::handle_state_change(hoermann_state_t new_state)
{
  state = new_state;

  switch (new_state)
  {
    case hoermann_state_open:
      state_string = "Open";
      break;

    case hoermann_state_closed:
      state_string = "Closed";
      break;

    case hoermann_state_opening:
      state_string = "Opening";
      break;

    case hoermann_state_closing:
      state_string = "Closing";
      break;

    case hoermann_state_venting:
      state_string = "Venting";
      break;

    case hoermann_state_stopped:
      state_string = "Stopped";
      break;

    default:
      state_string = "Error";
      break;
  }

  data_has_changed = true;
}

} // namespace uapbridge_esp
} // namespace esphome
