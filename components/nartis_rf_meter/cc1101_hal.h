/*
 * CC1101 Hardware Abstraction Layer (CMT-compatible interface)
 *
 * Drives the CC1101 transceiver over ESP-IDF hardware SPI using the vendored
 * nopnop2002/esp-idf-cc1101 driver (cc1101_vendored.*).  This file is the sole
 * active radio implementation for the ESPHome component; the public surface
 * below is the contract that nartis_rf_meter.cpp codes against and must not
 * change.
 *
 * Pin mapping kept for the YAML/config contract:
 *   SDIO  - MISO
 *   SCLK  - SPI clock
 *   CSB   - chip select (CSN, active low)
 *   FCSB  - MOSI
 *   GPIO3 - GDO0 (packet-received interrupt line)
 *
 */

#pragma once

#include "nartis_rf_constants.h"
#include "cc1101_vendored.h"
#include <driver/gpio.h>
#include "esphome/core/hal.h"  // millis()/delay() without <Arduino.h>
#include "cc1101_rx_stream.h"

namespace esphome::nartis_rf_meter {
struct RfPhyProfile {
  float carrier_mhz;
  float bitrate_kbps;
  float deviation_khz;
  float rx_bandwidth_khz;
  bool sync_enabled;
  bool ook{false};  // on-off keying demod (meter remotes are often ASK)
};

class Cc1101Hal {
 public:
  void set_pins(int, int, int, int, int);
  bool init(); bool is_chip_connected(); bool go_standby(); bool go_sleep(); bool go_rx(); bool go_tx();
  void log_init_diagnostics() const;
  uint8_t get_state(); bool wait_for_state(uint8_t, uint32_t = RF_STATE_POLL_TIMEOUT_MS);
  // CC1101 has one carrier register: TX and RX are selected sequentially.
  // TX always uses the meter wake carrier; RX uses the selected reply carrier.
  bool set_tx_frequency();
  void set_rx_channel(uint8_t);
  // Compatibility name for older callers; this selects the RX/reply carrier.
  void set_frequency_channel(uint8_t ch) { set_rx_channel(ch); }
  void set_use_custom_channels(bool v) { custom_ = v; }
  void clear_fifo(); void clear_tx_fifo(); void clear_rx_fifo();
  // go_rx() already flushes/re-arms the CC1101 RX path.
  // Do not flush here: doing so would return the chip to IDLE mid-session.
  void reset_rx_fifo_full() {}
  size_t write_fifo(const uint8_t*, size_t); void set_tx_payload_length(uint16_t); void set_rx_payload_length();
  bool transmit_chunked(const uint8_t*, size_t); size_t read_fifo(uint8_t*, size_t);
  bool is_tx_done();
  uint8_t get_interrupt_flags() { return 0; } uint8_t get_int_clr1() { return 0; }
  void clear_interrupt_flags() {} bool is_pkt_ok(); int8_t get_rssi_dbm(); uint8_t get_rssi_code();
  void set_rssi_mode(bool) {} uint8_t scan_channels(int8_t* = nullptr); uint8_t get_fifo_flags();
  bool read_gpio3(); bool test_gpio3_wiring() { return true; } void set_int_source(uint8_t) {}
  void prepare_rx_session(); size_t poll_rx_drain(uint8_t*, size_t);
  void write_reg(uint8_t, uint8_t); uint8_t read_reg(uint8_t); void write_bank(uint8_t, const uint8_t*, size_t) {}
  void update_reg(uint8_t, uint8_t, uint8_t) {}
  // Stop the TX deviation sweep (call when a received frame passed CRC).
 private:
  bool apply_profile_(const RfPhyProfile &profile);
  bool transmit_packet_(const uint8_t *data, size_t len);
  uint8_t read_stable_status_(uint8_t reg);
  bool stop_tx_safely_();
  bool use_custom_channels_() const { return custom_; }
  int miso_{-1}, sclk_{-1}, csn_{-1}, mosi_{-1}, gdo0_{-1};
  float freq_{433.82f}; bool custom_{false}; uint8_t channel_{0};
  uint32_t rx_started_ms_{0};
  Cc1101RxSoftwareSync rx_software_sync_;
  // True once cc1101_init() has brought the SPI bus and the chip up. Replaces
  // the RadioLib `CC1101 *radio_` null check that guarded every register access.
  bool radio_ready_{false};
  uint8_t init_partnum_{0xFF};
  uint8_t init_version_{0xFF};
  int16_t init_result_{-32768};
  const char *init_profile_step_{"not attempted"};
  int16_t init_profile_error_{0};
  bool tx_done_{false};
  bool profile_valid_{false};
  // Last profile actually written to the chip. TX and RX use the same carrier
  // when fix_channel pins them together, so apply_profile_() can recognise an
  // unchanged profile and skip the whole write/readback/log sequence - that
  // sequence is what made the TX->RX arm gap 51 ms while the reply's preamble
  // and sync word only last 47 ms.
  RfPhyProfile applied_profile_{};
  // TX deviation is a single fixed value (NARTIS_RF433_DEVIATN), applied both
  // by apply_profile_() and by transmit_packet_().
};
}
