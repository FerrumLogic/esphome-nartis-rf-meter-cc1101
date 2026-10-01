/*
 * NARTIS RF component-wide limits.
 *
 * These values describe the NARTIS state machine, not registers of a
 * particular transceiver.  Keeping them here prevents the CC1101 path from
 * depending on the legacy CMT2300A register map.
 */

#pragma once

#include <cstdint>

namespace esphome::nartis_rf_meter {

static constexpr uint8_t NARTIS_RF_CHANNEL_COUNT = 4;
// RX frame tails shorter than this are handled after the FIFO chunk drain.
// This is a component framing limit, not a register setting of CC1101/CMT.
static constexpr uint8_t NARTIS_RF_RX_FIFO_THRESHOLD_BYTES = 15;
static constexpr uint32_t RF_STATE_POLL_TIMEOUT_MS = 10;

}  // namespace esphome::nartis_rf_meter
