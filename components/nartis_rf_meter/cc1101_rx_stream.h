#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_meter {

// CC1101 hardware sync detection consumes 72 F6. NARTIS extends that sync with
// 55 55, so these first two FIFO bytes must match before forwarding a frame.
class Cc1101RxSyncTail {
 public:
  void reset() {
    resolved_ = false;
    pending_first_55_ = false;
    tail_dropped_ = false;
    false_sync_candidate_ = false;
  }

  // output_capacity must be >= available + 1; HAL reserves one byte for a
  // possible 55 held across FIFO polls.
  size_t filter(const uint8_t *input, size_t available, uint8_t *output, size_t output_capacity) {
    if (input == nullptr || output == nullptr || available == 0 || output_capacity == 0) return 0;
    size_t input_offset = 0;
    size_t output_size = 0;
    if (!resolved_) {
      if (pending_first_55_) {
        if (input[0] == 0x55) {
          pending_first_55_ = false;
          resolved_ = true;
          tail_dropped_ = true;
          input_offset = 1;
        } else {
          false_sync_candidate_ = true;
          return 0;
        }
      } else if (input[0] == 0x55) {
        if (available == 1) {
          pending_first_55_ = true;
          return 0;
        }
        if (input[1] == 0x55) {
          resolved_ = true;
          tail_dropped_ = true;
          input_offset = 2;
        } else {
          false_sync_candidate_ = true;
          return 0;
        }
      } else {
        false_sync_candidate_ = true;
        return 0;
      }
    }

    const size_t copy_size = std::min(available - input_offset, output_capacity - output_size);
    std::copy(input + input_offset, input + input_offset + copy_size, output + output_size);
    return output_size + copy_size;
  }

  bool tail_dropped() const { return tail_dropped_; }
  bool has_pending_prefix_byte() const { return pending_first_55_; }
  bool false_sync_candidate() const { return false_sync_candidate_; }

 private:
  bool resolved_{false};
  bool pending_first_55_{false};
  bool tail_dropped_{false};
  bool false_sync_candidate_{false};
};

// Pairing/RX packet path: CC1101 cannot express NARTIS's 32-bit 72F65555
// sync word (its 32-bit modes repeat a 16-bit word). Disable its sync gate and
// qualify a 32-bit preamble prefix + full sync in software, accepting either
// bit polarity. This does not require the exact 10-byte TX preamble length.
class Cc1101RxSoftwareSync {
 public:
  void reset() {
    bit_window_ = 0;
    bits_seen_ = 0;
    capturing_ = false;
    inverted_ = false;
    frame_complete_ = false;
    sync_detected_ = false;
    frame_length_ = 0;
    frame_expected_ = 0;
    capture_byte_ = 0;
    capture_bits_ = 0;
  }

  size_t filter(const uint8_t *input, size_t available, uint8_t *output,
                size_t output_capacity, bool *sync_found = nullptr) {
    if (sync_found != nullptr) *sync_found = false;
    if (input == nullptr || output == nullptr || available == 0 || output_capacity == 0 ||
        frame_complete_) return 0;

    size_t output_size = 0;
    for (size_t i = 0; i < available && !frame_complete_; ++i) {
      for (int bit = 7; bit >= 0 && !frame_complete_; --bit) {
        const uint8_t value = static_cast<uint8_t>((input[i] >> bit) & 1U);
        if (!capturing_) {
          bit_window_ = (bit_window_ << 1) | value;
          if (bits_seen_ < 64) ++bits_seen_;
          const bool as_normal = sync_normal(bit_window_);
          const bool as_inverted = sync_inverted(bit_window_);
          if (bits_seen_ == 64 && (as_normal || as_inverted)) {
            inverted_ = as_inverted && !as_normal;
            capturing_ = true;
            sync_detected_ = true;
            frame_length_ = 0;
            frame_expected_ = 0;
            capture_byte_ = 0;
            capture_bits_ = 0;
            bit_window_ = 0;
            bits_seen_ = 0;
            if (sync_found != nullptr) *sync_found = true;
          }
          continue;
        }

        capture_byte_ = static_cast<uint8_t>((capture_byte_ << 1) | (value ^ (inverted_ ? 1U : 0U)));
        if (++capture_bits_ != 8) continue;
        if (output_size >= output_capacity) break;
        output[output_size++] = capture_byte_;
        if (frame_length_ == 0) frame_expected_ = static_cast<size_t>(capture_byte_) + 1;
        ++frame_length_;
        capture_byte_ = 0;
        capture_bits_ = 0;
        if (frame_length_ >= frame_expected_) {
          frame_complete_ = true;
          capturing_ = false;
        }
      }
    }
    return output_size;
  }

  bool frame_complete() const { return frame_complete_; }
  bool sync_detected() const { return sync_detected_; }
  bool frame_inverted() const { return inverted_; }

 private:
  // ON-AIR order, not the CMT register order. A CRC-validated D101 frame shows
  // 10 x 0x55 preamble then the sync bytes reversed: 55 55 F6 72. Matching
  // 72 F6 55 55 meant the filter never released a single byte -> "RX timeout
  // (0/0 bytes received)" while the HAL was reading the stream fine.
  // Accept every plausible on-air spelling instead of one guess:
  //   55 55 55 55 55 55 F6 72 - observed on a CRC-validated D101 frame
  //   55 55 55 55 55 55 72 F6 - same bytes, CMT register order
  //   55 55 55 55 72 F6 55 55 - what the migration doc assumed (4+2 preamble)
  //   55 55 55 55 F6 55 55 55 - NARTIS_BASEBAND_BANK PKT10..13 read as F6 55 55 55
  // Any of them plus its bit-inversion; a wrong guess meant the filter dropped
  // every byte and poll_rx_ reported "RX timeout (0/0 bytes received)".
  static constexpr uint64_t SYNC_CANDIDATES[] = {
      0x555555555555F672ULL,
      0x55555555555572F6ULL,
      0x5555555572F65555ULL,
      0x55555555F6555555ULL,
  };
  static bool sync_normal(uint64_t w) {
    for (uint64_t c : SYNC_CANDIDATES) if (w == c) return true;
    return false;
  }
  static bool sync_inverted(uint64_t w) {
    for (uint64_t c : SYNC_CANDIDATES) if (w == ~c) return true;
    return false;
  }
  uint64_t bit_window_{0};
  uint8_t bits_seen_{0};
  bool capturing_{false};
  bool inverted_{false};
  bool frame_complete_{false};
  bool sync_detected_{false};
  size_t frame_length_{0};
  size_t frame_expected_{0};
  uint8_t capture_byte_{0};
  uint8_t capture_bits_{0};
};

}  // namespace esphome::nartis_rf_meter
