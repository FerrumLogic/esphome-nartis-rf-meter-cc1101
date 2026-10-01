/* CC1101 hardware abstraction over the vendored ESP-IDF driver. */

#include "cc1101_hal.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include "esphome/core/log.h"

namespace esphome::nartis_rf_meter {
static const char *const TAG = "nartis_cc1101";

// ---------------------------------------------------------------------------
// Local substitutes for the RadioLib result codes this file used to return.
// Keeping the same names/shape lets the existing log strings and control flow
// stay untouched while the transport underneath changes.
// ---------------------------------------------------------------------------
static constexpr int16_t RC_OK = 0;          // was RC_OK
static constexpr int16_t RC_ERR = -1;         // was RC_ERR
static constexpr int16_t RC_TX_TIMEOUT = -5;  // was RC_TX_TIMEOUT

// ---------------------------------------------------------------------------
// Register access via the vendored driver.
//
// CC1101 header byte: bit7 = 1 for a read, bit6 = burst, bits5:0 = address.
// That gives three addressing modes, which is exactly what the vendored
// readReg() regType argument selects:
//   config registers (0x00-0x2F) -> READ_SINGLE  (addr | 0x80)
//   status registers (0x30-0x3D) -> READ_BURST   (addr | 0xC0)
//   FIFO (0x3F)                  -> READ_BURST   (0x3F | 0xC0)
// Writes always go out as the bare address; burst writes set bit6.
//
// RadioLib expressed the same thing as SPIreadRegister/SPIwriteRegister/
// SPIreadRegisterBurst/SPIwriteRegisterBurst/SPIsendCommand, so these helpers
// are a one-to-one replacement - notably for MARCSTATE/RXBYTES/RSSI, whose
// status-register read must use 0xC0 or every state read returns garbage.
// ---------------------------------------------------------------------------
static inline uint8_t hal_rd(uint8_t reg) {
  return readReg(reg, reg < 0x30 ? CC1101_CONFIG_REGISTER : CC1101_STATUS_REGISTER);
}
static inline void hal_wr(uint8_t reg, uint8_t val) { writeReg(reg, val); }
static inline void hal_rdburst(uint8_t reg, uint8_t len, uint8_t *buf) { readBurstReg(buf, reg, len); }
static inline void hal_wrburst(uint8_t reg, uint8_t *buf, uint8_t len) { writeBurstReg(reg, buf, len); }
static inline void hal_strobe(uint8_t cmd) { cmdStrobe(cmd); }
static inline void hal_idle() { cmdStrobe(CC1101_SIDLE); }
static inline void hal_rx() { cmdStrobe(CC1101_SRX); }
static inline void hal_flush_rx() { cmdStrobe(CC1101_SFRX); }
static inline void hal_flush_tx() { cmdStrobe(CC1101_SFTX); }
static inline void hal_tx() { cmdStrobe(CC1101_STX); }

static constexpr uint8_t REG_FIFO=0x3F, REG_RXBYTES=0x3B, REG_TXBYTES=0x3A;
static constexpr uint8_t REG_PKTSTATUS=0x38, REG_MARCSTATE=0x35;
static constexpr uint8_t REG_PKTCTRL1=0x07, REG_PKTCTRL0=0x08, REG_PKTLEN=0x06;
static constexpr uint8_t CC1101_CMD_FLUSH_RX=0x3A;
static constexpr uint8_t CC1101_CMD_FLUSH_TX=0x3B;
static constexpr uint8_t MARCSTATE_RX=0x0D, MARCSTATE_RX_END=0x0E, MARCSTATE_RX_RST=0x0F;
static constexpr uint8_t MARCSTATE_CALIBRATE_BEGIN=0x03, MARCSTATE_ENDCAL=0x0C;
static constexpr uint32_t DIAG_RX_TRANSITION_GRACE_MS=50;

// SIDLE only *requests* IDLE. Configuration registers are ignored unless the
// state machine is really there, so wait for MARCSTATE==0x01 - bounded, because
// a wedged chip must degrade to a failed profile, not hang the event loop.
static uint8_t hal_go_idle_(uint32_t timeout_ms = 20) {
  hal_idle();
  const uint32_t started = millis();
  uint8_t st = 0;
  do {
    st = hal_rd(REG_MARCSTATE) & 0x1F;
    if (st == 0x01) return st;
    delay(1);
  } while (millis() - started < timeout_ms);
  return st;
}

// Pairing rate candidate from the CMT DATA_RATE_BANK fingerprint: ~2.4 ksymbol/s.
// The 4.8-kbit/s RX trial completed without detecting a full sync.
static constexpr float NORMAL_TX_BITRATE_KBPS = 2.4f;
static constexpr float NORMAL_RX_BITRATE_KBPS = 2.4f;
// One-shot probe and its following receiver must use the CMT-derived nominal
// symbol rate. At 4.8 kbps a valid 2.4 kbps meter reply would be undecodable.
static constexpr float RX_DEVIATION_KHZ = 10.32f;
// TX/RX deviation. CHANGED 0x35 -> 0x25: 0x35 = 20.63 kHz is the SmartRF
// deviation of the 38.4 kBaud preset (MDMCFG4=0xCA, MDMCFG3=0x83, DEVIATN=0x35
// decode to exactly 38400 Bd), while our MDMCFG4/3 run at 2.4 kBaud - so the
// modulation index was 20.63/2.4 = 8.6. 0x25 gives +-10.3 kHz (index 4.3) and
// is what the alternative CC1101 port (nartis-cc1101.zip) defaults to for the
// same 2.4k air interface. A meter expecting ~5-10 kHz of deviation can fail to
// limit our +-20.6 kHz signal even though it is perfectly readable by us.
// 0x35 = 20.63 kHz, 0x25 = 10.32 kHz, 0x15 = 5.16 kHz.
static constexpr uint8_t NARTIS_RF433_DEVIATN = 0x25;
// Controlled modulation trial: the original CMT profile is (G)FSK, but its
// checked-in RFPDK bank does not prove Gaussian shaping; test plain 2-FSK.
static constexpr uint8_t NARTIS_RF433_MOD_FORMAT = 0x00;
// Exact CMT packet prefix from BASEBAND CUS_PKT2/3 and PKT10-13.
static constexpr uint8_t NARTIS_CMT_TX_PREAMBLE_BYTES = 10;
// ON-AIR byte order. The CMT PKT10..13 register order is 72 F6 55 55, but the
// captured D101 frames put it on the air reversed: after the 10 x 0x55 preamble
// a receiver reading a CRC-validated frame sees exactly 12 x 0x55 then F6 72 -
// i.e. preamble(10) + sync's leading 55 55 = 12 runs, then F6 72. Sending
// 72 F6 55 55 made the meter's sync detector never match -> "no probe response".
static constexpr uint8_t NARTIS_CMT_SYNC_WORD[] = {0x55, 0x55, 0xF6, 0x72};
static constexpr size_t CC1101_TX_FIFO_CAPACITY = 64;
static constexpr size_t CC1101_MAX_PACKET_BYTES = 255;
// CC1101 exposes discrete RX filter steps: 101.6 is the hardware step below
// the 203.1 maximum that was used before. Carson bandwidth for the configured
// PHY is 2*(20.63 + 2.4/2) = 43.7 kHz, so 101.6 kHz keeps ~2.3x margin and
// still tolerates ~+/-29 kHz of relative carrier offset (~+/-67 ppm) while
// passing less out-of-band interference and noise into the demodulator.
// RX filter bandwidth. REVERTED from 101.6 back to the 203.1 kHz step, because
// 203.1 was the width in use on 29.09 21:19 - the only moment a meter answer was
// ever captured. Narrowing to 101.6 (30.09) was justified by Carson's rule
// (2*(20.63+1.2) = 43.7 kHz, so 101.6 is already 2.3x margin) but that assumes a
// perfectly centred carrier. The AFC/FOCC capture range scales with the filter
// width: roughly +/-(BW/2 - DEV), i.e. +/-30 kHz at 101.6 but +/-81 kHz at 203.1.
// A 100 ppm crystal error at 434 MHz is 43 kHz - outside the narrow window and
// inside the wide one. Choosing headroom over selectivity until a capture proves
// the carrier is centred.
// RX filter bandwidth: 101.6 kHz.
//
// Do NOT "restore" 203.1 kHz here - it was tried and is measurably worse. A/B on
// identical captures (same MDMCFG3=0x83 rate, same PKTCTRL=02.04/PKTLEN=FF, only
// MDMCFG4 differs C6 vs 86):
//   101.6 kHz -> ones density 0.50-0.52, entropy 7.93, 0% zero bytes: the slicer
//                is running and the channel is actually being sampled.
//   203.1 kHz -> ones density 0.072, entropy 1.54, 85% zero bytes: the demodulator
//                output collapses to one rail and the receiver stops observing
//                anything at all.
// Carson for this PHY is 2*(20.63+2.4/2) = 43.7 kHz, so 203.1 is 4.6x wide - wide
// enough that the AGC attenuates the band below the slicer threshold. 101.6 is
// still 2.3x Carson: kept for the frequency-offset headroom (+/-30 kHz AFC), which
// is the only reason not to tighten further to the 81.25/67.7 kHz steps.
static constexpr float RX_BANDWIDTH_KHZ = 101.6f;

void Cc1101Hal::set_pins(int miso, int sclk, int csn, int mosi, int gdo0) {
  miso_=miso; sclk_=sclk; csn_=csn; mosi_=mosi; gdo0_=gdo0;
}
// ---------------------------------------------------------------------------
// CC1101 PHY arithmetic.
//
// These four helpers replace RadioLib's setFrequency()/setBitRate()/
// setRxBandwidth()/setFrequencyDeviation(). The formulas are the same ones the
// readback at the end of apply_profile_() uses, so "what we asked for" and
// "what we verify" are computed from one source and cannot drift apart - which
// was possible before, where RadioLib rounded and the readback only compared.
//
//   f_carrier = FREQ_code * 26 MHz / 2^16        -> FREQ2/1/0
//   DRATE     = (256+M)*2^E * 26 MHz / 2^28      -> MDMCFG3 = M, MDMCFG4[3:0] = E
//   BW        = 26 MHz / (8*(4+bw_e)*2^bw_m)     -> MDMCFG4[5:4], MDMCFG4[7:6]
//   deviation = (8+M)*2^E * 26 MHz / 2^17        -> DEVIATN (pinned to 0x35)
//
// Verified against the live profile: 2.4 kbps -> M=131/E=6 -> MDMCFG3=0x83,
// MDMCFG4 low nibble 6, and 101.6 kHz -> bw_m=3/bw_e=0 -> MDMCFG4=0xC6, i.e.
// exactly the MDMCFG=C6.83.00 the device has been logging since the fix.
// ---------------------------------------------------------------------------
static float hal_bandwidth_of_(uint8_t mdmcfg4) {
  return 26000.0f / (8.0f * float(4u + ((mdmcfg4 >> 4) & 0x03)) *
                     float(1u << ((mdmcfg4 >> 6) & 0x03)));
}
static uint8_t hal_bandwidth_bits_(float khz) {
  uint8_t best = 0;
  float best_err = 1e9f;
  for (int e = 0; e < 4; ++e) {
    for (int m = 0; m < 4; ++m) {
      const uint8_t bits = (uint8_t) ((m << 6) | (e << 4));
      const float err = hal_bandwidth_of_(bits) - khz;
      const float abs_err = err < 0 ? -err : err;
      if (abs_err < best_err) {
        best_err = abs_err;
        best = bits;
      }
    }
  }
  return best;
}
static void hal_write_bitrate_(float kbps, uint8_t *mdmcfg3, uint8_t *drate_e) {
  const float scaled = kbps * 1000.0f * 268435456.0f / 26000000.0f;
  int e = 0;
  float mant = scaled;
  while (mant >= 512.0f && e < 15) { mant *= 0.5f; ++e; }
  while (mant < 256.0f && e > 0) { mant *= 2.0f; --e; }
  int m = (int) (mant + 0.5f) - 256;
  if (m < 0) m = 0;
  if (m > 255) m = 255;
  *mdmcfg3 = (uint8_t) m;
  *drate_e = (uint8_t) e;
}
static void hal_write_frequency_(float mhz) {
  const uint32_t code = (uint32_t) (mhz * 65536.0f / 26.0f);
  hal_wr(0x0D, (uint8_t) ((code >> 16) & 0x3F));  // FREQ2, bits5:0 only
  hal_wr(0x0E, (uint8_t) ((code >> 8) & 0xFF));   // FREQ1
  hal_wr(0x0F, (uint8_t) (code & 0xFF));          // FREQ0
}

bool Cc1101Hal::apply_profile_(const RfPhyProfile &p) {
  if (!radio_ready_) return false;

  // IDLE first. This is NOT part of the profile - it is a precondition for every
  // configuration write AND for the TX FIFO write, so it must run before the fast
  // path below. init() leaves the chip in RX, and after an RX window it is in RX
  // again; with TX and RX pinned to the same carrier the profile comparison says
  // "unchanged", so skipping the idle would write the TX FIFO while the chip is
  // still receiving - CC1101 drops those bytes silently and the driver reports
  // "TX FIFO readback mismatch: expected=44 got=0x00" followed by ERROR_RECOVERY.
  const uint8_t idle = hal_go_idle_();
  if (idle != 0x01) {
    init_profile_step_ = "standby";
    init_profile_error_ = RC_ERR;
    ESP_LOGE(TAG, "CC1101 standby failed: MARCSTATE=0x%02X", (unsigned) idle);
    return false;
  }

  // Unchanged profile: nothing to write, nothing to verify, nothing to log.
  // Measured TX->RX arm gap was 51 ms while apply_profile_() plus its
  // dominated it; the reply preamble+sync lasts only 47 ms,
  // so every millisecond spent re-proving an identical configuration is a
  // millisecond of blindness right at the start of the answer.
  if (profile_valid_ &&
      applied_profile_.carrier_mhz == p.carrier_mhz &&
      applied_profile_.bitrate_kbps == p.bitrate_kbps &&
      applied_profile_.deviation_khz == p.deviation_khz &&
      applied_profile_.rx_bandwidth_khz == p.rx_bandwidth_khz &&
      applied_profile_.sync_enabled == p.sync_enabled &&
      applied_profile_.ook == p.ook &&
      // Defence in depth: TX rewrites these two, and an RF-only comparison
      // cannot see that. Two register reads are far cheaper than silently
      // receiving in fixed-length framing, which yields one garbage packet and
      // then an empty FIFO for the whole window.
      hal_rd(REG_PKTCTRL0) == 0x02 && hal_rd(REG_PKTLEN) == 0xFF) {
    return true;
  }
  profile_valid_ = false;

  hal_write_frequency_(p.carrier_mhz);

  uint8_t drate_m = 0, drate_e = 0;
  hal_write_bitrate_(p.bitrate_kbps, &drate_m, &drate_e);
  hal_wr(0x11, drate_m);                                   // MDMCFG3
  const uint8_t bw_bits = hal_bandwidth_bits_(p.rx_bandwidth_khz);
  const uint8_t mdmcfg4 = (uint8_t) (bw_bits | drate_e);
  hal_wr(0x10, mdmcfg4);                                   // MDMCFG4

  // Deviation: pin the recovered RF433 value. RadioLib mapped the requested
  // 20.6 kHz to 0x34 (19.04 kHz), which is a different PHY than the source.
  hal_wr(0x15, NARTIS_RF433_DEVIATN);
  if (hal_rd(0x15) != NARTIS_RF433_DEVIATN) {
    ESP_LOGE(TAG, "CC1101 RF433 deviation readback mismatch");
    init_profile_step_ = "deviation";
    init_profile_error_ = RC_ERR;
    return false;
  }

  // On-air 16-bit sync is F6 72: CC1101 transmits SYNC1 (0x04) first, and a
  // CRC-validated capture reads "... 55 55 F6 72 | len ...". The leading 55 55
  // of NARTIS_CMT_SYNC_WORD is carried by the preamble bytes we push into the
  // FIFO ourselves, because CC1101 holds only 16 sync bits. Written directly so
  // the readback below cannot contradict the air order.
  hal_wr(0x04, 0xF6);  // SYNC1 - transmitted first
  hal_wr(0x05, 0x72);  // SYNC0
  if (hal_rd(0x04) != 0xF6 || hal_rd(0x05) != 0x72) {
    ESP_LOGE(TAG, "CC1101 sync word readback mismatch");
    init_profile_step_ = "sync word";
    init_profile_error_ = RC_ERR;
    return false;
  }

  // CC1101 MOD_FORMAT (MDMCFG2[6:4]): 000 = 2-FSK, 001 = GFSK, 011 = ASK/OOK.
  // SYNC_MODE (MDMCFG2[1:0]): 0 = none, 2 = 16/16 - CC1101 has 16 hardware
  // sync bits; the trailing 55 55 is handled by the stream code.
  const uint8_t mdmcfg2 = p.ook ? (p.sync_enabled ? 0x32 : 0x30)
                                 : (NARTIS_RF433_MOD_FORMAT | (p.sync_enabled ? 0x02 : 0x00));
  hal_wr(0x12, mdmcfg2);
  // NUM_PREAMBLE only; CHANSPC/EXTRANGE bits are preserved as-is.
  const uint8_t mdmcfg1 = hal_rd(0x13);
  const uint8_t mdmcfg1_expected = (uint8_t) ((mdmcfg1 & 0x8F) | 0x50);
  hal_wr(0x13, mdmcfg1_expected);

  // Infinite RX stream, no hardware CRC/whitening, APPEND_STATUS on.
  hal_wr(REG_PKTCTRL1, 0x04);
  hal_wr(REG_PKTCTRL0, 0x02);
  hal_wr(REG_PKTLEN, 0xFF);

  if (hal_rd(0x04) != 0xF6 || hal_rd(0x05) != 0x72 || hal_rd(0x12) != mdmcfg2 ||
      hal_rd(REG_PKTCTRL1) != 0x04 || hal_rd(REG_PKTCTRL0) != 0x02 ||
      hal_rd(REG_PKTLEN) != 0xFF) {
    init_profile_step_ = "PHY readback";
    init_profile_error_ = RC_ERR;
    ESP_LOGE(TAG, "CC1101 PHY readback mismatch");
    return false;
  }

  // Extended readback: "these registers hold what we wrote" is not enough -
  // the rate and bandwidth DERIVED from MDMCFG3/4 must match the requested
  // profile, otherwise rounding silently lands on a different PHY.
  const uint8_t rb_m4 = hal_rd(0x10);  // MDMCFG4: CHANBW + DRATE_E
  const uint8_t rb_m3 = hal_rd(0x11);  // MDMCFG3: DRATE_M
  const float rb_bitrate = float(256u + rb_m3) * float(1u << (rb_m4 & 0x0F)) *
                           26.0f / 268435456.0f * 1000.0f;
  const float rb_bandwidth = hal_bandwidth_of_(rb_m4);
  const float bitrate_err = (rb_bitrate > p.bitrate_kbps) ? (rb_bitrate - p.bitrate_kbps)
                                                          : (p.bitrate_kbps - rb_bitrate);
  const float bandwidth_err = (rb_bandwidth > p.rx_bandwidth_khz) ? (rb_bandwidth - p.rx_bandwidth_khz)
                                                                 : (p.rx_bandwidth_khz - rb_bandwidth);
  if (p.bitrate_kbps <= 0 || bitrate_err / p.bitrate_kbps > 0.03f ||
      bandwidth_err / p.rx_bandwidth_khz > 0.05f ||
      hal_rd(0x13) != mdmcfg1_expected || hal_rd(0x15) != NARTIS_RF433_DEVIATN) {
    ESP_LOGE(TAG, "CC1101 PHY readback mismatch: rate %.3f/%.3f kbps bw %.1f/%.1f kHz "
                  "mdmcfg1=%02X/%02X deviatn=%02X",
             (double) rb_bitrate, (double) p.bitrate_kbps,
             (double) rb_bandwidth, (double) p.rx_bandwidth_khz,
             (unsigned) hal_rd(0x13), (unsigned) mdmcfg1_expected,
             (unsigned) hal_rd(0x15));
    init_profile_step_ = "PHY readback";
    init_profile_error_ = RC_ERR;
    return false;
  }

  freq_ = p.carrier_mhz;
  applied_profile_ = p;
  profile_valid_ = true;
  init_profile_step_ = "ok";
  init_profile_error_ = RC_OK;
  return true;
}

bool Cc1101Hal::init() {
  if (miso_ < 0 || sclk_ < 0 || csn_ < 0 || mosi_ < 0) return false;
  // The vendored driver takes its pins at runtime instead of the Kconfig
  // defaults, and cc1101_init() then claims the SPI bus, resets the chip and
  // loads the reference register set (leaving the chip in IDLE, because the
  // upstream trailing sendData() was removed).
  cc1101_set_pins(miso_, sclk_, mosi_, csn_, gdo0_);
  init_result_ = (cc1101_init(CFREQ_433, CSPEED_4800) == ESP_OK) ? RC_OK : RC_ERR;
  radio_ready_ = (init_result_ == RC_OK);
  // Identity must be read AFTER init(): it is what brings the bus and the CS
  // line up, so a pre-init status-register read always returns 0x00/0x00.
  init_partnum_ = radio_ready_ ? hal_rd(0x30) : 0xFF;
  init_version_ = radio_ready_ ? hal_rd(0x31) : 0xFF;
  if (!radio_ready_) {
    ESP_LOGE(TAG, "CC1101 init failed: %d", (int) init_result_);
    return false;
  }
  // TX output power lives in PATABLE, and neither setCCregs() nor our profile
  // PATABLE decides the actual TX output power and neither the register-reset
  // defaults nor setCCregs() touch it, so pin it explicitly (both upstream
  // Arduino examples call setTxPowerAmp()) and verify the chip took it.
  hal_wr(CC1101_PATABLE, PA_MaxPower_433);
  uint8_t pa_set[8]{};
  hal_rdburst(CC1101_PATABLE, 8, pa_set);
  if (pa_set[0] != PA_MaxPower_433) {
    ESP_LOGE(TAG, "PATABLE readback mismatch: wrote 0x%02X, chip reports 0x%02X",
             (unsigned) PA_MaxPower_433, (unsigned) pa_set[0]);
    init_profile_step_ = "PATABLE";
    init_profile_error_ = RC_ERR;
    return false;
  }

  init_profile_step_ = "apply profile";
  init_profile_error_ = RC_OK;
  if (!apply_profile_({freq_, NORMAL_RX_BITRATE_KBPS, RX_DEVIATION_KHZ, RX_BANDWIDTH_KHZ, false})) return false;
  hal_rx();
  return true;
}
void Cc1101Hal::log_init_diagnostics() const {
  ESP_LOGCONFIG(TAG, "  CC1101 init: PARTNUM=0x%02X VERSION=0x%02X driver init=%d profile=%s error=%d",
                (unsigned) init_partnum_, (unsigned) init_version_, (int) init_result_,
                init_profile_step_, (int) init_profile_error_);
}
bool Cc1101Hal::is_chip_connected(){ return radio_ready_; }
bool Cc1101Hal::go_standby(){
  if (!radio_ready_) return false;
  return hal_go_idle_() == 0x01;
}
bool Cc1101Hal::go_sleep(){ return go_standby(); }
bool Cc1101Hal::go_rx(){
  if (!radio_ready_ || !profile_valid_) return false;
  // Flush then re-arm: stale bytes from the previous dwell must not be read as
  // the head of the next frame. IDLE before SFRX, because the flush strobe is
  // only valid in IDLE or RXFIFO_OVERFLOW.
  hal_idle();
  hal_flush_rx();
  hal_rx();
  rx_started_ms_ = millis();
  return true;
}
bool Cc1101Hal::go_tx(){ return true; }
bool Cc1101Hal::is_tx_done(){ bool done=tx_done_; tx_done_=false; return done; }
uint8_t Cc1101Hal::get_state(){ return radio_ready_ ? (uint8_t) (hal_rd(REG_MARCSTATE) & 0x1F) : 0xFF; }
bool Cc1101Hal::wait_for_state(uint8_t,uint32_t){ return true; }
bool Cc1101Hal::set_tx_frequency(){
  // Unlike the CMT2300A, CC1101 has no separate RX/TX frequency halves.
  // The meter wakes on this carrier; RX is retuned only after TX completes.
  if (!radio_ready_ || !apply_profile_({433.82f, NORMAL_TX_BITRATE_KBPS, RX_DEVIATION_KHZ, RX_BANDWIDTH_KHZ, false})) {
    ESP_LOGE(TAG, "Failed to configure TX wake carrier");
    return false;
  }
  return true;
}
void Cc1101Hal::set_rx_channel(uint8_t ch){
  // These are physical reply carriers for CC1101, not CMT2300A LO values.
  static constexpr float REPLY_DEFAULT[] = {434.10f, 433.58f, 434.54f, 434.98f};
  static constexpr float REPLY_CUSTOM[]  = {433.82f, 433.30f, 434.26f, 434.70f};
  if(ch<4) {
    channel_=ch;
    if (!apply_profile_({use_custom_channels_() ? REPLY_CUSTOM[ch] : REPLY_DEFAULT[ch],
                         NORMAL_RX_BITRATE_KBPS, RX_DEVIATION_KHZ, RX_BANDWIDTH_KHZ, false}))
      ESP_LOGE(TAG, "Failed to configure RX channel %u", (unsigned) ch);
  }
}
void Cc1101Hal::clear_tx_fifo(){ if (radio_ready_) hal_idle(); }
void Cc1101Hal::clear_rx_fifo(){ if (radio_ready_) { hal_idle(); hal_flush_rx(); } }
void Cc1101Hal::clear_fifo(){ clear_tx_fifo(); clear_rx_fifo(); }
void Cc1101Hal::prepare_rx_session(){
  if (!radio_ready_) return;
  rx_software_sync_.reset();
  set_rx_channel(channel_ & 3);
  hal_strobe(CC1101_CMD_FLUSH_RX);
}
void Cc1101Hal::set_tx_payload_length(uint16_t){ }
void Cc1101Hal::set_rx_payload_length(){ }
size_t Cc1101Hal::write_fifo(const uint8_t*,size_t){ return 0; }
bool Cc1101Hal::transmit_chunked(const uint8_t* d,size_t n){
  return transmit_packet_(d, n);
}
uint8_t Cc1101Hal::read_stable_status_(uint8_t reg) {
  if (!radio_ready_) return 0xFF;
  uint8_t previous = hal_rd(reg);
  for (uint8_t i = 0; i < 8; ++i) {
    const uint8_t current = hal_rd(reg);
    if (current == previous) return current;
    previous = current;
  }
  return 0xFF;  // A changing status is not evidence of a stable state.
}
bool Cc1101Hal::stop_tx_safely_() {
  if (!radio_ready_) return false;
  hal_idle();
  // SFTX is valid only in IDLE; do not flush while the state is uncertain.
  const uint8_t idle = read_stable_status_(REG_MARCSTATE);
  if (idle != 0x01) {
    ESP_LOGE(TAG, "CC1101 TX cleanup failed: standby timeout, MARC=0x%02X", (unsigned) idle);
    return false;
  }
  hal_strobe(CC1101_CMD_FLUSH_TX);
  const uint8_t txbytes = read_stable_status_(REG_TXBYTES);
  if (txbytes != 0 || read_stable_status_(REG_MARCSTATE) != 0x01) {
    ESP_LOGE(TAG, "CC1101 TX cleanup failed: TXBYTES=0x%02X", (unsigned) txbytes);
    return false;
  }
  // TX deliberately runs in fixed-length framing (PKTCTRL0=0x00, PKTLEN=airlen)
  // so the exact CMT bytes go out as-is. Put the packet engine back into the
  // infinite-stream RX framing that the applied profile describes. Without this
  // the apply_profile_() fast path sees an unchanged RF profile and skips the
  // rewrite, leaving RX at PKTCTRL0=0x00/PKTLEN=44 - the chip then clocks 44
  // bytes of garbage, closes the receive, and the FIFO stays empty for the rest
  // of the 6 s window (observed as "RX timeout (0/0 bytes)" with a 46-byte
  // dump = 44 data + 2 APPEND_STATUS bytes).
  hal_wr(REG_PKTCTRL0, 0x02);
  hal_wr(REG_PKTLEN, 0xFF);
  return true;
}
bool Cc1101Hal::transmit_packet_(const uint8_t* d, size_t n) {
  tx_done_ = false;
  if (!radio_ready_ || !d || !n ||
      n > (CC1101_MAX_PACKET_BYTES - NARTIS_CMT_TX_PREAMBLE_BYTES - sizeof(NARTIS_CMT_SYNC_WORD))) return false;
  if (!set_tx_frequency()) { stop_tx_safely_(); return false; }
  // TX deviation: one fixed value from the recovered RF433 profile,
  // DEVIATN=0x35 (~20.63 kHz at 26 MHz). Encoding: dev kHz = (8+M)*2^E *
  // 26/131072, E=bits 6:4, M=bits 2:0. A "sweep candidates until a reply
  // passes CRC" mechanism used to live here, but DEV_TABLE held exactly one
  // entry and apply_profile_() pins 0x15 anyway, so it never swept anything.
  const uint8_t tx_deviation = NARTIS_RF433_DEVIATN;
  hal_wr(0x15, tx_deviation);
  if (hal_rd(0x15) != tx_deviation) {
    ESP_LOGE(TAG, "CC1101 TX deviation readback mismatch");
    stop_tx_safely_();
    return false;
  }
  // Send the source CMT packet prefix byte-for-byte. CC1101 automatic preamble
  // length is discrete and its 32-bit sync option repeats 72F6 (not 72F65555),
  // so disable its preamble/sync insertion and put the exact CMT bytes in FIFO.
  uint8_t txbuf[CC1101_MAX_PACKET_BYTES];
  size_t header_len = 0;
  for (uint8_t i = 0; i < NARTIS_CMT_TX_PREAMBLE_BYTES; ++i) txbuf[header_len++] = 0x55;
  memcpy(txbuf + header_len, NARTIS_CMT_SYNC_WORD, sizeof(NARTIS_CMT_SYNC_WORD));
  header_len += sizeof(NARTIS_CMT_SYNC_WORD);
  memcpy(txbuf + header_len, d, n);
  const size_t airlen = header_len + n;
  // Result of the manual TX sequence below; declared here because the RadioLib
  // call it replaced carried it.
  int16_t state = RC_OK;
  // Fixed-length TX: LENGTH_CONFIG=00 and PKTLEN=airlen. RadioLib's
  // fixedPacketLengthMode() did both; PKTCTRL0 is checked again below.
  hal_wr(REG_PKTLEN, static_cast<uint8_t>(airlen));
  hal_wr(REG_PKTCTRL0, 0x00);
  if (hal_rd(REG_PKTLEN) != airlen) {
    ESP_LOGE(TAG, "CC1101 fixed packet length failed");
    stop_tx_safely_();
    return false;
  }
  if (hal_rd(REG_PKTLEN) != airlen) {
    ESP_LOGE(TAG, "CC1101 TX packet length readback mismatch");
    stop_tx_safely_();
    return false;
  }
  // Disable only automatic preamble/sync generation: those bytes are already
  // present in txbuf in the exact CMT order. RX profile restores 16-bit sync.
  hal_wr(0x12, NARTIS_RF433_MOD_FORMAT);
  if (hal_rd(0x04) != 0xF6 || hal_rd(0x05) != 0x72 ||
      hal_rd(0x12) != NARTIS_RF433_MOD_FORMAT ||
      hal_rd(REG_PKTCTRL1) != 0x04 ||
      hal_rd(REG_PKTCTRL0) != 0x00) {
    ESP_LOGE(TAG, "CC1101 TX static PHY mismatch");
    stop_tx_safely_();
    return false;
  }
  // RadioLib startTransmit() is unusable here: it issues FSTXON and waits
  // only 1600 us for MARCSTATE==0x12, returning ERR_TX_TIMEOUT when the
  // synthesizer calibration outlasts that window (observed on this board:
  // rc=-5, TXBYTES=00, chip stuck in IDLE, nothing on air). Drive the TX
  // sequence manually instead: flush, fill FIFO, STX, then poll the state
  // machine in the loop below.
  state = RC_OK;
  hal_strobe(CC1101_CMD_FLUSH_TX);
  const size_t initial = std::min<size_t>(airlen, CC1101_TX_FIFO_CAPACITY);
  hal_wrburst(REG_FIFO, txbuf, static_cast<uint8_t>(initial));
  size_t written = initial;
  const uint8_t armed_txbytes = read_stable_status_(REG_TXBYTES);
  // STX can be dropped while the synth is still calibrating; re-issue it
  // until MARCSTATE leaves IDLE (calibration or TX entry proves acceptance).
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    hal_strobe(CC1101_STX);
    const uint32_t stx_wait = millis();
    while (millis() - stx_wait < 20) {
      if (read_stable_status_(REG_MARCSTATE) != 0x01) break;  // left IDLE
      delay(1);
    }
    if (read_stable_status_(REG_MARCSTATE) != 0x01) break;
  }
  if (armed_txbytes != initial) {
    ESP_LOGE(TAG, "CC1101 TX FIFO readback mismatch: expected=%u initially queued, got=0x%02X",
             (unsigned) initial, (unsigned) armed_txbytes);
    stop_tx_safely_();
    return false;
  }
  bool seen_tx = false;
  if(state == RC_OK) {
    // RadioLib's blocking transmit() waits for GDO2. This board leaves GDO2
    // unconnected, so use the CC1101 state machine instead and finish when
    // the radio returns to IDLE.
    const uint32_t started = millis();
    // A full 255-byte fixed packet takes ~850 ms at 2.4 kbit/s.
    while(millis() - started < 1300) {
      const uint8_t marc = read_stable_status_(REG_MARCSTATE);
      if(marc == 0x13) seen_tx = true;
      const uint8_t txbytes = read_stable_status_(REG_TXBYTES);
      if(marc == 0x16 || (txbytes & 0x80)) {  // TXFIFO underflow is failure.
        ESP_LOGE(TAG, "CC1101 TXFIFO underflow");
        break;
      }
      // Keep a 32-byte margin before the FIFO can empty at the fastest
      // supported pairing rate. At 2.4 kbit/s that is >100 ms of airtime.
      if (seen_tx && written < airlen && (txbytes & 0x7F) <= 32) {
        const size_t chunk = std::min<size_t>(airlen - written, 32);
        hal_wrburst(REG_FIFO, txbuf + written, static_cast<uint8_t>(chunk));
        written += chunk;
      }
      if(seen_tx && marc == 0x01) break;                 // IDLE = packet done
      delay(1);
    }
    const uint8_t marc = read_stable_status_(REG_MARCSTATE);
    if (!seen_tx || marc != 0x01 || written != airlen) state = RC_TX_TIMEOUT;
    else { hal_idle(); state = RC_OK; }
  }
  const bool cleanup_ok = stop_tx_safely_();
  if (!cleanup_ok) state = RC_TX_TIMEOUT;
  tx_done_=(state==RC_OK);
  if(state!=RC_OK) ESP_LOGW(TAG, "CC1101 TX failed: %d", (int)state);
  return tx_done_;
}
size_t Cc1101Hal::poll_rx_drain(uint8_t* b,size_t n){
  if(!radio_ready_) return 0;
  uint8_t status=hal_rd(REG_RXBYTES);
  if(status & 0x80){
    ESP_LOGW(TAG,"CC1101 RX FIFO overflow (RXBYTES=0x%02X)",status);
    rx_software_sync_.reset();
    hal_idle();
    hal_flush_rx();
    hal_rx();
    rx_started_ms_ = millis();
    return 0;
  }
  const uint8_t a=status & 0x7F;
  if(!a || !n) return 0;
  const size_t take = std::min<size_t>(a, std::min<size_t>(n, 64));
  if (!take) return 0;
  uint8_t raw[64];
  hal_rdburst(REG_FIFO, take, raw);
  bool sync_found = false;
  const size_t received = rx_software_sync_.filter(raw, take, b, n, &sync_found);
  return received;
}
size_t Cc1101Hal::read_fifo(uint8_t*b,size_t n){return poll_rx_drain(b,n);}
bool Cc1101Hal::is_pkt_ok(){return radio_ready_ && ((hal_rd(REG_PKTSTATUS)&0x80)!=0);}
int8_t Cc1101Hal::get_rssi_dbm(){if(!radio_ready_)return -127;
  // Direct RSSI status register (0x34) read: RadioLib's getRSSI() returns a
  // code, not dBm, and kept reading register 0 in this state (-74 constant).
  const int8_t code = (int8_t) hal_rd(0x34);
  return (int8_t)(code / 2 - 74);
}
uint8_t Cc1101Hal::get_rssi_code(){return radio_ready_ ? hal_rd(0x34) : 0;}
uint8_t Cc1101Hal::get_fifo_flags(){return radio_ready_ ? hal_rd(REG_RXBYTES) : 0;}
bool Cc1101Hal::read_gpio3(){return gdo0_ >= 0 && gpio_get_level((gpio_num_t) gdo0_) == 1;}
uint8_t Cc1101Hal::scan_channels(int8_t*s){
  if(!radio_ready_) return 0;
  for(uint8_t i=0;i<4;i++){ set_rx_channel(i); hal_rx(); delay(2); s[i]=get_rssi_dbm(); hal_idle(); }
  return 0;
}
void Cc1101Hal::write_reg(uint8_t a,uint8_t v){ if (radio_ready_) hal_wr(a, v); }
uint8_t Cc1101Hal::read_reg(uint8_t a){return radio_ready_ ? hal_rd(a) : 0xFF;}
}  // namespace esphome::nartis_rf_meter
