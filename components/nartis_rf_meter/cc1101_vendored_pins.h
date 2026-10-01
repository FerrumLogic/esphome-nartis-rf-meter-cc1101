#pragma once
// ---------------------------------------------------------------------------
// Vendoring patch: these five macros were Kconfig options in the upstream
// esp-idf component. ESPHome has no Kconfig, so the pins come from our YAML
// (nartis_rf_meter: pin_sdio / pin_sclk / pin_fcsb / pin_csb / pin_gpio3).
// The values below match the active configuration; the radio-layer rewrite
// will pass them at runtime instead of baking them in.
// ---------------------------------------------------------------------------
#ifndef CC1101_PIN_MISO
#define CC1101_PIN_MISO 6    // YAML: pin_sdio  (module SDIO/MISO)
#endif
#ifndef CC1101_PIN_SCK
#define CC1101_PIN_SCK 4     // YAML: pin_sclk
#endif
#ifndef CC1101_PIN_MOSI
#define CC1101_PIN_MOSI 5    // YAML: pin_fcsb  (module FCSB/MOSI)
#endif
#ifndef CC1101_PIN_CSN
#define CC1101_PIN_CSN 7     // YAML: pin_csb   (module CSB/CSN)
#endif
#ifndef CC1101_PIN_GDO0
#define CC1101_PIN_GDO0 15   // YAML: pin_gpio3 = module GDO0 (must be IRQ-capable)
#endif
