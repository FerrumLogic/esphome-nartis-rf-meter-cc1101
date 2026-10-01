# Нартис И100/И300 с CC1101

Порт радиослоя [оригинального компонента](https://github.com/latonita/esphome-nartis-rf-meter): CMT2300A заменён на CC1101. Работа с протоколом счётчика и настройка сенсоров остаются такими же, как в оригинальном проекте.

Для счётчиков с CMT2300A используйте оригинальный репозиторий. Совместимость с RF433-2 / Д101-2 здесь не проверялась.

## Подключение CC1101

| CC1101 | ESPHome |
|---|---|
| SO | `pin_sdio` |
| SCK | `pin_sclk` |
| CSN | `pin_csb` |
| SI | `pin_fcsb` |
| GDO0 | `pin_gpio3` |
| VCC | 3.3 V |
| GND | GND |

GDO0 подключите к GPIO ESP32 с поддержкой прерываний. GDO2 не используется.

## Конфигурация ESPHome

```yaml
external_components:
  - source: github://FerrumLogic/esphome-nartis-rf-meter-cc1101
    components: [nartis_rf_meter]

esp32:
  board: esp32dev
  framework:
    type: esp-idf

nartis_rf_meter:
  pin_sdio: 19
  pin_sclk: 18
  pin_csb: 5
  pin_fcsb: 23
  pin_gpio3: 4
  meter_serial: "012345678901"  # номер с шильдика счётчика
  use_alternative_channels: true
  fix_channel: 0
  update_interval: 10min

sensor:
  - platform: nartis_rf_meter
    name: "Active Power"
    obis_code: "1.0.1.7.0.255"
    unit_of_measurement: W
    device_class: power
    state_class: measurement
```

Остальные параметры компонента, примеры сенсоров и справочник OBIS-кодов — в [README оригинального проекта](https://github.com/latonita/esphome-nartis-rf-meter).
