# SALS
Enable LoRaWAN communications on your [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/) or any RP2040 based board using a [Semtech SX1262 radio module](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262).

Based on the Semtech's [LoRaWAN end-device stack implementation and example projects](https://github.com/Lora-net/LoRaMac-node) with a slight [edit](https://github.com/Prop4et/LoRaMac-node.git) to force the node on a single frequency with a SF7BW125 to communicate with a nanogateway.

The Raspberry Pi Pico is connected to a [Bosch BME688](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme688/) sensor that reads temperature, atmospherical pressure, humidity and air quality measured in Ohm. 

The project also includes the [little-fs library](https://github.com/lurk101/littlefs-lib/tree/4364b3ac5e91a0be8b6f6138318f4e47fda4a6a7) to create a little filesystem on the pico that is used to save the state file for the BME688
## Hardware

 * RP2040 board
   * [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/)
 * Semtech SX1262 board
   * [Waveshare EU868 LoRa Hat](https://www.waveshare.com/pico-lora-sx1262-868m.htm)
 * Bosch BME688
   * [Adafruit BME688](https://www.adafruit.com/product/5046) 

### Default Pinout

| Raspberry Pi Pico / RP2040 | Semtech SX1262 |
| ----------------- | -------------- |
| 3.3V | VCC |
| GND | GND |
| GPIO 10 | SCK |
| GPIO 11 | MOSI |
| GPIO 12 | MISO |
| GPIO 3 | NSS / CS |
| GPIO 15 | RESET |
| GPIO 2 | BUSY
| GPIO 20 | DIO1 / G1 |

| Raspberry Pi Pico / Waveshare SX1262 | Adafruit BME688 |
| ----------------- | -------------- |
| 3.3V | VCC |
| GND | GND | 
| GND | SDO |
| GPIO 8 | SDA / SDI |
| GPIO 9 | SCL / SCK |

| Raspberry Pi Pico / Waveshare SX1262 | little-fs |
| ----------------- | -------------- |
| GPIO 17 | GND / non formatting filesystem| 
| GPIO 17 | GPIO 16 / formatting filesystem|
The first time the Raspberry Pi Pico is switched on it should be GPIO 17 - GPIO 16

| Raspberry Pi Pico / Waveshare SX1262 | Adafruit SD breakout |
| ----------------- | -------------- |
| 3.3v | 3v | 
| GND | GND |
| GPIO 4 | DO |
| GPIO 5 | CS |
| GPIO 6 | CLK |
| GPIO 7 | DI |


