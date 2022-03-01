# Data acquisition

## Usage
Arduino with [LoRa and GPS Shield](https://www.dragino.com/products/lora/item/108-lora-gps-shield.html) required

 - select software file depending on activation method of this device set in TTN:
   - OTAA: [LoRa_GPS_OTAA](./LoRa_GPS_OTAA/LoRa_GPS_OTAA.ino)
   - ABP: [LoRa_GPS_ABP](./LoRa_GPS_ABP/LoRa_GPS_ABP.ino)
 - choose center of operating range and set latitude and longitude:
   - set in Arduino code:
     - set *gps_center_lat* to latitude
     - set *gps_center_lon* to longitude
   - set in payload formatter:
     - set *center_lat* to latitude
     - set *center_lon* to longitude
 - programm Arduino with selected file using Arduino IDE
 - set [payload formatter](./payload_formatter.js) in TTN
 - activate Storage Integration in TTN

## Coding

### Latitude
input: 51.083.948
output: (center - 4\*32767) ... (center + 4\*32767)

lat = (uint16)((gps.latitude() - center) / 4)

-> 2 Byte (precision 4e-6) -> /2 & 0x7fff -> 15 Bit (precision 8e-6)

### Longitude
input: 13.727.262
output: (center - 8\*32767) ... (center + 8\*32767)

lon = (uint16)((gps.longitude() - center) / 8)

-> 2 Byte (precision 8e-6) -> /2 & 0x7fff -> 15 Bit (precision 1.6e-5)

### HDOP
input: 0 ... 5.11
output: 0.0 (0x0) .. 5.11 (0x01ff)

hdop = (gps.hdop() & 0x01ff)

->  9 Bit

### Altitude
input: 0.00m ... 1023.45m
output: 0m (0x0) ... +1023m (0x3ff)

alt = ((gps.altitude() / 100) -> 10 Bit (precision 1m) -> /2 & 0x1ff

-> 9 Bit (precision 2m)

### Example
input:
 - HDOP: 155 (1.55)
 - Latitude: 51166690 (51.166690°)
 - Longitude: 13996446 (13.996446°)
 - Altitude: 130.42 (130.42m)
 - with center set to:
   - center_lat: 51035730
   - center_lon: 13734430
output bytes: 4D FF F2 20 FF F8

### Operating Area
visualization of operating area around center coordinates in [operating_range.geojson](./operating_range.geojson):
