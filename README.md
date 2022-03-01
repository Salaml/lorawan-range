# LoRaWAN range test

Test range of gateway and visualize results.

- [data acquisition](./data-acquisition):
  - acquire data with node sending current position via LoRaWAN
  - code for Arduino Uno with LoRa- and GPS-Shield is available
  - every node capable of sending current position is usable
  - Storage Integration in TTN has to be activated to store data during acquisition phase
- [data conversion](./data-conversion):
  - query stored measurement data from TTN
  - convert data to GeoJSON format readable by presentation
- [presentation](./presentation):
  - visualizes acquired data
  - copy created GeoJSON files in [data folder](./presentation/data)
  - put entire presentation folder onto webserver
  - if PHP is disabled:
    - load files in presentation by adding filenames of copied files in [index.html](./presentation/index.html) at line 42 and following
    - open [index.html](./presentation/index.html) in browser to view results
  - if PHP is enabled: open [index.php](./presentation/index.php) in browser to view results

# License
[MIT License](https://mit-license.org/)
