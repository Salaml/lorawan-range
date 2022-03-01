# Data conversion

get stored GPS data from TTN and convert it to format needed for presentation

 - queries uplink messages from TTN v3 application
 - creates a GeoJSON file containing points with metadata from LoRa communication (SNR, RSSI, SF, ...)
 - uses Python library [geojson](https://pypi.org/project/geojson/)

Parameters in [config.ini](./config.ini):
 - TTN_APP_ID: application ID of TTN application containing used device
 - TTN_API_KEY: API key to access application, granted with at least following right: Read application traffic (uplink and downlink)
 - TTN_ADDRESS: address of TTN instance used
 - IGNORE_SIMULATED_MESSAGES: ignore or include messages simulated in TTN console
 - AGGREGATE_GATEWAY: create one file for every receiving gateway or one file for every combination of receiving gateway and sending device



# File Structure
Contains 1 feature collection with:
 - 0 or 1 features containing information about gateway, must contain `properties.type == "gateway"`
 - 0 to n features containing measurement points, must not contain `properties.type == "gateway"` 
```json
{
	"type": "FeatureCollection",
	"features":
	[
		{
			"type": "Feature",
			"geometry": {"type": "Point", "coordinates": [13.73465, 51.03562]},
			"properties": {
				"gateway_id": "htw-dresden-ttn-gw1",
				"eui": "AC1F09FFFE05222E",
				"name": "HTW Dresden TTN Gateway 1",
				"altitude": 138,
				"type": "gateway"
			}
		},
		{
			"type": "Feature",
			"geometry": {"type": "Point", "coordinates": [13.734414, 51.03625]},
			"properties": {
				"altitude": 104,
				"hdop": 2.98,
				"spreading_factor": 7,
				"bandwidth": 125000,
				"coding_rate": "4/5",
				"frequency": "868300000",
				"snr": 11.2,
				"rssi": -72
			}
		},
		{
			"type": "Feature",
			"geometry": {"type": "Point", "coordinates": [13.734414, 51.03625]},
			"properties": {
				"altitude": 104,
				"hdop": 2.98,
				...
			}
		},
		...
	]
}
```

