<html>
	<head>
		<meta http-equiv="content-type" content="text/html; charset=utf-8">
		<title>Reichweite LoRa</title>
		<meta name="viewport" content="initial-scale=1.0, user-scalable=no" />
		<link rel="stylesheet" href="leaflet/leaflet.css" />
		<style type="text/css">
			.info { padding: 6px 8px; font: 16px Arial, Helvetica, sans-serif; background: white; background: rgba(255,255,255,1.0); box-shadow: 0 0 15px rgba(0,0,0,0.2); border-radius: 5px; }
			.legend { line-height: 18px; }
			.legend i { width: 18px; height: 18px; float: left; margin-right: 8px; opacity: 1; }
		</style>
		<script src="leaflet/leaflet.js"></script>
		<script src="leaflet/plugins/leaflet-hash/leaflet-hash.js"></script>
		<script type="text/javascript">
			var geojson;

			var layerControl = L.control.layers(null, null);

			var info = L.control({position: 'bottomright'});
			info.onAdd = function (map) {
				this._div = L.DomUtil.create('div', 'info'); // create a div with a class "info"
				this.update();
				return this._div;
			};

			var legend = L.control({position: 'bottomleft'});
			legend.onAdd = function (map) {
				var div = L.DomUtil.create('div', 'info legend'),
					grades = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10],
					labels = [];

				div.innerHTML += "<b>SNR in dB</b><br>";
				// loop through density intervals and generate a label with a colored square for each interval
				for (var i = 0; i < grades.length; i++) {
					div.innerHTML +=
						'<i style="background:' + colorBySNR(grades[i]) + '"></i> ' +
						(i == 0 ? '< ' + grades[i+1] : (i + 1 == grades.length ? '≥ ' + grades[i] : grades[i] + ' &ndash; ' + grades[i + 1])) + '<br>';
				}
				return div;
			};

			// load all GeoJSON files in data folder
			<?php
			foreach(glob("./data/*.geojson") as $filename){
				echo "loadData('$filename');\n";
			}
			?>

			// show route of data acquisition
			loadRoute('./route_210614.geojson');

			function loadRoute(path) {
				// show route at back behind data
				loadJSONData(path, (x) => L.geoJson(x).addTo(map).bringToBack());
			}
			function loadData(path) {
				loadJSONData(path, parseData)
			}
			function loadJSONData(path, action)
			{
				var req = new XMLHttpRequest();
				req.open('GET', path, true); 
				req.onload = function () {
					if (req.status === 200) {
						action(JSON.parse(req.responseText));
					}
				};
				req.send();
			}

			function highlightFeature(e) {
				var layer = e.target;

				layer.setStyle({
					weight: 3,
					color: '#fff',
					dashArray: '',
					fillOpacity: 1
				});

				if (!L.Browser.ie && !L.Browser.opera && !L.Browser.edge) {
					layer.bringToFront();
				}
				info.update(layer.feature.properties);
			}
			function resetHighlight(e) {
				geojson.resetStyle(e.target);
				info.update();
			}
			function zoomToFeature(e) {
				map.fitBounds(e.target.getBounds());
			}
			function onEachFeature(feature, layer) {
				if (feature.properties.type !== "gateway")
					layer.on({
						mouseover: highlightFeature,
						mouseout: resetHighlight,
						click: zoomToFeature
					});
			}

			function parseData(data)
			{
				let gateway_data = data.features.filter(feature => feature.properties.type === "gateway");
				let gateway_name;
				if (gateway_data.length == 0)
					gateway_name = "unknown gateway";
				else
					// use gateway name as name of layer, gateway id or eui as fallback if name unknown
					gateway_name = gateway_data[0].properties.name || gateway_data[0].properties.gateway_id || gateway_data[0].properties.eui || "unknown gateway";
				
				geojson = L.geoJson(data, {
					style: style,
					onEachFeature: onEachFeature,
					pointToLayer: function (feature, latlng) {
						if (feature.properties.type === "gateway")
							return L.marker(latlng).bindPopup(
									"<div class='infoblock'><b style='font-size: 16px;'>"+feature.properties.name+"</b><table>" +
									"<tr><td>ID: </td><td>" + feature.properties.gateway_id + "</td></tr>" +
									"<tr><td>EUI: </td><td>" + feature.properties.eui + "</td></tr>" +
									"<tr><td>Altitude: </td><td>" + feature.properties.altitude + " m</td></tr>" +
									"</table></div>"
									);
						else
							return L.circleMarker(latlng, {radius: 9, color: '#000'});
					}
				}).addTo(map);
				layerControl.addOverlay(geojson, gateway_name);
				layerControl.addTo(map);

				// method that we will use to update the control based on feature properties passed
				info.update = function (props) {
					this._div.innerHTML = props ?
						"<div class='infoblock'><b>GPS</b><table>" +
						"<tr><td>Altitude: </td><td>" + props.altitude + " m</td></tr>" +
						"<tr><td>HDOP: </td><td><font color='" + (isHDOPok(props.hdop) ? 'black' : 'red') + "'>" + props.hdop + "</font></td></tr>" +
						"</table></div><br>" +
						"<div class='infoblock'><b>Lora</b><table>" +
						"<tr><td>Frequency: </td><td>" + (props.frequency / 1000000) + " MHz</td></tr>" +
						"<tr><td>SF: </td><td>" + props.spreading_factor + "</td></tr>" +
						"<tr><td>Coding Rate: </td><td>" + props.coding_rate + "</td></tr>" +
						"</table></div><br>" +
						"<div class='infoblock'><b>Signal</b><table>" +
						"<tr><td>SNR: </td><td>" + props.snr + " dB</td></tr>" +
						"<tr><td>RSSI: </td><td>" + props.rssi + " dBm</td></tr>" +
						"</table></div>"
						: "<div class='infoblock'><b>Hover over a point for details</b></div>";
				};

				info.addTo(map);
				legend.addTo(map);
			}

			function isHDOPok(hdop) {
				return hdop < 2.5;
			}

			function colorBySNR(snr)
			{
				return snr < -8 ? '#a50026' : snr < -6 ? '#d73027' : snr < -4 ? '#f46d43' : snr < -2 ? '#fdae61' : snr < 0 ? '#fee08b' : 
						snr < 2 ? '#ffffbf' : snr < 4 ? '#d9ef8b' : snr < 6 ? '#a6d96a' : snr < 8 ? '#66bd63' : snr < 10 ? '#1a9850' : '#006837';
			}

			function style(feature) {
				if (isHDOPok(feature.properties.hdop))
					return {
						fillColor: colorBySNR(feature.properties.snr), weight: 2,
						opacity: 1, fillOpacity:1, dashArray: '0'
					};
				else
					// draw points with bad HDOP transparent with dashed border
					return {
						fillColor: colorBySNR(feature.properties.snr), weight: 2,
						opacity: 0.5, fillOpacity: 0.5, dashArray: '6,6'
					};
			}
		</script>
	</head>
	<body style="margin:0px">
		<div id="map" style="height: 100%;"></div>
		<script>
			var map = new L.Map('map').setView([51.03562, 13.73465], 13);
			L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
				attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
				maxZoom: 19
			}).addTo(map);

			// activate permalink
			var hash = new L.Hash(map);
		</script>
	</body>
</html>
