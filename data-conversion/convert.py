from configparser import ConfigParser
from requests import get
from json import loads
from geojson import Feature, Point, FeatureCollection, dump


# queries given path of TTN API with given parameters, authentication with given API key
def query_ttn(address, api_key, path, params):
    url = f"https://{address}/api/v3/{path}"
    # authenticate via API key
    headers = {"Authorization": f"Bearer {api_key}"}

    resp = get(url, headers=headers, params=params)
    resp.close()
    # API request returns newline-delimited JSON data, convert to Python list
    return resp.text.splitlines()


# queries past uplink messages from TTN via storage API, returns a list of JSON strings
def query_ttn_uplink_messages(address, app_id, api_key):
    path = f"as/applications/{app_id}/packages/storage/uplink_message"

    # only query for decoded payload, rx metadata (SNR, ...), settings (frequency, spreading factor) and simulated flag
    params = {
        'field_mask': 'up.uplink_message.decoded_payload,up.uplink_message.rx_metadata,up.uplink_message.settings,simulated',
        #'after': '2021-06-14T13:15:00.000000Z'
    }

    return query_ttn(address, api_key, path, params)


# queries location of gateway from TTN API, returns (lat, lon) or None
def query_ttn_gateway(address, api_key, gateway_id):
    path = f"gateways/{gateway_id}"
    params = {'field_mask': 'name,antennas,location_public'}

    query_result = query_ttn(address, api_key, path, params)
    gateway_data = loads(query_result[0])

    gateway_info = {"gateway_id": gateway_id}
    if 'message' not in gateway_data:
        try:
            gateway_info["eui"] = gateway_data["ids"]["eui"]
            gateway_info["name"] = gateway_data["name"]
            location = gateway_data["antennas"][0]["location"]
            gateway_info["latitude"] = location["latitude"]
            gateway_info["longitude"] = location["longitude"]
            gateway_info["altitude"] = location["altitude"]
        except KeyError:
            pass
    return gateway_info


# parses list of JSON strings, returns dictionary with one entry for every gateway,
# each containing a dict with one entry for every device,
# each containing a list of messages received by the according gateway from the according device
def parse_json(json_strings, ignore_simulated=True):
    data_gateways = {}

    cnt_messages_all = 0
    cnt_messages_stored = 0

    for line in json_strings:
        data = loads(line)
        if 'error' in data:
            print(data['error']['message'])
            return {}
        else:
            cnt_messages_all += 1
            result = data['result']

            if ignore_simulated and result.get('simulated', False):
                # ignore simulated messages
                continue

            device_id = result['end_device_ids']['device_id']
            message = result['uplink_message']

            # decoded payload is not always available e. g. if payload formatter is broken
            payload = message.get('decoded_payload', {})
            if 'latitude' not in payload or 'longitude' not in payload:
                # ignore messages without coordinates in payload
                continue

            lora_settings = message['settings']
            spreading_factor = lora_settings['data_rate']['lora']['spreading_factor']
            bandwidth = lora_settings['data_rate']['lora']['bandwidth']
            coding_rate = lora_settings.get('coding_rate', None)
            frequency = lora_settings.get('frequency', None)

            # iterate over list of gateways by which message was received
            for gateway_metadata in message['rx_metadata']:
                gateway_id = gateway_metadata['gateway_ids']['gateway_id']

                snr = gateway_metadata.get('snr', None)
                rssi = gateway_metadata.get('rssi', None)

                # store payload of message and metadata
                data_message = dict(payload)
                data_message['spreading_factor'] = spreading_factor
                data_message['bandwidth'] = bandwidth
                if coding_rate is not None:
                    data_message['coding_rate'] = coding_rate
                if frequency is not None:
                    data_message['frequency'] = frequency
                if snr is not None:
                    data_message['snr'] = snr
                if rssi is not None:
                    data_message['rssi'] = rssi

                # create new entry in dict for every gateway_id, each containing another dict with entries for every device_id
                if gateway_id not in data_gateways:
                    data_gateways[gateway_id] = {}

                # create new entry in dict for every device_id, each containing a list with every message
                if device_id not in data_gateways[gateway_id]:
                    data_gateways[gateway_id][device_id] = []

                data_gateways[gateway_id][device_id].append(data_message)

            cnt_messages_stored += 1

    print('Stored messages:', cnt_messages_stored)
    print('Ignored messages:', cnt_messages_all - cnt_messages_stored)

    return data_gateways


# converts message_data list with entries for latitude and longitude to GeoJSON feature collection
# and puts remaining JSON data in properties of features
# converts gateway_data to GeoJSON feature and adds to feature collection
def create_geojson(message_data, gateway_data):
    features = []

    gateway_pos = None
    if 'latitude' in gateway_data and 'longitude' in gateway_data:
        gateway_pos = Point((gateway_data.pop('longitude'), gateway_data.pop('latitude')))
    gateway_properties = dict(gateway_data)
    gateway_properties['type'] = 'gateway'

    gateway = Feature(geometry=gateway_pos, properties=gateway_properties)
    features.append(gateway)

    for item in message_data:

        pos = Point((item.pop('longitude'), item.pop('latitude')))

        properties = dict(item)

        feature = Feature(geometry=pos, properties=properties)
        features.append(feature)

    collection = FeatureCollection(features)
    return collection


# queries uplink messages from TTN v3 application given in config.ini
# and creates a GeoJSON file containing points with metadata from LoRa communication (SNR, RSSI, SF, ...)
# different file for every combination of receiving gateway and sending device
if __name__ == '__main__':
    # read config from file config.ini
    config = ConfigParser()
    config.read('config.ini')

    ttn_app_id = config['TTN_V3']['TTN_APP_ID']
    ttn_api_key = config['TTN_V3']['TTN_API_KEY']
    ttn_address = config['TTN_V3']['TTN_ADDRESS']

    ignore_simulated = config['SIMULATION'].getboolean('IGNORE_SIMULATED_MESSAGES')
    aggregate_gateway = config['AGGREGATION'].getboolean('AGGREGATE_GATEWAY')

    uplink_messages = query_ttn_uplink_messages(ttn_address, ttn_app_id, ttn_api_key)

    parsed_data = parse_json(uplink_messages, ignore_simulated=ignore_simulated)

    for gateway_id, gateway_data in parsed_data.items():

        gateway_info = query_ttn_gateway(ttn_address, ttn_api_key, gateway_id)
        print(f'Gateway {gateway_id} ({gateway_info.get("name", "no name")}, {gateway_info.get("eui", "no EUI")}): ')
        if 'latitude' in gateway_info and 'longitude' in gateway_info:
            print(f'\tPosition {gateway_info["latitude"]}, {gateway_info["longitude"]}')

        if aggregate_gateway:
            # flatten list with lists of messages for every device into single list
            data_all_devices = [message for devices in gateway_data.values() for message in devices]

            geojson_data = create_geojson(data_all_devices, gateway_info)

            # store in file with gateway ID as filename
            filename = f'{gateway_id}.geojson'
            with open(filename, 'w') as file:
                dump(geojson_data, file)
                print(f'\tResults in file "{filename}"')
        else:
            for device_id, device_data in gateway_data.items():

                # convert every combination of device and gateway separately
                geojson_data = create_geojson(device_data, gateway_info)

                # store in file with IDs as filename
                filename = f'{gateway_id}_{device_id}.geojson'
                with open(filename, 'w') as file:
                    dump(geojson_data, file)
                    print(f'\tResults for messages sent by device "{device_id}" in file "{filename}"')