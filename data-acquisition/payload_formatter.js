//Payload formatter for TTN v3
function decodeUplink(input) {
  if (input.fPort == 1)
  {
    // use same operating area as in Arduino code
    var center_lat = 51035730;
    var center_lon = 13734430;
  
    var hdp = ((input.bytes[0]       ) << 1) | ((input.bytes[1] & 0x80) >> 7);
    var lat = ((input.bytes[1] & 0x7f) << 8) | ( input.bytes[2]             );
    var alt = ((input.bytes[3]       ) << 1) | ((input.bytes[4] & 0x80) >> 7);
    var lon = ((input.bytes[4] & 0x7f) << 8) | ( input.bytes[5]             );
    
    return {
      data: {
        latitude:  ((lat - 16384) *  8 + center_lat) / 1000000.0,
        longitude: ((lon - 16384) * 16 + center_lon) / 1000000.0,
        altitude: alt * 2,
        hdop: hdp / 100
      } ,
      warnings: [],
      errors: []
    };
  }
  else if (input.fPort == 2)
  {
    return {
      data: {
        hdop: -1,
        note: "GPS unknown"
      },
      warnings: [],
      errors: []
    };
  }
}