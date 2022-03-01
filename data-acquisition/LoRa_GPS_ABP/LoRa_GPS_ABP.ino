/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Sends a valid LoRaWAN packet with gps data (latitute, longitude, altitude, hdop)
 * as payload, using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(4,3);

/* enable define to send data via LoRa on single channel */
//#define SINGLE_CHANNEL

//#define CFG_eu868

// LoRaWAN NwkSKey, network session key, big-endian (aka msb)
static const PROGMEM u1_t NWKSKEY[16] = { 0x7F, 0xED, 0x79, 0x8E, 0x96, 0x59, 0x98, 0xED, 0x19, 0xEE, 0x2A, 0x05, 0xF3, 0xB1, 0xE9, 0x57 };

// LoRaWAN AppSKey, application session key, big-endian (aka msb)
static const u1_t PROGMEM APPSKEY[16] = { 0x3F, 0x98, 0xFA, 0x6B, 0xD1, 0xD6, 0xDB, 0x3B, 0x9F, 0x6B, 0xE7, 0x47, 0xD9, 0xA5, 0x5E, 0x74 };

// LoRaWAN end-device address (DevAddr), big-endian (aka msb)
static const u4_t DEVADDR = 0x2601107D ;

// center of operating area used for compression of LoRa payload, has to be the same as in TTN Payload Formatter
const long gps_center_lat = 51035730; // use 50123456 for 50.123456
const long gps_center_lon = 13734430; // use 13123456 for 13.123456

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 15;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
uint8_t txBuffer[6];


static void smartdelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (ss.available()) {
            gps.encode(ss.read());
        }
    } while(millis() - start < ms);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            Serial.print(LMIC.seqnoUp);
            Serial.print(", ");
            Serial.println(LMIC.seqnoDn);
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    long lat_gps, lon_gps, alt_gps;
    unsigned long age_gps, hdop_gps;

    gps.get_position(&lat_gps, &lon_gps, &age_gps);
    hdop_gps = gps.hdop(); // dilution of precision in 100ths
    alt_gps = gps.altitude(); // altitude in cm

    Serial.print(F("HDOP="));
    Serial.print(hdop_gps == TinyGPS::GPS_INVALID_HDOP ? -1 : hdop_gps);
    Serial.print(F(" Age="));
    Serial.print(age_gps == TinyGPS::GPS_INVALID_AGE ? -1 : age_gps);
    Serial.print(F(" Latitude="));
    Serial.print(lat_gps == TinyGPS::GPS_INVALID_ANGLE ? -1 : lat_gps);
    Serial.print(F(" Longitude="));
    Serial.print(lon_gps == TinyGPS::GPS_INVALID_ANGLE ? -1 : lon_gps);
    Serial.print(F(" Altitude="));
    Serial.println(alt_gps == TinyGPS::GPS_INVALID_ALTITUDE ? -1 : alt_gps);
  
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // in case of invalid data or old data: just send notification as heartbeat
        if (age_gps == TinyGPS::GPS_INVALID_AGE     
                || hdop_gps == TinyGPS::GPS_INVALID_HDOP
                || lat_gps == TinyGPS::GPS_INVALID_ANGLE
                || lon_gps == TinyGPS::GPS_INVALID_ANGLE
                || alt_gps == TinyGPS::GPS_INVALID_ALTITUDE
                || age_gps > 2000){
            // Prepare upstream data transmission at the next possible time. (port == 2)
            uint8_t empty = 0x00;
            LMIC_setTxData2(2, &empty, sizeof(empty), 0);
            Serial.println(F("Empty heartbeat packet queued"));
        } else {

            // build the TTN packet
            uint16_t lat_msg, lon_msg, alt_msg, hdop_msg;
            
            lat_msg = (((lat_gps - gps_center_lat) /  8) + 16384) & 0x7fff; // 15 bit, precision 8e-6
            lon_msg = (((lon_gps - gps_center_lon) / 16) + 16384) & 0x7fff; // 15 bit, precision 1.6e-5
            // TODO check if out of operating area, then send different packet with less precision or different compression

            // clip hdop to range 0..511
            hdop_msg = (hdop_gps > 0x01ff) ? (0x01ff) : (hdop_gps & 0x01ff); // 9 bit, range 0.0 .. 5.11 

            // clip alt to range 0..1023
            if (alt_gps < 0) {
                alt_msg = 0;
            } else {
                alt_msg = (alt_gps / 200 > 0x01ff) ? (0x01ff) : ((alt_gps / 200) & 0x01ff); // 9 bit, range 0m .. 1023m, precision 2m
            }
            
            txBuffer[0] = ( hdop_msg >> 1 ) & 0xff; // first 8 bits of hdop
            txBuffer[1] = ( ( hdop_msg << 7 ) & 0x80) | ( ( lat_msg >> 8 ) & 0x7f ); // last bit of hdop and first 7 of lat
            txBuffer[2] = lat_msg & 0xff; // last 8 bits 7 of lat
            
            txBuffer[3] = ( alt_msg >> 1 ) & 0xff; // first 8 bits of alt
            txBuffer[4] = ( ( alt_msg << 7 ) & 0x80) | ( ( lon_msg >> 8 ) & 0x7f ); // last bit of alt and first 7 of lon
            txBuffer[5] = lon_msg & 0xff; // last 8 bits 7 of lon
          
            // Prepare upstream data transmission at the next possible time. (port == 1)
            LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
            Serial.print(F("GPS packet queued: 0x"));
            Serial.print(txBuffer[0], 16);
            Serial.print(txBuffer[1], 16);
            Serial.print(txBuffer[2], 16);
            Serial.print(txBuffer[3], 16); 
            Serial.print(txBuffer[4], 16);
            Serial.println(txBuffer[5], 16);
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

    Serial.begin(115200);
    Serial.println(F("Starting"));

    ss.begin(9600);

    // LMIC init
    os_init_ex(&lmic_pins);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    #ifdef SINGLE_CHANNEL
    #warning Using single channel configuration!
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI); // Change SF here!
    for(int i = 1; i <= 8; i++) LMIC_disableChannel(i);
    #else
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,27);

    // try to fetch enough gps data after startup before first packet is queued
    smartdelay(5000);

    // Start job
    do_send(&sendjob);
}

void loop() {
    // run the radio loop
    os_runloop_once();

    //read GPS software serial
    smartdelay(0);
}
