
#include <Arduino.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "SSD1306.h"

#define DEVICE_TESTESP32
#include "lorakeys.h"

#define RSTOLED  16
#define Light  25

void do_send();

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
// defined in lorakeys.h
void getArtEui(uint8_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
// defined in lorakeys.h
// static const uint8_t PROGMEM DEVEUI[8]={ 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void getDevEui(uint8_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
// defined in lorakeys.h

uint8_t data[4] = {};

//SSD1306  display(0x3c, SDA, SCL);
SSD1306  display(0x3c, 4, 15);

OsJob sendjob;

bool nosleep = false;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(60 * 5);

const unsigned int BAUDRATE = 19200;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33},
};

void showState(const char* state) {
    display.clear();
    display.drawString(5, 5, state);
    display.display();
}


void onEvent(ev_t ev)
{
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        PRINT_DEBUG_2("EV_SCAN_TIMEOUT");
        break;
    case EV_BEACON_FOUND:
        PRINT_DEBUG_2("EV_BEACON_FOUND");
        break;
    case EV_BEACON_MISSED:
        PRINT_DEBUG_2("EV_BEACON_MISSED");
        break;
    case EV_BEACON_TRACKED:
        PRINT_DEBUG_2("EV_BEACON_TRACKED");
        break;
    case EV_JOINING:
        PRINT_DEBUG_2("EV_JOINING");
        showState("JOINING");

        break;
    case EV_JOINED:
        PRINT_DEBUG_2("EV_JOINED");
        showState("JOINED");

        // LMIC.setDrTxpow(DR_SF9, KEEP_TXPOW);
        break;
    case EV_RFU1:
        PRINT_DEBUG_2("EV_RFU1");
        break;
    case EV_JOIN_FAILED:
        PRINT_DEBUG_2("EV_JOIN_FAILED");
        showState("JOIN FAIL");

        break;
    case EV_REJOIN_FAILED:
        PRINT_DEBUG_2("EV_REJOIN_FAILED");
        break;
    case EV_TXCOMPLETE:
        PRINT_DEBUG_2("EV_TXCOMPLETE (includes waiting for RX windows)");
        showState("TX COMP");

        if (LMIC.txrxFlags & TXRX_ACK)
            PRINT_DEBUG_2("Received ack");
        if (LMIC.dataLen)
        {
            PRINT_DEBUG_2("Received %d  bytes of payload", LMIC.dataLen);
        }
        // Schedule next transmission
        sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);
        break;
    case EV_LOST_TSYNC:
        PRINT_DEBUG_2("EV_LOST_TSYNC");
        break;
    case EV_RESET:
        PRINT_DEBUG_2("EV_RESET");
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        PRINT_DEBUG_2("EV_RXCOMPLETE");
        break;
    case EV_LINK_DEAD:
        PRINT_DEBUG_2("EV_LINK_DEAD");

        break;
    case EV_LINK_ALIVE:
        PRINT_DEBUG_2("EV_LINK_ALIVE");
        break;
    default:
        PRINT_DEBUG_2("Unknown event");
        break;
    }
}

void do_send()
{
    // Check if there is not a current TX/RX job running
    if (LMIC.getOpMode() & OP_TXRXPEND)
    {
        PRINT_DEBUG_1("OP_TXRXPEND, not sending");
        // should not happen so reschedule anymway
        sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);
    }
    else
    {

        int16_t temp = temperatureRead() * 10;
        data[0] = 1;
        data[1] = 0x67;
        data[2] = temp>>8;
        data[3] = temp & 0xFF;

        // Prepare upstream data transmission at the next possible time.
        LMIC.setTxData2(1, data, 4, false);
        PRINT_DEBUG_1("Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setup()
{
#if LMIC_DEBUG_LEVEL > 0
    Serial.begin(BAUDRATE);
    Serial.println(F("Starting"));
#endif
    pinMode(25, OUTPUT);
    pinMode(RSTOLED,OUTPUT);
    delay(50); 
    digitalWrite(RSTOLED, LOW);
    delay(50);
    digitalWrite(RSTOLED, HIGH);
    delay(50);
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    showState("Starting");

    delay(10000);

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC.reset();

    uint8_t buf[16];
    memcpy_P(buf, APPKEY, 16);
    LMIC.aes.setDevKey(buf);
    LMIC.setEventCallBack(onEvent);
    LMIC.setDevEuiCallback(getDevEui);
    LMIC.setArtEuiCallback(getArtEui);

    // set clock error to allow good connection.
    LMIC.setClockError(MAX_CLOCK_ERROR * 5 / 100);

    // for(int i = 1; i <= 8; i++) LMIC_disableChannel(i);
    // LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // TTN uses SF9 for its RX2 window.
    // LMIC.dn2Dr = DR_SF9;
    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    // LMIC_setDrTxpow(DR_SF9,14);

    // Start job (sending automatically starts OTAA too)
    do_send();
}


void loop()
{
    OsDeltaTime to_wait = OSS.runloopOnce();
    if (!nosleep && to_wait > 0 && hal_is_sleep_allow())
    {
       // powersave(to_wait);
    }
}