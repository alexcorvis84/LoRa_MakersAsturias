/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * 
 * Modified 2019 by @AlexCorvis84 for BricoLabs ESP8266 LoRa shield
 * 
 * ----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * IMPORTANT NOTE: Need to connect a pair of Wires from D3 to D1 & from D4 to D2
 * This is needed due the pins used by the PCB (D3 & D4) for DIO0 and DIO1
 * respectively, are used by the ESP8266 on the initialization and will make it 
 * reboot (PCB design error ^^)
 * Changing them to use D1 & D2 instead, makes it to work OK! :-)
 * ----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "MakersAsturias!", 
 * using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt
 * 
 * IRQ working (dio0 connected), detect TC complete, sends repeteadly :)
 *
 *******************************************************************************/

 // References:
 // https://github.com/brico-labs/LoRa/blob/master/WorkshopOSHWDem18/BricoLabs_ESP8266_LoRa_shield/BricoLabs_ESP8266_LoRa_shield-Schematic.pdf

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//Pinout! Customized for BricoLabs ESP8266 LoRa shield!
#define RFM95W_SCK   D5   // GPIO14 -- RFM95W's SCK
#define RFM95W_MISO  D6   // GPIO12 -- RFM95W's MISO
#define RFM95W_MOSI  D7   // GPIO13 -- RFM95W's MOSI
#define RFM95W_CS    D8   // GPIO15 -- RFM95W's CS
#define RFM95W_DI_OR D3   // GPIO0  -- (DIO0) RFM95W's IRQ(Interrupt Request) NOT USED! Pin not connected to ESP8266!
//#define GPIO02     D4   // GPIO2  -- (DIO1)
//#define GPIO04     D2   // GPIO4     
//#define GPIO05     D1   // GPIO5
//#define GPIO16     D0   // GPIO16    (DIO2) Pin not connected to ESP8266!

// Pin mapping
// Adapted for ESP8266 BricoLabs LoRa Board
const lmic_pinmap lmic_pins = {
  .nss  = RFM95W_CS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst  = LMIC_UNUSED_PIN,
  .dio  = {D1, D2, LMIC_UNUSED_PIN},  //{GPIO05 , GPIO04 ,LMIC_UNUSED_PIN}
                                                                /*If you don't have any DIO pins connected to GPIO (new software feature)
                                                                  you just need to declare 3 .dio to LMIC_UNUSED_PIN, in your sketch 
                                                                  That's all, stack will do the job for you. */
};

static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u4_t DEVADDR = 0x00000000 ; // Node address

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "MakersAsturias!";
static osjob_t sendjob;

const unsigned TX_INTERVAL = 15; // Schedule TX every this many seconds

int channel = 0;
int dr = DR_SF7;

void displayInfo(String info){
  Serial.println(info);
}

void onEvent (ev_t ev) {
  switch(ev)
  {
      case EV_SCAN_TIMEOUT:   displayInfo("SCAN_TIMEOUT"); break;
      case EV_BEACON_FOUND:   displayInfo("BEACON_FOUND"); break;
      case EV_BEACON_MISSED:  displayInfo("BEACON_MISSED"); break;
      case EV_BEACON_TRACKED: displayInfo("BEACON_TRACKED"); break;
      case EV_JOINING:        displayInfo("JOINING"); break;
      case EV_RFU1:           displayInfo("RFU1"); break;
      case EV_LOST_TSYNC:     displayInfo("LOST_TSYNC"); break;
      case EV_RESET:          displayInfo("RESET"); break;
      case EV_RXCOMPLETE:     displayInfo("RXCOMPLETE"); break;
      case EV_LINK_DEAD:      displayInfo("LINK_DEAD"); break;
      case EV_LINK_ALIVE:     displayInfo("LINK_ALIVE"); break;
      case EV_JOIN_FAILED:    displayInfo("JOIN_FAILED"); break;
      case EV_REJOIN_FAILED:  displayInfo("REJOIN_FAILED"); break;
      case EV_JOINED:
        displayInfo("REJOIN_FAILED");
        LMIC_setLinkCheckMode(0);
        break;
      case EV_TXCOMPLETE:
          displayInfo("TXCOMPLETE");
          
          if (LMIC.txrxFlags & TXRX_ACK)
            displayInfo("Received ACK");
          if (LMIC.dataLen) {
            Serial.print("RX:");
            Serial.print(LMIC.dataLen);
            Serial.print(" bytes RSSI:");
            Serial.print(LMIC.rssi);
            Serial.print("SNR:");
            Serial.println(LMIC.snr);
          }
          // Schedule next transmission
          os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
          break;
  }
}

void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) 
  {
      displayInfo("OP_TXRXPEND, not sent");
  }
  else 
  {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
      displayInfo("PACKET QUEUED");
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  
  Serial.begin(115200);
  Serial.println("\nWemosD1 Mini + BricoLabs LoRa Shield Node TTN");
  
  os_init(); // LMIC init
  
  LMIC_reset(); // Reset MAC state. Session &pending data transfers discarded.

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
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  LMIC_setLinkCheckMode(0); // Disable link check validation
  LMIC.dn2Dr = DR_SF9;  // TTN uses SF9 for its RX2 window.
  LMIC_setDrTxpow(DR_SF7,14); // Set data rate and transmit power for uplink

  // Disables all channels, except for the one defined above, and sets the
  // data rate (SF). This only affects uplinks; for downlinks the default
  // channels or the configuration from the OTAA Join Accept are used.
  // Not LoRaWAN compliant; FOR TESTING ONLY!
  for (int i = 0; i < 9; i++) // For EU; for US use i<71
  {
    if (i != channel) LMIC_disableChannel(i);
  }
  LMIC_setDrTxpow(dr, 14); // Set data rate (SF) and transmit power for uplink
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  do_send(&sendjob); // Start job
}

void loop() {
  os_runloop_once();
}
