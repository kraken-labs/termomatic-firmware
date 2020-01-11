#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#define CE_PIN PB1
#define CSN_PIN PB2 // PB0
#define RF_CHANNEL 40

#include "Wire.h"

#define DEVICE_TYPE_CASAMATIC 1
#define THIS_DEVICE_TYPE 3 // Termomatic

#define CASAMATIC_EVENT_DEVICE_ADDED 1

#define EVENT_LINK_PRESENT 0 // Link
#define EVENT_CURRENT_TEMP 1 // Current Temp
#define EVENT_SET_TEMP 2 // New Set Temp
#define EVENT_SET_MODE 3 // Set mode

#define DS18B20_PIN 2 // PIN_B2

#include "PinChangeInterrupt.h"

#define PIN_PULSADOR_A PIN_A2
#define PIN_PULSADOR_B PIN_A1
#define PIN_PULSADOR_C PIN_A0

const byte pulsadores[] = { PIN_PULSADOR_A, PIN_PULSADOR_B, PIN_PULSADOR_C };

#define PRESS_STATE_IDLE 0
#define PRESS_STATE_STARTED 1

#define DETECT_SHORT_MS 5
#define DETECT_LONG_MS 2000
#define DETECT_DOUBLE_MS 400

#define DEBUG 1

struct event_t {
  char from[5];
  byte device_type;
  byte event_type;
  char to[5];
  byte data;
};

uint32_t currentMillis = 0;

volatile byte pressState[3]  = {PRESS_STATE_IDLE, PRESS_STATE_IDLE, PRESS_STATE_IDLE};
volatile unsigned long initPressMillis[3] = {0, 0, 0};
volatile unsigned long prevPressMillis[3] = {0, 0, 0};
unsigned long diffMillis[3] = {0, 0, 0};

#define STATE_BOOT 0
#define STATE_ERROR 1
#define STATE_LINK 2
#define STATE_ACTIVE 3
#define STATE_CLEAR 4

byte CURRENT_STATE = STATE_BOOT;

#define MODE_HEAT 0
#define MODE_COLD 1

byte CURRENT_MODE = MODE_HEAT;

/*
void setPulsador(byte i) {
  if (pressState[i] == PRESS_STATE_IDLE) {
    if (((PINA >> pulsadores[i])& 1) == 1) {
      pressState[i] = PRESS_STATE_STARTED;
      initPressMillis[i] = millis();
    }
  }
}
*/

/*
void setPulsadorA(void) { setPulsador(0); }
void setPulsadorB(void) { setPulsador(1); }
void setPulsadorC(void) { setPulsador(2); }
*/

#include <EEPROM.h>
#include "EEPROMAnything.h"
#define EEPROM_ADDR_STATE 0
#define EEPROM_ADDR_MAC 1
#define EEPROM_ADDR_TEMP_SET 10
//char mac[5] = { 0 };
char mac[5] = { 'f', 'f', 'f', 'f', 0 };
volatile byte tempSet = 0;

uint32_t publishTimer = 0;

RF24 radio(CE_PIN, CSN_PIN);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

void storeCounter() {
  EEPROM.update(EEPROM_ADDR_TEMP_SET, tempSet);
}

void publishMsg(byte event_type, int data) {
  char to[5] = { 'f', 'f', 'f', 'f', 0 };
  event_t rf_event;
  rf_event.device_type = THIS_DEVICE_TYPE;
  rf_event.event_type = event_type;
  rf_event.data = data;
  memcpy(rf_event.from, mac, sizeof(char) * 5);
  memcpy(rf_event.to, to, sizeof(char) * 5);

  if (!mesh.write(&rf_event, 'M', sizeof(rf_event))) {
    // If a write fails, check connectivity to the mesh network
    #ifdef DEBUG
    debugOut(70);
    #endif
    _delay_ms(500);
    if ( !mesh.checkConnection() ) {
      //refresh the network address
      #ifdef DEBUG
      debugOut(71);
      #endif
      if (!mesh.renewAddress(1000)) {
        #ifdef DEBUG
        debugOut(72);
        #endif
        //If address renewal fails, reconfigure the radio and restart the mesh
        //This allows recovery from most if not all radio errors
        mesh.begin(RF_CHANNEL, RF24_250KBPS, 1000);
      }
    } else {
      #ifdef DEBUG
      debugOut(80);
      #endif
    }
  } else {
    #ifdef DEBUG
    debugOut(69);
    #endif
  }
}

#ifdef DEBUG
void debugOut(byte value) {
  showDigits(value);
  _delay_ms(500);
}
#endif

/////////////////////////////

uint32_t currentTemperature = 0;

void loadConfig() {
  byte state = EEPROM.read(EEPROM_ADDR_STATE);
  EEPROM_readAnything(EEPROM_ADDR_MAC, mac);
  
  switch (state) {
    case STATE_ACTIVE:
      CURRENT_STATE = STATE_ACTIVE;
      break;
    default:
      CURRENT_STATE = STATE_LINK;
  }
}

void showDigits(byte value) {
  Wire.beginTransmission(0x5);
  Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);

  pinMode(PIN_PULSADOR_A, INPUT);
  pinMode(PIN_PULSADOR_B, INPUT);
  pinMode(PIN_PULSADOR_C, INPUT);
  pinMode(DS18B20_PIN, INPUT);

  // Run only once
  //char m[5] = {'a', 'b', '1', '2', 0};
  //EEPROM_writeAnything(EEPROM_ADDR_MAC, m);
  
  //loadConfig();

  Wire.begin();
  
  //onewire_init(DS18B20_PIN);

  delay(100);
  showDigits(88);

  /*
  attachPCINT(digitalPinToPCINT(PIN_PULSADOR_A), setPulsadorA, RISING);
  attachPCINT(digitalPinToPCINT(PIN_PULSADOR_B), setPulsadorB, RISING);
  attachPCINT(digitalPinToPCINT(PIN_PULSADOR_C), setPulsadorC, RISING);
  */

  //CURRENT_STATE = STATE_LINK;

  //if ((CURRENT_STATE == STATE_ACTIVE) || (CURRENT_STATE == STATE_LINK)) {
    //tempSet = EEPROM.read(EEPROM_ADDR_TEMP_SET);
    //radio.setPALevel(RF24_PA_MAX);
    mesh.begin(RF_CHANNEL, RF24_250KBPS, 1000);
  //}

  showDigits(75);
}

void loop() {
  mesh.update();
  
  publishMsg(EVENT_CURRENT_TEMP, (uint16_t) 15);
  delay(1000);

  /*
  if (CURRENT_STATE == STATE_LINK) {
    while (network.available()) {
      RF24NetworkHeader header;
      event_t payload;
      network.read(header, &payload, sizeof(payload));

      if (strcmp(payload.to, mac) == 0) {
        if (payload.device_type == DEVICE_TYPE_CASAMATIC) {
          if (payload.event_type == CASAMATIC_EVENT_DEVICE_ADDED) {
            EEPROM.update(EEPROM_ADDR_STATE, STATE_ACTIVE);
            CURRENT_STATE = STATE_ACTIVE;
          }  
        }
      } else {
        // showDigits(18);
      }
    }
    
    currentMillis = millis();
    
    if (currentMillis - publishTimer >= 10000) {
      publishTimer = currentMillis;

      showDigits(0);
      
      publishMsg(EVENT_LINK_PRESENT, 0);
    }
  } else if (CURRENT_STATE == STATE_ACTIVE) {
    currentMillis = millis();

    for(byte i = 0; i < 3; i++) {
      if (pressState[i] == PRESS_STATE_STARTED) {
        diffMillis[i] = currentMillis - initPressMillis[i];
    
        if (((PINA >> pulsadores[i])& 1) == 0) {
          //if ((initPressMillis[i] - prevPressMillis[i]) < DETECT_DOUBLE_MS) {
          //  prevPressMillis[i] = 0;
          //  initPressMillis[i] = 0;
          //  showDigits(40 + i);
          //  _delay_ms(300);
          //} else
          if (diffMillis[i] >= DETECT_SHORT_MS && diffMillis[i] < DETECT_LONG_MS) {
            prevPressMillis[i] = initPressMillis[i];
            initPressMillis[i] = 0;

            switch(i) {
              case 0:
                if (CURRENT_MODE == 0) {
                  CURRENT_MODE = 1;
                } else {
                  CURRENT_MODE = 0;
                }
                publishMsg(EVENT_SET_MODE, CURRENT_MODE);
                break;
              case 1:
                tempSet--;
                showDigits(tempSet);
                storeCounter();
                publishMsg(EVENT_SET_TEMP, (uint16_t) tempSet);
                _delay_ms(300);
                break;
              case 2:
                tempSet++;
                showDigits(tempSet);
                storeCounter();
                publishMsg(EVENT_SET_TEMP, (uint16_t) tempSet);
                _delay_ms(300);
            }
          }
          pressState[i] = PRESS_STATE_IDLE;
        } else {
          if (diffMillis[i] > DETECT_LONG_MS) {
            pressState[i] = PRESS_STATE_IDLE;
            initPressMillis[i] = 0;
            prevPressMillis[i] = 0;

            if (i == 0) {
              EEPROM.update(EEPROM_ADDR_STATE, STATE_LINK);
              CURRENT_STATE = STATE_LINK;
            } else {
              showDigits(50 + i);
              _delay_ms(300);
            }
          }
        }
      }
    }

    if (currentMillis - publishTimer >= 5000) {
      publishTimer = currentMillis;

      uint16_t t;
      onewire_reset(); // 1-Wire reset
      onewire_write(ONEWIRE_SKIP_ROM); // to all devices on the bus
      onewire_write(0x44); // send DS18B20 command, "CONVERT T"

      onewire_reset(); // 1-Wire reset
      onewire_write(ONEWIRE_SKIP_ROM); // to all devices on the bus
      onewire_write(0xBE); // send DS18B20 command, "READ SCRATCHPAD"

      t = onewire_read(); // read temperature low byte
      t |= (uint16_t)onewire_read() << 8; // and high byte
      t = ((t >> 4) * 100 + ((t << 12) / 6553) * 10) / 100; // decode temp
  
      showDigits((byte) t);
  
      publishMsg(EVENT_CURRENT_TEMP, (uint16_t) t);
    }
  }*/
}
