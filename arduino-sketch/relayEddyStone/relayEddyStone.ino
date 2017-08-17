
#include <CurieBLE.h>
/**
 * Example demonstrating the use of the relay click and arduino uno click shield from
 * MikroElektronika.
 * 
 * Uplaod this sketch to your arduino/genuino101, complete the circuit and access from the web!
 * Web Bluetoooth goodness!
 * 
 * To be added:
 *    2) BLE control. Done
 *    3) relay object. Done
 *    4) relay interval function. Done
 *    5) RTC "alarm clock" function. Yet to be done . . .
 */

/*PhysicalWeb_EnvironmentalSensor
 * 
 * Based off of Eddystone Beacon Code from: 
 * Copyright (c) 2016 Bradford Needham, North Plains, Oregon, USA
 * @bneedhamia, https://www.needhamia.com
 * Licensed under the Apache 2.0 License, a copy of which
 * should have been included with this software.
 */

/**
 * pins for each relay specified here. 
 * depend on the Arduino click shield specifications, with the relay click board 
 * plugged into slot 1
 */
#define RELAY1 13 
#define RELAY2 10

/**
 * The Relay object will model the relay state and help with control flow in the loop()
 * function.
 */

//const char* MY_URL = "https://maple-slash.glitch.me/";
const char* MY_URL = "https://goo.gl/1VFZxC";

class Relay
{

  int relayID;
  int pinAssignment;
  boolean relayState;
  boolean intervalState;
  long interval;
  boolean intervalToggle;

  public:

  Relay(int id, int pin )
  {
    relayID = id;
    pinAssignment = pin;
    relayState = false;  
    intervalState = false;
    interval = 0;
    intervalToggle = false;
  }

  /*
   * The relay is initialized to the physical pin assigned to it and 
   * this pin is set to low.
   * Not the relay intial states are set in the relay constructor.
   */
  void initialize()
  {
    pinMode( pinAssignment, OUTPUT);
    digitalWrite(pinAssignment,LOW);
  }
  
  int getRelayID() {return relayID;}
  void setRelayId(int id) {relayID = id;}

  int getPinAssignment() {return pinAssignment;}
  void setPinAssignment(int pin) {pinAssignment = pin;}

  boolean getRelayState() {return relayState;}
  void setRelayState(boolean state ) {relayState = state;}

  boolean getIntervalState() {return intervalState;}
  void setIntervalState(boolean state) { intervalState = state;}

  long getInterval(){return interval;}
  void setInterval(long i) {interval = i;}

  boolean getIntervalToggle() {return intervalToggle;}
  void setIntervalToggle(boolean tog){intervalToggle = tog;}

  void toggle() 
  { 
    intervalToggle = !intervalToggle;
    digitalWrite(pinAssignment, intervalToggle);
  }

  void turnOn()
  {
    digitalWrite(pinAssignment,HIGH);
  }

  void turnOff()
  {
    digitalWrite(pinAssignment,LOW);
  }

};

Relay relay1(1, RELAY1);
Relay relay2(2, RELAY2);

// timing parameters
long currentMillis, previousMillisRelay1, previousMillisRelay2;

union
{
  long interval;
  unsigned char bytes[4];  
} relaySettings;

const int8_t TX_POWER_DBM = (-70 + 41);
const uint8_t MAX_URL_FRAME_LENGTH = 1 + 1 + 1 + 17;
const uint8_t FRAME_TYPE_EDDYSTONE_URL = 0x10;
const uint8_t URL_PREFIX_HTTP_WWW_DOT = 0x00;
const uint8_t URL_PREFIX_HTTPS_WWW_DOT = 0x01;
const uint8_t URL_PREFIX_HTTP_COLON_SLASH_SLASH = 0x02;
const uint8_t URL_PREFIX_HTTPS_COLON_SLASH_SLASH = 0x03;

uint8_t urlFrame[MAX_URL_FRAME_LENGTH];
uint8_t urlFrameLength;
bool setupSucceeded;

/* establish BLE service & characteristics */

BLEPeripheral blePeripheral;
BLEService eddyService("FEAA");
BLEService relayService("917649A0-D98E-11E5-9EEC-0002A5D5C51B");
BLECharacteristic relayCharacteristic("917649A1-D98E-11E5-9EEC-0002A5D5C51B", BLEWrite, 5);
// BLEUnsignedCharCharacteristic relayCharacteristic("917649A1-D98E-11E5-9EEC-0002A5D5C51B", BLERead | BLEWrite);
BLEDescriptor relayDescriptor("2902","relay");

void setup() 
{
  setupSucceeded = false;
  // initiate serial communications for debugging
  Serial.begin(115200);
  Serial.println("click relay example");
 /** 
   *  BLE initilizations
   */

  if (!initEddystoneUrlFrame(TX_POWER_DBM, MY_URL)) {
     Serial.println("did not work");
    return; // don't start advertising if the URL won't work.
  }

  blePeripheral.setAdvertisedServiceUuid(eddyService.uuid());
  blePeripheral.setAdvertisedServiceData(eddyService.uuid(), urlFrame, urlFrameLength);
 

  blePeripheral.setDeviceName("relays");
  blePeripheral.setAdvertisedServiceUuid(relayService.uuid());
  blePeripheral.addAttribute(relayService);
  blePeripheral.addAttribute(relayCharacteristic);
  blePeripheral.addAttribute(relayDescriptor);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  relayCharacteristic.setEventHandler(BLEWritten, relayCharacteristicWritten);


 
  // All BLE characteristics should be initialized to a starting value prior
  // using them.

  const unsigned char relayInitializer[5] = {0,0,0,0,0};
  relayCharacteristic.setValue(relayInitializer,5);
  
  blePeripheral.begin();
  
  // Set relays to initial states
  relay1.initialize();
  relay2.initialize();

  // intialize timing paramters
  currentMillis = millis();
  previousMillisRelay1 = millis();
  previousMillisRelay2 = millis();

  setupSucceeded = true;
}

void loop() 
{
  //Serial.println(setupSucceeded);
  /**
   * The main loop() checks for the relays being set to intervals and
   * toggles then is so and the interval is exceeded.
   */
  blePeripheral.poll();
  currentMillis = millis();
  
  if (relay1.getIntervalState()) 
  {
    if ((currentMillis - previousMillisRelay1) > relay1.getInterval())
    {
      relay1.toggle();
      previousMillisRelay1 = currentMillis;
      Serial.println("toggle1");
    }
  }

  if (relay2.getIntervalState())
  {  
    if ((currentMillis - previousMillisRelay2) > relay2.getInterval())
    {
      relay2.toggle();
      previousMillisRelay2 = currentMillis;
      Serial.println("toggle2");
    }
  }
  
}


void blePeripheralConnectHandler(BLECentral& central) 
{
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) 
{
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void relayCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic)
{
  /**
   * This function is called whenever the user changes the relayCharacteristic on the hybrid
   * web app. It is used to chage the state of the relay object, these changes are then
   * checked for in the loop. 
   * Why? Well if we wished to simply turn the relays off and on, this approach is unnecessary. 
   * we could just simply turn the relays off and on in the swtch/case statement. However
   * we would like to add other functionality to the relays, such as interval on/off timing.
   * Since we have 2 relays we can;t use the CurieTImer library, it has only one timer and we may
   * wish to have different timing intervals for each relay. Addtionally this approach will allow
   * us to more easily extend this app to many relays, which is is more realisitc for
   * a home automation system.
   */
  const unsigned char* relayData = relayCharacteristic.value();
  
  relaySettings.bytes[0] = relayData[4];
  relaySettings.bytes[1] = relayData[3];
  relaySettings.bytes[2] = relayData[2];
  relaySettings.bytes[3] = relayData[1];

  // convert interval byte array to long by casting;
  long interval = (long)relaySettings.interval;
  switch(relayData[0])
  {
    case 0:
      // 0 value, do nothing
      Serial.print(relayData[0]);
      Serial.print(" written ");Serial.print("interval: ");Serial.println(interval);
      break;
    case 1:
      // relay 1 selected
      Serial.print("relay1: ");Serial.print(relayData[0]);Serial.print(" interval: ");Serial.println(interval);
      if (interval != 0) 
      {
        relay1.setIntervalState(true);
        relay1.setRelayState(true);
        relay1.setInterval(interval);
      } else {
        if (relay1.getRelayState()) 
        { 
          relay1.setRelayState(false); 
          relay1.turnOff(); 
          relay1.setIntervalState(false);
          relay1.setInterval(0); 
        }
        else {
          relay1.setRelayState(true); 
          relay1.turnOn();
          relay1.setIntervalState(false);
          relay1.setInterval(0);
        }       
      }
      break;
    case 2:
      // relay 2 selected
      Serial.print("relay2: ");Serial.print(relayData[0]);Serial.print(" interval: ");Serial.println(interval);
      if (interval != 0) 
      {
        relay2.setIntervalState(true);
        relay2.setRelayState(true);
        relay2.setInterval(interval);
      } else {
        if (relay2.getRelayState()) 
        { 
          relay2.setRelayState(false); 
          relay2.turnOff(); 
          relay2.setIntervalState(false);
          relay2.setInterval(0); 
        }
        else {
          relay2.setRelayState(true); 
          relay2.turnOn();
          relay2.setIntervalState(false);
          relay2.setInterval(0);
        }       
      }
      break;     
  }
}  

boolean initEddystoneUrlFrame(int8_t txPower, const char* url) {
  urlFrameLength = 0;

  // The frame starts with a type byte, then power byte.
  urlFrame[urlFrameLength++] = FRAME_TYPE_EDDYSTONE_URL;
  urlFrame[urlFrameLength++] = (uint8_t) txPower;

  if (url == 0 || url[0] == '\0') {
    return false;   // empty URL
  }

  const char *pNext = url;

  /*
   * the next byte of the frame is an URL prefix code.
   * See URL_PREFIX_* above.
   */
  
  if (strncmp("http", pNext, 4) != 0) {
    return false;  // doesn't start with HTTP or HTTPS.
  }
  pNext += 4;
  
  bool isHttps = false; // that is, HTTP
  if (*pNext == 's') {
    pNext++;
    isHttps = true;
  }

  if (strncmp("://", pNext, 3) != 0) {
    return false; // malformed URL
  }
  pNext += 3;

  urlFrame[urlFrameLength] = URL_PREFIX_HTTP_COLON_SLASH_SLASH;
  if (isHttps) {
    urlFrame[urlFrameLength] = URL_PREFIX_HTTPS_COLON_SLASH_SLASH;
  }

  if (strncmp("www.", pNext, 4) == 0) {
    pNext += 4;

    urlFrame[urlFrameLength] = URL_PREFIX_HTTP_WWW_DOT;
    if (isHttps) {
      urlFrame[urlFrameLength] = URL_PREFIX_HTTPS_WWW_DOT;
    }
  }
  
  urlFrameLength++;

  // Encode the URL.

  while (urlFrameLength < MAX_URL_FRAME_LENGTH && *pNext != '\0') {
    if (strncmp(".com/", pNext, 5) == 0) {
      pNext += 5;
      urlFrame[urlFrameLength++] = 0x00;
    } else if (strncmp(".org/", pNext, 5) == 0) {
      pNext += 5;
      urlFrame[urlFrameLength++] = 0x01;
    } else if (strncmp(".edu/", pNext, 5) == 0) {
      pNext += 5;
      urlFrame[urlFrameLength++] = 0x02;
    } else if (strncmp(".net/", pNext, 5) == 0) {
      pNext += 5;
      urlFrame[urlFrameLength++] = 0x03;
    } else if (strncmp(".info/", pNext, 6) == 0) {
      pNext += 6;
      urlFrame[urlFrameLength++] = 0x04;
    } else if (strncmp(".biz/", pNext, 5) == 0) {
      pNext += 5;
      urlFrame[urlFrameLength++] = 0x05;
    } else if (strncmp(".gov/", pNext, 5) == 0) {
      pNext += 5;
      urlFrame[urlFrameLength++] = 0x06;
    } else if (strncmp(".com", pNext, 4) == 0) {
      pNext += 4;
      urlFrame[urlFrameLength++] = 0x07;
    } else if (strncmp(".org", pNext, 4) == 0) {
      pNext += 4;
      urlFrame[urlFrameLength++] = 0x08;
    } else if (strncmp(".edu", pNext, 4) == 0) {
      pNext += 4;
      urlFrame[urlFrameLength++] = 0x09;
    } else if (strncmp(".net", pNext, 4) == 0) {
      pNext += 4;
      urlFrame[urlFrameLength++] = 0x0A;
    } else if (strncmp(".info", pNext, 5) == 0) {
      pNext += 5;
      urlFrame[urlFrameLength++] = 0x0B;
    } else if (strncmp(".biz", pNext, 4) == 0) {
      pNext += 4;
      urlFrame[urlFrameLength++] = 0x0C;
    } else if (strncmp(".gov", pNext, 4) == 0) {
      pNext += 4;
      urlFrame[urlFrameLength++] = 0x0D;
    } else {
      // It's not special.  Just copy the character
      urlFrame[urlFrameLength++] = (uint8_t) *pNext++;
    }
  }

  return true;
}

