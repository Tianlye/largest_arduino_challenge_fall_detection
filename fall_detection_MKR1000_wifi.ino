/*
   Firmata is a generic protocol for communicating with microcontrollers
   from software on a host computer. It is intended to work with
   any host computer software package.

   To download a host software package, please clink on the following link
   to open the download page in your default browser.

   http://firmata.org/wiki/Download
*/

/* Supports as many analog inputs and analog PWM outputs as possible.

   This example code is in the public domain.
*/
#include <Firmata.h>
#define SERIAL_DEBUG
#include "utility/firmataDebug.h"
// follow the instructions in wifiConfig.h to configure your particular hardware
#include "wifiConfig.h"

//for push notification
char serverName[] = "api.pushingbox.com";
char DEVID1[] = "";        
WiFiClient client;
boolean lastConnected = false;

#define WIFI_MAX_CONN_ATTEMPTS      3
byte analogPin = 0;
int wifiConnectionAttemptCounter = 0;
int wifiStatus = WL_IDLE_STATUS;
boolean isResetting = false;


/* analog inputs */
int analogInputsToReport = 0;      // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else


float x_init, y_init, z_init, x_val, y_val, z_val;
float  x, y, z;
int windowsSendHelpPin = 3;
int wifiLEDPin = 9;
int helpButtonPin = 4;
int resetButtonPin = 5;
int buzzpin = 8;
int warningLEDPin = 7;
int resetButtonState = 0;
int windowsTriggerPin = 6;

int minAxisValue = 426;
int maxAxisValue = 627;

float maxChange = 1.85;
int calls = 0;
int firstRun = 0;
// Set the number of Max Calls to one since the user only needs to be notified once that the object has fallen
int maxCalls = 1;


void analogWriteCallback(byte pin, int value)
{
  if (IS_PIN_PWM(pin)) {
    pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
    analogWrite(PIN_TO_PWM(pin), value);
  }
}

void setup()
{
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  pinMode(warningLEDPin, OUTPUT);
  pinMode(resetButtonPin, INPUT);
  pinMode(windowsTriggerPin, OUTPUT);
  digitalWrite(windowsTriggerPin, LOW);
  pinMode(windowsTriggerPin, INPUT);
  /*
     WIFI SETUP
  */
  DEBUG_BEGIN(9600);

  /*
     This statement will clarify how a connection is being made
  */
  DEBUG_PRINT( "StandardFirmataWiFi will attempt a WiFi connection " );
  DEBUG_PRINTLN( "using the WiFi 101 library." );

  DEBUG_PRINTLN( "IP will be requested from DHCP ..." );
  /*
       Configure WiFi security
  */
#if defined(WIFI_WEP_SECURITY)
  while (wifiStatus != WL_CONNECTED) {
    DEBUG_PRINT( "Attempting to connect to WEP SSID: " );
    DEBUG_PRINTLN(ssid);
    wifiStatus = stream.begin( ssid, wep_index, wep_key, SERVER_PORT );
    delay(5000); // TODO - determine minimum delay
    if (++wifiConnectionAttemptCounter > WIFI_MAX_CONN_ATTEMPTS) break;
  }

#elif defined(WIFI_WPA_SECURITY)
  while (wifiStatus != WL_CONNECTED) {
    DEBUG_PRINT( "Attempting to connect to WPA SSID: " );
    DEBUG_PRINTLN(ssid);
    wifiStatus = stream.begin(ssid, wpa_passphrase, SERVER_PORT);
    delay(2000); // TODO - determine minimum delay
    if (++wifiConnectionAttemptCounter > WIFI_MAX_CONN_ATTEMPTS) break;
  }

#else                          //OPEN network
  while (wifiStatus != WL_CONNECTED) {
    DEBUG_PRINTLN( "Attempting to connect to open SSID: " );
    DEBUG_PRINTLN(ssid);
    wifiStatus = stream.begin(ssid, SERVER_PORT);
    delay(5000); // TODO - determine minimum delay
    if (++wifiConnectionAttemptCounter > WIFI_MAX_CONN_ATTEMPTS) break;
  }
#endif //defined(WIFI_WEP_SECURITY)

  DEBUG_PRINTLN( "WiFi setup done" );
  pinMode(wifiLEDPin, OUTPUT);
  digitalWrite(wifiLEDPin, HIGH);

  printWifiStatus();

  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  //Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  //Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
  // start up Network Firmata:
  Firmata.begin(stream);
  systemResetCallback();  // reset to default config
}

void loop()
{
  if ( WiFi.status() == WL_CONNECTED )
  {

    pinMode(wifiLEDPin, OUTPUT);
    digitalWrite(wifiLEDPin, HIGH);

  }
  pinMode(warningLEDPin, INPUT);
  pinMode(resetButtonPin, INPUT);
  pinMode(helpButtonPin, INPUT);
  x = analogRead(0);       // read analog input pin 0
  y = analogRead(1);       // read analog input pin 1
  z = analogRead(2);       // read analog input pin 1

  //Serial.print(x, DEC);    // print the acceleration in the X axis
  x = map(x, minAxisValue, maxAxisValue, -1000, +1000) / 1000.0;
  y = map(y, minAxisValue, maxAxisValue, -1000, +1000) / 1000.0;
  z = map(z, minAxisValue, maxAxisValue, -1000, +1000) / 1000.0;

  x_init = x_val;
  x_val = x;
  y_init = y_val;
  y_val = y;
  z_init = z_val;

  // wait 100ms for next reading
  if (!firstRun) {

    //trigger the choreo if the accelerometer values have changed more than the max allowed amount
    if ((absFloat(x_val - x_init) >= maxChange) || (absFloat(y_val - y_init) >= maxChange) || (absFloat(z_val - z_init) >= maxChange)) {
      
      if (calls < maxCalls) {
        
        //Serial.println("\nTriggered! Sending alert...");
        //sendToPushingBox(DEVID1);
        calls++;
      }
      else {
        //Serial.println("\nTriggered! Already notified user of the fall so no text sent!");
      }
      
    }
    if (digitalRead(helpButtonPin) == HIGH) {
      calls = 1;
    }
    //need to notify ppl
    if (calls >= 1) {

      blinkWarningLight(1000);
      buzz(2000);
      //blinkWarningLight(1000);
      pinMode(windowsSendHelpPin, OUTPUT);
      digitalWrite(windowsSendHelpPin, HIGH);
      Firmata.sendString("FALLEN!");
      if (calls <= maxCalls) {
        Serial.println("\nTriggered! Sending alert...");
        sendToPushingBox(DEVID1);
        calls++;
      }
      else {
        Serial.println("\nTriggered! Already notified user of the fall so no text sent!");
      }
      //pinMode(windowsSendHelpPin,INPUT);
    } else {
      pinMode(warningLEDPin, OUTPUT);
      digitalWrite(warningLEDPin, LOW);
      pinMode(windowsSendHelpPin, OUTPUT);
      digitalWrite(windowsSendHelpPin, LOW);
      Firmata.sendString("NORMAL");
      //pinMode(windowsSendHelpPin,INPUT);
    }

    delay(250);
  }
  else {
    //if this is the first time through, set the initial values and change the firstRun boolean value
    x_init = x_val;
    y_init = y_val;
    z_init = z_val;
    firstRun = 0;
  }


  resetButtonState = digitalRead(resetButtonPin);

  //button pressed seems to be low when pressed
  if (resetButtonState == HIGH) {
    calls = 0;
    //Serial.println("Button Pressed:HIGH");
    buzz_reset(500);
    delay(1000);
  } else {
    //Serial.println("Button Pressed:LOW");

  }
  delay(100);

  while (Firmata.available()) {
    Firmata.processInput();
  }
  // do one analogRead per loop, so if PC is sending a lot of
  // analog write messages, we will only delay 1 analogRead
  //think we just need the 3 we are interested in. 
  
  Firmata.sendAnalog(analogPin, analogRead(analogPin));
  analogPin = analogPin + 1;
  //uncomment to send for all
  //if (analogPin >= TOTAL_ANALOG_PINS) analogPin = 0;
  if (analogPin >= 2) analogPin = 0;
  pinMode(windowsTriggerPin, INPUT);

  if (digitalRead(windowsTriggerPin) == HIGH) {
    DEBUG_PRINT( "TONE AS WINDOWS TRIGGER PIN is high" );
    //tone(buzzpin, 4000, 500);
    calls = 2;
  }

  //DEBUG part
  // this write the respons from PushingBox Server.
  // You should see a "200 OK"
  if (client.available()) {
    char c = client.read();
    DEBUG_PRINT(c);
  }
  // if there's no net connection, but there was one last time
  // through the loop, then stop the client:
  if (!client.connected() && lastConnected) {
    DEBUG_PRINT(" ");
    DEBUG_PRINTLN("disconnecting.");
    client.stop();
  }
  lastConnected = client.connected();
}



void printWifiStatus() {
#if defined(ARDUINO_WIFI_SHIELD) || defined(WIFI_101)
  if ( WiFi.status() != WL_CONNECTED )
  {

    DEBUG_PRINT( "WiFi connection failed. Status value: " );
    DEBUG_PRINTLN( WiFi.status() );
  }
  else
#endif    //defined(ARDUINO_WIFI_SHIELD) || defined(WIFI_101)
  {
    // print the SSID of the network you're attached to:
    DEBUG_PRINT( "SSID: " );

#if defined(ARDUINO_WIFI_SHIELD) || defined(WIFI_101)
    DEBUG_PRINTLN( WiFi.SSID() );
#endif    //defined(ARDUINO_WIFI_SHIELD) || defined(WIFI_101)

    // print your WiFi shield's IP address:
    DEBUG_PRINT( "IP Address: " );

#if defined(ARDUINO_WIFI_SHIELD) || defined(WIFI_101)
    IPAddress ip = WiFi.localIP();
    DEBUG_PRINTLN( ip );
#endif    //defined(ARDUINO_WIFI_SHIELD) || defined(WIFI_101)

    // print the received signal strength:
    DEBUG_PRINT( "signal strength (RSSI): " );

#if defined(ARDUINO_WIFI_SHIELD) || defined(WIFI_101)
    long rssi = WiFi.RSSI();
    DEBUG_PRINT( rssi );
#endif    //defined(ARDUINO_WIFI_SHIELD) || defined(WIFI_101)

    DEBUG_PRINTLN( " dBm" );
    //digitalWrite(6,HIGH);
  }

  stream.maintain();

}


void systemResetCallback()
{
  isResetting = true;

  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif


  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }

  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;


  /* send digital inputs to set the initial state on the host computer,
     since once in the loop(), this firmware will only send on change */
  /*
    TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
    for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
    }
  */
  isResetting = false;
}

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte stopTX;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;

  switch (command) {

    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_PULLUP);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(8); // 8 = 8-bit resolution
        }
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write(PIN_MODE_SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(PIN_MODE_I2C);
          Firmata.write(1);  // TODO: could assign a number to map to SCL or SDA
        }
#ifdef FIRMATA_SERIAL_FEATURE
        serialFeature.handleCapability(pin);
#endif
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
          if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
          if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;

    case SERIAL_MESSAGE:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handleSysex(command, argc, argv);
#endif
      break;
  }
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
   two bit-arrays that track Digital I/O and PWM status
*/
void setPinModeCallback(byte pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;


  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
          // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
        // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        Firmata.setPinMode(pin, INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
      break;

    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
*/
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{

  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void digitalWriteCallback(byte port, int value)
{
  DEBUG_PRINTLN( "digitalWriteCallback " );
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        //DEBUG_PRINTLN( "Pin is digital pin" );
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (Firmata.getPinMode(pin) == OUTPUT || Firmata.getPinMode(pin) == INPUT) {
          pinValue = ((byte)value & mask) ? 1 : 0;
          DEBUG_PRINT( "Pin ");
          DEBUG_PRINT( pin);
          DEBUG_PRINT( " VALUE[");
          DEBUG_PRINT( pinValue);
          DEBUG_PRINTLN("]");
          if (Firmata.getPinMode(pin) == OUTPUT) {

            pinWriteMask |= mask;
          } else if (Firmata.getPinMode(pin) == INPUT && pinValue == 1 && Firmata.getPinState(pin) != 1) {
            // only handle INPUT here for backwards compatibility
#if ARDUINO > 100
            pinMode(pin, INPUT_PULLUP);
#else
            // only write to the INPUT pin to enable pullups if Arduino v1.0.0 or earlier
            pinWriteMask |= mask;
#endif
          }
          Firmata.setPinState(pin, pinValue);
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}

void buzz_reset(unsigned char time)
{
  tone(buzzpin, 2000, time);
}
void buzz(unsigned char time)
{
  tone(buzzpin, 4000, time);
}

float absFloat(float x) {
  return (x) > 0 ? (x) : -(x);
}
void blinkWarningLight(unsigned char time) {
  //DEBUG_PRINTLN("BLINK");
  pinMode(warningLEDPin, OUTPUT);
  digitalWrite(warningLEDPin, HIGH);
  delay(time);
  digitalWrite(warningLEDPin, LOW);

}



void reportDigitalCallback(byte port, int value)
{
  DEBUG_PRINTLN("reportDigitalCallback");
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}
void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}
void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (Firmata.getPinMode(pin) == OUTPUT) {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}
//Function for sending the request to PushingBox
void sendToPushingBox(char devid[]) {
  client.stop();

  DEBUG_PRINTLN("connecting...");

  if (client.connect(serverName, 80)) {
    DEBUG_PRINTLN("connected");

    DEBUG_PRINTLN("sending push notification request");
    client.print("GET /pushingbox?devid=");
    client.print(devid);
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(serverName);
    client.println("User-Agent: Arduino");
    client.println();
    DEBUG_PRINTLN("done sending push notification ");
  }
  else {
    DEBUG_PRINTLN("connection failed");
  }
}

