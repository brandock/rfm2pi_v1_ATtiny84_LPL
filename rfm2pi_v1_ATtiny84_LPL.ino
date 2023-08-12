/*  Brandock (Brandon Baldock) 2023
 *  This is the 2022 LPL-based RFM69Pi firmware from Open Energy Monitor modified to run on the original ATtiny84-based RFM2Pi. 
 *  https://github.com/openenergymonitor/RFM2Pi/blob/master/docs/rfm12pi_v1.md
 *    
 *  The code is designed to compile with Spence Konde's ATTinyCore for Arduino IDE. The "PIN_PA2" style of designating pins is used.
 *  Thus "clockwise" and "counterclockwise" pin mapping should not be an issue. Either board setting in Tools should work.
 *  
 *  The RFM69_LPL.h library is needed. https://github.com/openenergymonitor/RFM69_LPL
 *  It is a cut-down version of the LowPowerLab RF69 library, modified to work with AVR-DB and ATtiny84.
 *  
 *  Some of the user menu functions have been removed to save space in memory: list config (the terse version), 
 *  set RF band and transmit power, and radio transmit OFF.
 *  
 *  Saving and loading settings to EEPROM can be disabled to save even more space by commenting out "#define useEEPROM".
 *  Without these EEPROM functions the sketch uses 66% of program storage spaces (5432/8192 bytes), 59% of dynamic memory (303/512 bytes). 
 *  With the EEPROM functions the sketch uses 91% of program storage space (7498/8192 bytes), 75% of dynamic memory (387/512 bytes).
 *  The sketch seems to be stable with the EEPROM functions enabled, so they are enabled by default.
 *  
 *  SoftwareSerial is used because the RFM2Pi board routes serial through PA3 and PA7, not the ATTinyCore-defined serial pins.
 *    
 *  ---------------------------------------------------------------------------
 *  rfm2Pi v1 (ATtiny84) by OpenEnergyMonitor Pin Diagram
 *
 *                         +-\/-+
 *                   VCC  1|    |14  GND
 *             (10)  PB0  2|    |13  PA0  ( 0)  
 *       SEL   ( 9)  PB1  3|    |12  PA1  ( 1)  
 *             (11)  PB3  4|    |11  PA2  ( 2)  LED
 *       IRQ   ( 8)  PB2  5|    |10  PA3  ( 3)  RX
 *        TX   ( 7)  PA7  6|    |9   PA4  ( 4)  SCK
 *      MISO   ( 6)  PA6  7|    |8   PA5  ( 5)  MOSI
 *                         +----+
 *  ---------------------------------------------------------------------------
 */

/*
  emonBase rfm69pi LowPowerLabs radio format
  
   ------------------------------------------
  Part of the openenergymonitor.org project

  Authors: Glyn Hudson, Trystan Lea & Robert Wall
  Builds upon JCW JeeLabs RF69 Driver and Arduino

  Licence: GNU GPL V3


Change Log:
*/
const char *firmware_version = {"1.0.0\n\r"};
/*

*********************************************************************************************
*                                                                                           *
*                                  IMPORTANT NOTE                                           *
*                                                                                           *
* When compiling for the RFM2Pi:                                                            * 
*  In IDE, set Board to "Arduino Pro or Pro Mini" & Processor to "ATmega328P (3.3V, 8MHz)"  *
* When compiling for the RFM69Pi:                                                           *
*  In IDE, set Board to "Arduino Uno"                                                       *
*                                                                                           *
* The output file used must NOT be the "with_bootloader" version, else the processor will   *
*  be locked.                                                                               *
*********************************************************************************************
*/

#define useEEPROM           //comment this out to save memory space by disabling load/save settings from EEPROM

#ifdef useEEPROM
  #include <emonEProm.h>
#endif
  
// OEM EPROM library

#include <RFM69_LPL.h>      //https://github.com/openenergymonitor/RFM69_LPL
RFM69 radio;

#define MAXMSG 66                                                      // Max length of o/g message
char outmsg[MAXMSG];                                                   // outgoing message (to emonGLCD etc)
byte outmsgLength;                                                     // length of message: non-zero length triggers transmission

bool verbose = true;

struct {                                                               // Ancilliary information
  byte srcNode = 0;
  byte msgLength = 0;
  signed char rssi = -127;
  bool crc = false;
} rfInfo;

bool rfChanged = false;                                                // marker to re-initialise radio
#define RFTX 0x01                                                      // Control of RFM - transmit enabled

void single_LED_flash(void);
void double_LED_flash(void);
void getCalibration(void);
static void showString (PGM_P s);


//---------------------------- Serial ------------------------------------------------------------------------------------------------
#include <SoftwareSerial.h>
#define rxPin PIN_PA3
#define txPin PIN_PA7

SoftwareSerial mySerial(rxPin, txPin);

#define SERIAL_BAUD 38400

const unsigned long BAUD_RATE   = SERIAL_BAUD;          

//---------------------------- Settings  ------------------------------------------------
struct {
  byte RF_freq = RF69_433MHZ;                           // Frequency of radio module can be RFM_433MHZ, RFM_868MHZ or RFM_915MHZ. 
  byte rfPower = 25;                                    // Power when transmitting
  byte networkGroup = 210;                              // wireless network group, must be the same as emonBase / emonPi and emonGLCD. OEM default is 210
  byte  nodeID = 5;                                     // node ID for this emonBase.
} Settings;


uint16_t eepromSig = 0x0017;                            // oemEProm signature - see oemEEProm Library documentation for details.

//--------------------------- LED ----------------------------------------------------------------------------------------
#define LED         PIN_PA2

const byte LEDpin               = LED;                                   // LED - on when HIGH.  

//-------------------------------------------------------------------------------------------------------------------------------------------

/**************************************************************************************************************************************************
*
* SETUP        Set up & start the radio
*
***************************************************************************************************************************************************/
void setup() 
{  
  // Set I/O pins, print initial message
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin,HIGH);

  mySerial.begin(BAUD_RATE);
  mySerial.print(F("|emonBase_rfm69pi_LPL V")); mySerial.write(firmware_version);
  mySerial.println(F("|OpenEnergyMonitor.org"));
#ifdef useEEPROM
  load_config();                                                      // Load RF config from EEPROM (if any exists)
#endif
  delay(2000);

  radio.setPins(PIN_PB1,PIN_PA5,PIN_PA6,PIN_PA4);

  radio.initialize(RF69_433MHZ,Settings.nodeID,Settings.networkGroup);  
  radio.encrypt("89txbe4p8aik5kt3"); 

  digitalWrite(LEDpin, LOW);
}

/**************************************************************************************************************************************************
*
* LOOP         Poll the radio for incoming data, and the serial input for calibration & outgoing r.f. data 
*
***************************************************************************************************************************************************/

void loop()             
{
  
//-------------------------------------------------------------------------------------------------------------------------------------------
// RF Data handler - inbound ****************************************************************************************************************
//-------------------------------------------------------------------------------------------------------------------------------------------

  if (radio.receiveDone())
  {    
    mySerial.print(F("OK")); 
    mySerial.print(F(" "));
    mySerial.print(radio.SENDERID, DEC);
    mySerial.print(F(" "));
    for (byte i = 0; i < radio.DATALEN; i++) {
      mySerial.print((word)radio.DATA[i]);
      mySerial.print(F(" "));
    }
    mySerial.print(F("("));
    mySerial.print(radio.readRSSI());
    mySerial.print(F(")"));
    mySerial.println();
    double_LED_flash();
  }
  
//-------------------------------------------------------------------------------------------------------------------------------------------
// RF Data handler - outbound ***************************************************************************************************************
//-------------------------------------------------------------------------------------------------------------------------------------------


	if (outmsgLength) {                          // if command 'outmsg' is waiting to be sent then let's send it
    showString(PSTR(" -> "));
    mySerial.print((word) outmsgLength);
    showString(PSTR(" b\n"));
    radio.send(0, (void *)outmsg, outmsgLength);                          //  void RF69<SPI>::send (uint8_t header, const void* ptr
    outmsgLength = 0;
    single_LED_flash();
	}

  
//-------------------------------------------------------------------------------------------------------------------------------------------
// Calibration Data handler *****************************************************************************************************************
//-------------------------------------------------------------------------------------------------------------------------------------------

  if (mySerial.available())                                              // Serial input from RPi for configuration/calibration
  {
    getCalibration();                                                  // If serial input is received from RPi
    double_LED_flash();
    if (rfChanged)
    {
      //rf.init(Settings.nodeID, Settings.networkGroup,                      // Reset the RFM69CW if NodeID, Group or frequency has changed.
      //      Settings.RF_freq == RFM_915MHZ ? 915 : (Settings.RF_freq == RFM_868MHZ ? 868 : 434)); 
            
      radio.initialize(Settings.RF_freq,Settings.nodeID,Settings.networkGroup);  
      radio.encrypt("89txbe4p8aik5kt3"); 
            
      rfChanged = false;
    }
  }

}


// LED flash
void single_LED_flash(void)
{
  digitalWrite(LEDpin, HIGH);  delay(30); digitalWrite(LEDpin, LOW);
}

void double_LED_flash(void)
{
  digitalWrite(LEDpin, HIGH);  delay(20); digitalWrite(LEDpin, LOW); delay(60); 
  digitalWrite(LEDpin, HIGH);  delay(20); digitalWrite(LEDpin, LOW);
}
