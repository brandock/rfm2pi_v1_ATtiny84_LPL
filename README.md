# rfm2pi_v1_ATtiny84_LPL
LowPowerLab RFM69_LPL firmware for the original OpenEnergyMonitor RFM2Pi with ATtiny84.

# Background
The RFM12Pi GPIO expansion board enables a Raspberry Pi (emonBase) to receive data via HopeRF RFM12B or RFM69CW wireless (433/868/915MHz) from other OpenEnergyMonitor modules such an emonTx energy and temperature monitoring node.
https://github.com/openenergymonitor/RFM2Pi/blob/master/docs/rfm12pi_v1.md
The original RFM12Pi board was a hand-solderable kit with ATtiny84 microcontroller. The firmware used the Jeelib radio library and was compiled with the arduino-tiny core.
Flash forward to 2023 and OpenEnergyMonitor’s new continuous monitoring modules use the RF69 library from Felix Rusu of LowPowerLab. Spence Konde has developed the modern DxCore and ATTinyCore board managers for Arduino IDE. DxCore is used to compile the firmware for the latest AVR-DB-based emonTx v4.
The repository contains new firmware for the original ATtiny84-based RFM2Pi with RFM69CW radios, using the newer RFM69 radio format and ATTinyCore. 

# Notes
The code is designed to compile with Spence Konde's ATTinyCore for Arduino IDE. The "PIN_PA2" style of designating pins is used. Thus "clockwise" and "counterclockwise" pin mapping should not be an issue. Either board setting in Tools should work.
 
The RFM69_LPL.h library is needed. 
https://github.com/openenergymonitor/RFM69_LPL
It is a cut-down version of the LowPowerLab RF69 library, modified to work with AVR-DB and ATtiny84.
 
Some of the user menu functions have been removed to save space in memory: list config (the terse version), set RF band and transmit power, and radio transmit OFF.

Saving and loading user-settings to EEPROM can be disabled to save even more space by commenting out "#define useEEPROM".
Without these EEPROM functions the sketch uses 66% of program storage spaces (5432/8192 bytes), 59% of dynamic memory (303/512 bytes). 
With the EEPROM functions the sketch uses 91% of program storage space (7498/8192 bytes), 75% of dynamic memory (387/512 bytes).

The sketch seems to be stable with the EEPROM functions enabled, so they are enabled by default.
 
SoftwareSerial is used because the RFM2Pi board routes serial through PA3 and PA7, not the ATTinyCore-defined serial pins.
    
*  ---------------------------------------------------------------------------
*  rfm2Pi v1 (ATtiny84) by OpenEnergyMonitor Pin Diagram
*
*                         +-\/-+
*                   VCC  1|    |14  GND
*                   PB0  2|    |13  PA0    
*       SEL         PB1  3|    |12  PA1     
*                   PB3  4|    |11  PA2        LED
*       IRQ         PB2  5|    |10  PA3        RX
*        TX         PA7  6|    |9   PA4        SCK
*      MISO         PA6  7|    |8   PA5        MOSI
*                         +----+
*  ---------------------------------------------------------------------------

