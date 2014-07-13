+-----------------------------------------------------------------------------+
|                          SX12xx drivers V2.0.B2                             |
+-----------------------------------------------------------------------------+

This package currently supports the following radio drivers:
    - SX1232
    - SX1272
        - FSK modem
        - LoRa modem ( Application default )
    - SX1276
        - FSK modem
        - LoRa modem ( Application default )
        
1. Introduction
-------------------------------------------------------------------------------

The SX12xx drivers are included in a project made for the SX12xxEiger platform.

The SX12xxEiger platform is based on the OPEN4 platform from RAISONANCE.
The platform can use a STM32F407 or STM32F103 microcontroller ( ARM Cortex-M
based microcontrollers ).
Semtech has done an extension board for the OPEN4 platform which enables the
control of the SX12xx RF chip SM modules.

The project included is a RIDE 7 project. RIDE is an IDE made by RAISONANCE.
The project is also compatible with KEIL toolchain from ARM. The user can 
select either of the project depending of its toolchain.

The project contains the low level drivers needed to control the extension
board peripherals as well as the SX12xx RF chip drivers. The demonstration
application is a simple PING-PONG.

By default the PING-PONG app acts as Master. If another Master is already
running the application will switch automatically to Slave and the PING-PONG
starts.


2. Radio drivers description
-------------------------------------------------------------------------------

The radio interface for the application is manged by the src\radio\radio.c file.
The RadioDriverInit function must be updated with selected radio
( SX1232, SX1272, SX1276 ) functions.

The drivers have been made to operate on several hardware platforms:
- the SX12xxEiger platform
- the SX12000DVK platform
- the "Bleeper"

The radio selection is done by commenting/uncomenting the USE_SX12XX_RADIO
definitions in src\platform\platform.h file

Example:
In order to select SX1272 with LoRa modem one must modify
src\platform\platform.h file as follows:

/*!
 * Radio choice. Please uncomment the wanted radio and comment the others
 */
//#define USE_SX1232_RADIO
#define USE_SX1272_RADIO
//#define USE_SX1276_RADIO


For SX1272 radio device one must also choose the modem to be used.
In order to select the LoRa modem driver one must modify src\radio\radio.h as
follows:

/*!
 * SX1272 and SX1276 General parameters definition
 */
#define LORA                                     1         // [0: OFF, 1: ON]

The drivers can also



2.1 Specific radio drivers description
-------------------------------------------------------------------------------

The specific radio driver is split in several parts in order to ease the
portability.


2.1.1 SX1232 driver
-------------------------------------------------------------------------------

The SX1232 driver is split in 2 parts

1. Generic SX1232 driver.
  ( src\radio\SX1232.c and src\radio\SX1232-Misc.c )
2. SX1272 HAL ( Hardware Abstraction Layer ).
  ( src\platform\sx12xxEiger\SX1272-Hal.c )

1. The generic SX1232 driver implements at least the functions required by 
   the RadioDriver structure defined in src\radio\radio.h file
 
   In order to change generic settings one must modify the following
   parameters in file src\radio\SX1232.c
   The default parameters are all set into a structure which is fairly easy
   to update for any system.	

   tFskSettings FskSettings = 
   {
      870000000,      // RFFrequency
      9600,           // Bitrate
      50000,          // Fdev
      20,             // Power
      100000,         // RxBw
      150000,         // RxBwAfc
      true,           // CrcOn
      true,           // AfcOn    
      255             // PayloadLength
   };




   REMARK: All other parameters can be changed by modifying the SX1232Init
          function located in src\radio\SX1232.c file

2. The HAL makes the SX1232 driver platform independent.

    One must modify each function inside this file
    ( src\platform\sx12xxEiger\SX1232-Hal.c ) according to the platform used.


2.1.2 SX1272 driver version 2.0.B2 
-------------------------------------------------------------------------------

The SX1272 driver is split in 4 parts

1. Generic SX1272 driver.
  ( src\radio\SX1272.c )
2. SX1272 FSK modem driver.
  ( src\radio\SX1272-Fsk.c and src\radio\SX1272-FskMisc.c )
3. SX1272 LoRa modem driver.
  ( src\radio\SX1272-LoRa.c and src\radio\SX1272-LoRaMisc.c )
4. SX1272 HAL ( Hardware Abstraction Layer ).
  ( src\platform\sx12xxEiger\SX1272-Hal.c )

1. The generic SX1272 driver implements at least the functions required by 
   the RadioDriver structure defined in src\radio\radio.h file. It offers also
   the same interface for the FSK or the LoRa modem.
 
   In order to choose which modem to use one must modify the src\radio\radio.h
    file as follows:
 
   - For FSK modem
        #define LORA                                        0

   - For LoRa modem
        #define LORA                                        1

2. The FSK modem driver handles the SX1272 as a FSK modem

   In order to change generic FSK modem settings one must modify the following
   parameters in file src\radio\SX1272-Fsk.c

   tFskSettings FskSettings = 
   {
      870000000,      // RFFrequency
      9600,           // Bitrate
      50000,          // Fdev
      20,             // Power
      100000,         // RxBw
      150000,         // RxBwAfc
      true,           // CrcOn
      true,           // AfcOn    
      255             // PayloadLength (set payload size to the maximum for 
                      // variable mode, else set the exact payload length)
   };


   REMARK: All other parameters can be changed by modifying the SX1272FskInit
          function located in src\radio\SX1272-Fsk.c file

3. The LoRa modem driver handles the SX1272 as a LoRa modem

   In order to change generic LoRa modem settings one must modify the following
   parameters in file src\radio\SX1272-LoRa.c


   tLoRaSettings LoRaSettings =
   {
      870000000,      // RFFrequency
      20,             // Power  
      2,              // SignalBw [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 
                      // 3: Reserved] 
      7,              // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 
                      // 10: 1024, 11: 2048, 12: 4096  chips]
      2,              // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
      true,           // CrcOn
      false,          // ImplicitHeaderOn
      1,              // RxSingleOn
      0,              // FreqHopOn
      4,              // HopPeriod
      100,            // TxPacketTimeout
      100,            // RxPacketTimeout
      4               // PayloadLength
   };



4. The HAL makes the SX1272 driver platform independent.

    One must modify each function inside this file
    ( src\platform\sx12xxEiger\SX1272-Hal.c ) according to the platform used.


2.1.3 SX1276 driver version V2.0.B2
-------------------------------------------------------------------------------

The SX1276 drivers are very similar to the SX1272 drivers. In fact, most of the 
drivers are identical except for the new feature supported by the SX1276 (new 
frequency band, new Bandwidth of operation in LoRa, ...) and some registers
addresses which are different.

The SX1276 driver is split in 4 parts

1. Generic SX1276 driver.
  ( src\radio\SX1276.c )
2. SX1272 FSK modem driver.
  ( src\radio\SX1276-Fsk.c and src\radio\SX1276-FskMisc.c )
3. SX1272 LoRa modem driver.
  ( src\radio\SX1276-LoRa.c and src\radio\SX1276-LoRaMisc.c )
4. SX1276 HAL ( Hardware Abstraction Layer ).
  ( src\platform\sx12xxEiger\SX1276-Hal.c )

1. The generic SX1276 driver implements at least the functions required by 
   the RadioDriver structure defined in src\radio\radio.h file. It offers also
   the same interface for the FSK or the LoRa modem.
 
   In order to choose which modem to use one must modify the src\radio\radio.h
    file as follows:
 
   - For FSK modem
        #define LORA                                        0

   - For LoRa modem
        #define LORA                                        1

2. The FSK modem driver handles the SX1276 as a FSK modem

   In order to change generic FSK modem settings one must modify the following
   parameters in file src\radio\SX1276-Fsk.c

   tFskSettings FskSettings = 
   {
      870000000,      // RFFrequency
      9600,           // Bitrate
      50000,          // Fdev
      20,             // Power
      100000,         // RxBw
      150000,         // RxBwAfc
      true,           // CrcOn
      true,           // AfcOn    
      255             // PayloadLength (set payload size to the maximum for 
                      // variable mode, else set the exact payload length)
   };


   REMARK: All other parameters can be changed by modifying the SX1272FskInit
          function located in src\radio\SX1276-Fsk.c file

3. The LoRa modem driver handles the SX1276 as a LoRa modem

   In order to change generic LoRa modem settings one must modify the following
   parameters in file src\radio\SX1276-LoRa.c


   tLoRaSettings LoRaSettings =
   {
      870000000,      // RFFrequency
      20,             // Power  
      8,              // SignalBw [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved] 
      7,              // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 
                      // 10: 1024, 11: 2048, 12: 4096  chips]
      2,              // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
      true,           // CrcOn
      false,          // ImplicitHeaderOn
      1,              // RxSingleOn
      0,              // FreqHopOn
      4,              // HopPeriod
      100,            // TxPacketTimeout
      100,            // RxPacketTimeout
      4               // PayloadLength
   };



4. The HAL makes the SX1276 driver platform independent.

    One must modify each function inside this file
    ( src\platform\sx12xxEiger\SX1272-Hal.c ) according to the platform used.

3. How to use the driver:
-------------------------------------------------------------------------------


This driver has been tested for high speed transmission (up to 100kbps in FSK) 
and long payloads (up to 255 bytes in FSK or LoRa). To set a transmission /
reception, it is necessary to:

   - Change the payload length

The payload length for the system is defined with the parameter BUFFER_SIZE 
located in main.c

#define BUFFER_SIZE                                 128 // Define the payload 
                                                        // size here

The payload length can be configured from 1 up to 255


   - Change the RF Parameters

Depending on which mode you are operating (FSK or LoRa), you should modify the 
parameters in their dedicated structure, either the structure LoRaSettings or 
the structure FskSettings. Care must be taken when changing the parametes so 
that there are no conflict between the parameters. For example, in LoRa, the 
Spreading Factor SF6 only operates in Implicit Header mode.
In case of doubts, please refer to the latest datasheet for your device.

The driver is organised to perform a PING PONG between the transmitter and the 
receiver. Both the transmitter and the receiver will start as masters and will 
therefore start sending packet and waiting to receive one. As soon as one of the
device receive a packet PING, it will set itself as a slave and will then send 
a PONG. Afterward, the master will always send a PING and the slave will answer
with a PONG in an infinite loop. This organisation should provide a good example
of transmission and reception using the devices.


The code is compiled and ready to run on the F103 ST module of the Eiger 
evaluation platform. If your Eiger board is fitted with a F407 ST module, it is 
necessary to change the compilation option within Ride 7. To do so, in Ride 7, 
go into the "Project" menu at the top, clic on "Properties" in the sliding menu. 
Then select to correct configuration according to your Eiger platform:

- Select sx12xxEiger-f10x if your Eiger platform is fitted with a F103 ST module
- Select sx12xxEiger-f4xx if your Eiger platform is fitted with a F407 ST module





