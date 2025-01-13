/*
 NMEA0183/NMEA2000 library. NMEA0183 -> NMEA2000 -> NMEA0183
   Reads GPS messages from NMEA0183_In (Serial3) and forwards them to the N2k bus
   and to NMEA0183_out (SerialUSB)
   Also forwards all NMEA2000 bus messages to the PC (Serial)
   Forwards also wind data from N2k bus to the NMEA0183_out

 To use this example you need both NMEA2000 and NMEA0183 library installed.

 The example works with default settings on Arduino DUE, since it uses
 board second USB SerialUSB. That can be changed by definitions on the
 beginning of code. Note that on DUE SerialUSB blocks for some reason, when
 you close the port on PC, so the whole program stops. It continues, when
 you open port again. If you do not need NMEA2000 messages forwarded to PC,
 define Serial for NMEA0183_Out_Stream and comment line:
 #define N2kForward_Stream Serial

 Example reads NMEA0183 messages from one serial port. It is possible
 to add more serial ports for having NMEA0183 combiner functionality.

 The NMEA0183 messages, which will be handled has been defined on NMEA0183Handlers.cpp
 on NMEA0183Handlers variable initialization. So this does not automatically
 handle all NMEA0183 messages. If there is no handler for some message you need,
 you have to write handler for it and add it to the NMEA0183Handlers variable
 initialization. If you write new handlers, please after testing send them to me,
 so I can add them for others use.

 Example also reads NMEA2000 messages and forwards them to defined N2kForward_Stream
 - Serial as default.

 For NMEA2000 message handling and forwarding to NMEA0183_Out there is only sample
 N2kWindDataHandler defined on N2kWindDataHandler.h. For each new handler you need
 to create handler class for it and call NMEA2000.AttachMsgHandler(&<handler class>);
 as for N2kWindDataHandler at end of setup()
*/

#include <Arduino.h>
#include <stdio.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include "N2kDataToNMEA0183.h"
#include "BoatData.h"
#include "Log.h"
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object

#include <MemoryFree.h>
#include <TeensyView.h>    // Include the SFE_TeensyView library

///////////////////////////////////
// TeensyView Object Declaration //
///////////////////////////////////
//Standard
constexpr int PIN_RESET = 15;
constexpr int PIN_DC = 5;
constexpr int PIN_CS = 10;
constexpr int PIN_SCK = 13;
constexpr int PIN_MOSI = 11;

TeensyView oled(PIN_RESET, PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI);

tBoatData BoatData;
Logger* logger;

constexpr long NMEA0183_Stream_Speed = 38400;
#define NMEA0183_Stream Serial            // USB im Teensy
constexpr long NMEA0183_1_Stream_Speed = 115200;
#define NMEA0183_1_Stream Serial1         // USB an TX1/RX1
//constexpr long  NMEA0183_3_Stream_Speed = 38400;
//#define NMEA0183_3_Stream Serial3         // RS232 an TX3/RX3

constexpr long N2kForward_Stream_Speed = 115200;
#define N2kForward_Stream Serial1

//Stream* N2kHandlersDebugStream = &Serial;
//Stream* N2kHandlersDebugStream = &Serial1;
Stream* N2kHandlersDebugStream = nullptr;                //                to switch off, set to nullptr

//Stream* NMEA0183HandlersDebugStream = &Serial;
//Stream* NMEA0183HandlersDebugStream = &Serial1;
Stream* NMEA0183HandlersDebugStream = nullptr;           //                to switch off, set to nullptr

const int ledBuiltin = LED_BUILTIN;

tNMEA0183 NMEA0183;
tNMEA0183 NMEA0183_1;
//tNMEA0183 NMEA0183_3;


// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = { 126992L,127250L,127258L,129026L,129029L,0 };    // System Time, Vessel Heading, Magnetic Variation, COG & SOG, Rapid Update, GNSS Position Data 
const unsigned long ReceiveMessages[] PROGMEM = { 130306L,0 };                                     // Wind Data 

// NMEA 2000 handlers
tN2kDataToNMEA0183 N2kDataToNMEA0183(&NMEA2000, &NMEA0183);

// *****************************************************************************
// Empty stream input buffer. Ports may get stuck, if they get data in and it will
// not be read out.
static void FlushStreamInput(Stream& stream) {
	while (stream.available()) {
		stream.read();
	}
}

unsigned long NMEA0183TxCounter = 0;
unsigned long NMEA0183RxCounter = 0;
unsigned long N2kTxCounter = 0;
unsigned long N2kRxCounter = 0;

unsigned long LoopCounter = 0;

static char line[] = "                ";

static void initializeOLED() {
	oled.begin();    // Initialize the OLED
	oled.flipVertical(true);
	oled.flipHorizontal(true);
	oled.clear(PAGE); // Clear the display's internal memory
	oled.display();  // Display what's in the buffer (splashscreen)
}

/**
 * @brief Initialize hardware and software components.
 */
void setup() {
	initializeOLED();
	oled.setFontType(0);         // Smallest font
	oled.setCursor(0, 0);        // Set cursor to top-left

	// Setup NMEA2000 system
	N2kForward_Stream.begin(N2kForward_Stream_Speed);

	NMEA0183_Stream.begin(NMEA0183_Stream_Speed);
	NMEA0183_1_Stream.begin(NMEA0183_1_Stream_Speed);
	//NMEA0183_3_Stream.begin(NMEA0183_3_Stream_Speed);

	delay(1000); // Give some time for serial to initialize

	// to use the can0 with Tindie CAN-Bus Adapter, set High speed mode
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);

	Serial.begin(115200);

	logger = new Logger(&Serial, DEBUG_LEVEL_TRACE);

	info("Start initializing NMEA200 library. Free memory:%u", freeMemory());


	NMEA2000.SetForwardStream(&N2kForward_Stream);
	NMEA2000.SetProductInformation("0000042",             // Manufacturer's Model serial code
		107,                   // Manufacturer's product code
		"Teensy V3.2",         // Manufacturer's Model ID
		"1.40 (2019-08-02)",   // Manufacturer's Software version code
		"N2k->NMEA0183->N2K"); // Manufacturer's Model version

	// Det device information
	NMEA2000.SetDeviceInformation(42,    // Unique number. Use e.g. Serial number.
		130,   // Device function=PC Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
		25,    // Device class=Inter/Intranetwork Device. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
		2046); // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               


	//  NMEA2000.SetDeviceInformation(13333, // Unique number. Poduct code
	//								130,     // Device function=Display Device. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
	//								120,     // Device class=Display Device. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
	//								1851);   // Manufacturer=Raymarine. See codes on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               

	  //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Commented means show in Actisense format.

	NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 25);
	//NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 25);

	NMEA2000.EnableForward(true);               // Forward N2K MSGs to serial Port additionally to N2K-Bus, Format see SetForwardType above
	//NMEA2000.EnableForward(false);                // do not Forward N2K MSGs to serial Port additionally to N2K-Bus, Format see SetForwardType above

	NMEA2000.ExtendTransmitMessages(TransmitMessages);
	NMEA2000.ExtendReceiveMessages(ReceiveMessages);

	NMEA2000.Open();

	// Setup NMEA0183 ports and handlers
	InitNMEA0183Handlers(&NMEA2000, &BoatData);
	NMEA0183.SetMsgHandler(HandleNMEA0183Msg);
	NMEA0183_1.SetMsgHandler(HandleNMEA0183Msg);
	//NMEA0183_3.SetMsgHandler(HandleNMEA0183Msg);

	NMEA0183.SetMessageStream(&NMEA0183_Stream);
	NMEA0183.Open();
	NMEA0183_1.SetMessageStream(&NMEA0183_1_Stream);
	NMEA0183_1.Open();
	//NMEA0183_3.SetMessageStream(&NMEA0183_3_Stream);
	//NMEA0183_3.Open();

	pinMode(ledBuiltin, OUTPUT);

	//  while (!Serial);                    // wait until serial ready
	//  Serial.begin(NMEA0183_Stream_Speed);
	//  OutputStream = &Serial;
	Serial.println("N2k-ver1.0-USB buildtin");

	//  Serial1.begin(NMEA0183_1_Stream_Speed);
	//  Output1Stream = &Serial1;
	Serial1.println("N2k-ver1.0-USB ext");

	//  Serial3.begin(NMEA0183_3_Stream_Speed);
	//  Output3Stream = &Serial3;
	//  Serial3.println("N2k-ver1.0-RS232");

	oled.begin();    // Initialize the OLED
	oled.flipVertical(true);
	oled.flipHorizontal(true);
}

/**
 * @brief Main loop for processing messages and updating diagnostics.
 */
void loop() {
	NMEA2000.ParseMessages();
	N2kDataToNMEA0183.Update();

	NMEA0183.ParseMessages();
	// We need to clear output streams input data to avoid them to get stuck.
	FlushStreamInput(NMEA0183_Stream);

	NMEA0183_1.ParseMessages();
	// We need to clear output streams input data to avoid them to get stuck.
	FlushStreamInput(NMEA0183_1_Stream);

	// NMEA0183_3.ParseMessages();
	// We need to clear output streams input data to avoid them to get stuck.
	// FlushStreamInput(NMEA0183_3_Stream);

#ifdef N2kForward_Stream
	FlushStreamInput(N2kForward_Stream);
#endif

	SendSystemTime();

	ledBuiltinUpdate();

	LoopCount();
}

/**
 * @brief Update LED state for diagnostics.
 */

constexpr unsigned long ledBuiltinUpdatePeriod = 2000;

static void ledBuiltinUpdate() {
	static unsigned long ledBuiltinUpdated = millis();

	if (ledBuiltinUpdated + ledBuiltinUpdatePeriod < millis()) {
		ledBuiltinUpdated = millis();
		digitalWrite(ledBuiltin, HIGH);    //LED=on
		delay(100);
		digitalWrite(ledBuiltin, LOW);     //LED=off

		Serial.println ("Serial:   ledBuiltinUpdate()");
		Serial1.println("Serial_1: ledBuiltinUpdate()");
		Serial3.println("Serial_3: ledBuiltinUpdate()");
	}
}

/**
 * @brief LoopCount.
 */

constexpr unsigned long LoopCountUpdatePeriod = 1000;

static void LoopCount() {
	static unsigned long LoopCountUpdated = millis();

	if (LoopCountUpdated + LoopCountUpdatePeriod < millis()) {
		LoopCountUpdated = millis();

		LoopCounter = LoopCounter + 1;

		oled.setCursor(0, 0);        // Set cursor to top-left
		sprintf(line, "NMEA0183Tx: %9d", (int)NMEA0183TxCounter);
		oled.println(line);
		oled.display();

		oled.setCursor(0, 8);        // Set cursor to top-middle-left
		sprintf(line, "NMEA0183Rx: %9d", (int)NMEA0183RxCounter);
		oled.println(line);
		oled.display();

		oled.setCursor(0, 16);       // Set cursor to top-middle-left
		sprintf(line, "NMEA2000Tx: %9d", (int)N2kTxCounter);
		oled.println(line);
		oled.display();

		oled.setCursor(0, 24);       // Set cursor to top-middle-left
		sprintf(line, "NMEA2000Rx: %9d", (int)N2kRxCounter);
		oled.println(line);
		oled.display();
	}
}

/**
 * @brief Send SystemTime.
 */

constexpr unsigned long TimeUpdatePeriod = 5000;

static void SendSystemTime() {
	static unsigned long TimeUpdated = millis();
	tN2kMsg N2kMsg;

	//	if ( (TimeUpdated+TimeUpdatePeriod<millis()) && BoatData.DaysSince1970>0 ) {
	if (TimeUpdated + TimeUpdatePeriod < millis()) {
		Serial.println("SendSystemTime()");
		SetN2kSystemTime(N2kMsg, 0, BoatData.DaysSince1970, BoatData.GPSTime);       // PGN126992: System Time
		TimeUpdated = millis();
		NMEA2000.SendMsg(N2kMsg);
		N2kTxCounter = N2kTxCounter + 1;
	}
}

