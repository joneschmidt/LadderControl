// LadderControl
//
// An Arduino-based sketch to control a model railroad turnout ladder
//
// *************************************************************************
// Revision history:
#define Version "LadrCntl 1.1 26/02/16"
//   2026/02/16 J. Schmidt 1.1 throw turnout relay before applying power
//   2025/11/22 J. Schmidt 0.6 Begin work extended addressing
//   2022/12/10 J. Schmidt 0.5 First debugged version
//   2022/11/17 J. Schmidt 0.1 Begin work
// *************************************************************************
// This code is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed WITHOUT ANY WARRANTY; 
// without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// http://www.gnu.org/licenses/.
// *************************************************************************
// BEGIN DEFINITIONS
// ***************** TRACE *****************
// DEBUG FLAGS
#define TRACE 0
#define TRACEDELAYSECS 5
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// vvvvvvvvvvv if using the IOX and wire librarys define USEIOX true
#define USEIOX true
// ^^^^^^^^^^^ if using the IOX and wire librarys define USEIOX true
// vvvvvvvvvvv if using photosensors define PHOTOSENSE true
#define PHOTOSENSE true
// ^^^^^^^^^^^ if using photosensors define PHOTOSENSE true
//
// vvvvvvvvvvvvvvvv BEGIN CONDITIONAL DEFINES
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#if USEIOX 
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  #include <Wire.h>
  // SDA/SCL for UNO & NANO
  #define IOXSDA A4
  #define IOXSCL A5
  // IOX board address base 
  #define IOXBBase 0x20
  // IOX address bias base 
  #define IOXABias 500
  // IOX output refresh time MS
  #define IOXRefMS 500
  // use XADR to define an IOX port
  // Xdev is the board address 20-27
  // Xpin is the bit number
  #define XADR(Xdev, Xpin) ((Xdev * IOXABias) + Xpin)
  // use XBRD to retrieve the board addr from an IOX port
  #define XBrd(Xad) (Xad / IOXABias)
  // use XADR to retrieve the bit address from an IOX port
  #define XBit(Xad) (Xad % IOXABias)
  // vvvvvvvvvv definitions for the MCP23017 chip vvvvvvvvvv
  // I2C command registers BANK = 0
  //----------------------
   #define X_IODIR          0x00
   #define X_ALOW           0x02
   #define X_GPPU           0x0C
   #define X_GPIO           0x12
  // ^^^^^^^^^^ definitions for the MCP23017 chip ^^^^^^^^^^
// Structure to define each IO Extender [20x-27x]
typedef struct {
  // direction bits
  uint16_t IOXDIR[2];
  // pullup bits
  uint16_t IOXPUP[2];
  // current IO bits
  uint16_t IOXGIOC[2];
  // previous IO bits
  uint16_t IOXGIOL[2];
  // last time updated
  uint32_t IOXLTME;
} IOXDef;
// Array IO Extender [20x-27x]
#define IOXCnt 8
IOXDef IOXBords [IOXCnt];
#endif // USEIOX
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// definition for PortS mode value -- do not change or move
#define INPUT_PHOTO 66
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#if PHOTOSENSE 
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// analog sampling controls - needed to average light flicker
// length of sample integration in millisecs
// 34 ms for 60 cycle lighting mains
// 40 ms for 50 cycle lighting mains
#define PhotoMS 34
// Sensitivity - sensor must have dropped at least % to trigger
#define Sentivity 60
// time to hold occupied status minimize false clears
#define HoldOccMS 2000
typedef struct {
  int           Largest    ; // largest analog values seen
  unsigned long HoldOccTM  ; // timer for false clears
} Photo;
#endif // PHOTOSENSE
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^ END CONDITIONAL DEFINES
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// BEGIN CONSTANTS	
// Port Id is NONE
  #define NONE	499
// Invert signal flags
  // if true, HIGH means not pushed
  #define INVERTBUTTONS true 
  // if true, HIGH is off
  #define INVERTRELAYS  true 
// milliseconds between throws
  #define THROWINTVMS    1000
// Turnout motor type
  // stall motor
  #define STALL 1
  #define STALLMS 3000 // millis of power on for STALL motors
  // momentary
  #define MOMNTRY 2
  #define MOMNTRYMS 200 // millis of power on for MOMNTRY motors
// Turnout Direction
  #define TNORM 1 // LOW signal
  #define TREV  2 // HIGH
// StatLED error flash rate
  #define FLASHMS       100 
// time button live before action
/*
#define DEBOUNCEINRMS   5 // checking debounce
#define DEBOUNCEMS     20 // must be continuous active debounce
*/
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ***************** Globals & Structures *****************
int Err, PathCnt;
typedef struct {
	int      PortAddr; // address
	uint8_t  PortIO;   // mode
} PortDef;
typedef struct {
	uint8_t  ValuNow;  // current indication
	uint8_t  ValuLast; // prior indication
} PortVal;
// Structure to define each turnout 
typedef struct {
  // turnout control HANDLE to port
  int TurnPortHandle;
  // turnout type - STALL or MOMNTRY
  uint8_t TurnType;
  // turnout power HANDLE - required for MOMNTRY 
  int TurnPowrHandle;
  // turnout occupied sensor HANDLE 
  int TurnOccHandle;
  // upstream turnout  - next higher turnout
  int TurnDUpHandle;  
  // upstream turnout - which does this turnout connect to
  // TNORM - TREV
  uint8_t TurnUpLeg;
} TurnoutDef;
// structure to define destination selection buttons
typedef struct {
  // destination select button HANDLE
  int ButtonHandle;
  // turnout - button selects turnout and
  int TurnDHandle; 
  // turnout direction
  uint8_t TurnLeg;
} TrkButnDef;
// structure for finding paths up the ladder
typedef struct {
  // index to turnout
  int TurnDHandle;  
  // upstream turnout
  int TurnLeg; 
  } PathDef;
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// USER DEFINITIONS
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINE THE LADDER
// Example - ladder
//               |  
//       _______/*\____Trn00 - Sns00*
//       |    N       R|   
//       |             | Tracks
//       |             | 
//       |          __/*\_____Trn01 - Sns01*
//       |         N       R  | 
//       |         |          | 
//       |      __/ \__Trn02  |
//       |     | N     | R    |
//    TBut00 TBut01 TBut02  TBut03
//
// SIGNAL PORT DEFINITIONS
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// DEFINE each signal pin
// the #define [name] [num] associates the name with 
//   the index into the port table
// the port table has two entries:
//  PortAddr - the pin address for the signal
//  PortIO   - INPUT, INPUT_PULLUP, INPUT_PHOTO, or OUTPUT
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// vvvvvvvvvvv DEFINE THE NUMBER OF PORTS
// USER: number of Arduino & XIO ports used
#define NUMPORTS    11
// USER: number of turnouts in ladder
#define NUMTURNOUTS 3
// USER: number of tracks 
#define NUMTRACKS   4
//
// define the Arduino ports
// buttons, sensors, and turnouts
// #define is the HANDLE -> position in the table
PortDef const PortS [NUMPORTS] = {
//  PortAddr - PortIO
// Buttons
#define But00P 0
	XADR(0x24,0), INPUT_PULLUP, // 0
#define But01P 1
	XADR(0x24,1), INPUT_PULLUP, // 1
#define But02P 2
	6, INPUT_PULLUP, // 2
#define But03P 3
	8, INPUT_PULLUP, // 3
// Turnouts
#define Trn00P 4
	XADR(0x24,5), OUTPUT,       // 4
#define Trn01P 5
	7, OUTPUT,       // 5
#define Trn02P 6
	9, OUTPUT,       // 6
// Turnout power
#define TPwr00P 7
   10, OUTPUT,       // 7
// Sensors
#define Sns00P  8
	A2, INPUT_PHOTO, // 8
#define Sns01P  9
	A3, INPUT_PHOTO, // 9
#define StatLED  10
	13, OUTPUT, // 10
};
// define the ladder
TurnoutDef const Turnouts [NUMTURNOUTS] = {
#define Trn00D 0
#define Trn01D 1
#define Trn02D 2
// TurnPortHandle, TurnType, TurnPowrHandle, TurnOccHandle, TurnDUpHandle TurnUpLeg
    Trn00P,     STALL,     NONE,           Sns00P,       NONE,         NONE, // 0 top of ladder
    Trn01P,     MOMNTRY,   TPwr00P,        Sns01P,       Trn00D,       TREV, // 1
    Trn02P,     STALL,     NONE,           NONE,         Trn01D,       TNORM // 2
  }; // Turnouts
// define the destination tracks, buttons, next higher turnout and direction
TrkButnDef const TrkButnS [NUMTRACKS] = {
// ButtonPort  TurnDHandle     TurnLeg
// track 0
      But00P,     Trn00D,      TNORM,
// track 1             
      But01P,     Trn02D,      TNORM,
// track 2            
      But02P,     Trn02D,      TREV,
// track 3             
      But03P,     Trn01D,      TREV
  }; // TrkButnS
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// GLOBAL VARIABLES
int Button;
// array for the path through the turnouts
PathDef Paths[NUMTURNOUTS];
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#if PHOTOSENSE 
Photo PhotoT[NUMPORTS];
#endif // PHOTOSENSE
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// current and last port values
PortVal PortVals [NUMPORTS];
// ###############################################################
void setup() {
int ii, jj, Tidx, Tleg;
bool ierr;
  #if TRACE > 0
    Serial.begin(9600);
    Serial.println (" ");
    Serial.println (Version);
    Serial.print   ("******* Startup TRACE = ");
    Serial.println (TRACE);
    #endif // TRACE

// walk the ports for problems
    for (ii = 0; ii < NUMPORTS -1; ++ii) {
    for (jj = ii + 1; jj < NUMPORTS; ++jj) {
	  if (PortS[ii].PortAddr == PortS[jj].PortAddr) 
		{
  #if TRACE > 0
		Serial.print("Port dup between ");
		Serial.print(String(ii));
		Serial.print(" and ");
		Serial.println(String(jj));
  #endif
		ShowStatus(50); // error
		}
	} // for jj
	} // for ii
    for (ii = 0; ii < NUMPORTS -1; ++ii) {
		if (PortS[ii].PortIO == INPUT_PHOTO
			&& PHOTOSENSE == false)
			{
  #if TRACE > 0
			Serial.println("Port " + String(ii) + " INPUT_PHOTO without PHOTOSENSE");
  #endif
			ShowStatus(50);} // error
		if (PortS[ii].PortAddr > NONE && USEIOX == false)
			{
  #if TRACE > 0
			Serial.println("Port " + String(ii) + " address " + String(PortS[ii].PortAddr) + " without USEIOX");
  #endif
			ShowStatus(50);} // error
	} // for ii
	
	ierr = false;
    for (ii = 0; ii < NUMTURNOUTS-1; ++ii) {
		if (Turnouts[ii].TurnDUpHandle >= NUMTURNOUTS
		    && Turnouts[ii].TurnDUpHandle != NONE){
  #if TRACE > 0
		Serial.print("Turnout ");
		Serial.print(ii);
		Serial.println(" up pointer is bad");
  #endif
		ierr = true;}
	  if (Turnouts[ii].TurnType == MOMNTRY
	       && Turnouts[ii].TurnPowrHandle == NONE)
		   {  
  #if TRACE > 0
			Serial.print("Turnout ");
		    Serial.print(ii);
            Serial.println (" #### Missing Momentary Power Port");
  #endif
		   ierr = true;}
    for (jj = ii + 1; jj < NUMTURNOUTS; ++jj) {
	  if (Turnouts[ii].TurnPortHandle == Turnouts[jj].TurnPortHandle) 
		{ierr = true;
  #if TRACE > 0
		Serial.print("Port conflict turnout ");
		Serial.print(ii);
		Serial.print(" and ");
		Serial.println(jj);
  #endif
		}
	} // for jj
	} // for ii
	if (ierr == true){ShowStatus(50);}

	ierr = false;
  #if TRACE > 1
    Serial.println("Walk the ladder");
    for (ii = 0; ii < NUMTURNOUTS; ++ii) {
    Serial.println ("Turnout " + String(ii) + ": ");
	  Tidx = ii;
    do {
      Tidx = Turnouts[Tidx].TurnDUpHandle;
	  if (Tidx == NONE)
	    {Serial.print ("--Top? No upper link.");}
      else
      {Serial.print ("--Ties to turnout " + String(Tidx)
	    + " type " + (Turnouts[Tidx].TurnType == STALL ? "Stall" : "Momentary"));
	  if (Turnouts[Tidx].TurnOccHandle != NONE)
		   {Serial.print (" TurnOccHandle " + String(Turnouts[Tidx].TurnOccHandle));}
	  if (Turnouts[Tidx].TurnPowrHandle != NONE)
		   {Serial.print (" TurnPowrHandle " + String(Turnouts[Tidx].TurnPowrHandle));}
	  if (Turnouts[Tidx].TurnType == MOMNTRY
	       && Turnouts[Tidx].TurnPowrHandle == NONE)
		   {Serial.print (" #### Missing Momentary Power Port");
		   ierr = true;}
	  Tleg = Turnouts[Tidx].TurnUpLeg;
      if (Tleg == TNORM) 
        {Serial.println (" normal");}
        else Serial.println (" reverse");
	  };
    } while (Tidx != NONE); // do while 
    Serial.println ();
    } // for ii
	
	Serial.println("Walk the buttons");
    for (ii = 0; ii < NUMTRACKS; ++ii) {
		Serial.print("Button " + String(ii));
		Tidx = TrkButnS[ii].TurnDHandle;
		Serial.print(" connects to ");
        Serial.print(Tidx);
	  	Tleg = TrkButnS[ii].TurnLeg;
        if (Tleg == TNORM) 
         {Serial.println (" normal");}
		else {Serial.println (" reverse");};
	} // ii NUMTRACKS
    #endif // TRACE
	if (ierr == true){ShowStatus(50);}

  InitPorts();
  ReadPorts();
  #if TRACE > 0
    Serial.println("End setup");
    #endif // TRACE
} // setup
// ###############################################################

// ###############################################################
void loop() {
  // Scan the buttons
  GetButton();
  if (Button != NONE){
    PathCnt = FindBestPath();
	if (PathCnt >= 0) {ThrowPaths();}
  }// Button pushed
  #if TRACE > 1 // TRACE
    Serial.println("End loop");
	delay(TRACEDELAYSECS * 1000.);
    #endif // TRACE
    ReadPorts();
} // Loop 
// ###############################################################

// ***************************************************************
// SUBROUTINES & FUNCTIONS
// ***************************************************************
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int FindBestPath(){
	int ii, lleg, lhand;
	String lstring;
	
	ii = 0;
	lhand = TrkButnS[Button].TurnDHandle;
	lleg  = TrkButnS[Button].TurnLeg;
	do {
       #if TRACE > 1 // TRACE
        Serial.print("FindBestPath " + String(lhand));
        if (lleg == TNORM) 
          {Serial.println (" normal");}
          else Serial.println (" reverse");
		#endif
		Paths[ii].TurnDHandle = lhand;
		Paths[ii].TurnLeg  = lleg;
		++ii;
		lleg  = Turnouts[lhand].TurnUpLeg;
	    lhand = Turnouts[lhand].TurnDUpHandle;
	} while (lhand != NONE);
	
	return(ii);
} // FindBestPath
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void GetButton(){
  // stay in this routine looping until a button is pushed 
  int Bval, ii, ihndl;
  String iturn;
  Button = NONE;
  ShowStatus(0);
  do {
	for (ii = 0; ii < NUMTRACKS; ++ii){
      ihndl = TrkButnS[ii].ButtonHandle;
      iturn = TrkButnS[ii].TurnDHandle;
      Bval = PortVals[ihndl].ValuNow;
      if (Bval == HIGH && PortVals[ihndl].ValuLast != HIGH){
          Button = ii;
		  #if TRACE > 0
           Serial.println ("Active Button " + String(Button) + " " + iturn + " port handle " + String(ihndl));
           #endif
          ShowStatus(1);
          return;
        } // if Bval HIGH
  } // for NUMTRACKS
  ReadPorts();
  #if TRACE > 0
    delay (1000);
  #endif
  } while (Button == NONE); // do while button NONE
} // Getbutton
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void ShowStatus(int Stat){
  int ii;
  #ifdef StatLED
switch (Stat){
	  case 0: // 0 - normal - turn off
	    WriteSensor(StatLED, LOW);
		break;
	  case 1: // 1 - button pushed
	    WriteSensor(StatLED, HIGH);
		break;
	  default: // > 1 -- error code flashing
	    for (ii = 0; ii < Stat; ++ ii){
	    WriteSensor(StatLED, LOW);
		delay(FLASHMS);
	    WriteSensor(StatLED, HIGH);
		delay(FLASHMS);
		} // for ii
	    WriteSensor(StatLED, LOW);
		break;
  } // switch Stat
	#endif
  return;	
} // ShowStatus
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void ThrowPaths(){
  int NormDiv, PowrWait, Activate, Clear, ii, lport, thand, lhand, lleg, lpwrhndl; 
// scan for path occupied before doing anything
  for (ii = 0; ii < PathCnt; ++ ii){
    thand = Paths[ii].TurnDHandle;
    if (Turnouts[thand].TurnOccHandle != NONE) // check occupancy
        {if (PortVals[Turnouts[thand].TurnOccHandle].ValuNow != 0)
		{ShowStatus(10);
         #if TRACE > 0
         Serial.print   ("Turnout " + String(thand));
         Serial.println (" occupy handle " + String(Turnouts[thand].TurnOccHandle) + " occupied");
		 #endif
		 return;}
		}// occ port exists
  } // for ii
  
  // throw the turnouts
  for (ii = 0; ii < PathCnt; ++ ii){
  thand = Paths[ii].TurnDHandle;
  lleg  = Paths[ii].TurnLeg;
  lport = Turnouts[thand].TurnPortHandle;
  #if TRACE > 0
    Serial.print("Throw turnout " + String(thand));
	if (lleg == TNORM) 
        {Serial.print (" normal ");}
        else {Serial.print (" reverse ");}
    Serial.print(" port handle " + String(lport));
    if (Turnouts[thand].TurnPowrHandle != NONE) // motor power
        {Serial.print (" power handle " + String(Turnouts[thand].TurnPowrHandle));}
    if (Turnouts[thand].TurnOccHandle != NONE) // check occupancy
        {Serial.print (" occupy handle " + String(Turnouts[thand].TurnOccHandle));}
	Serial.println(" ");
	#endif
  NormDiv = lleg == TREV ? HIGH : LOW;
  // now set the motor throw
  delay(THROWINTVMS);
  WriteSensor(lport, NormDiv);
  // check if there is a power relay for stall turnout motors
  PowrWait = 0; // minimum between throws
  lpwrhndl = Turnouts[thand].TurnPowrHandle;
  if (lpwrhndl != NONE) {// turn on motor power
    #if TRACE > 0
    Serial.println("--power on " + String(lpwrhndl));
	#endif
	WriteSensor(lpwrhndl, HIGH);
  // check for stall or momentary turnout motors
  if (Turnouts[thand].TurnType == STALL)
     {PowrWait = STALLMS;}
  if (Turnouts[thand].TurnType == MOMNTRY)
     {PowrWait = MOMNTRYMS;}
  } // checking power
   if (PowrWait != 0)
      {  
       #if TRACE > 0
         Serial.println("--power port " + String(lpwrhndl) + " drop after " + String(PowrWait) + " MS");
       #endif 
       delay(PowrWait);
	   WriteSensor(lpwrhndl, LOW);
	  } // power wait
  } // for ii
} // ThrowPaths
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void InitPorts(){
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  int Lhndl;
  int LMode, PPort;

  for (Lhndl = 0; Lhndl < NUMPORTS; ++Lhndl)
   {
	PortVals[Lhndl].ValuNow  = LOW;
	PortVals[Lhndl].ValuLast = LOW;
	#if TRACE > 1
      Serial.print   (Lhndl);
      Serial.print   (" addr ");
      Serial.println   (PortName(Lhndl));
    #endif
   } // for

  #if TRACE > 0
    CheckPorts();
  #endif

  InitIOXBds();
  InitPhoto();

  for (Lhndl = 0; Lhndl < NUMPORTS; ++Lhndl){
    PPort = PortS[Lhndl].PortAddr;
    LMode = PortS[Lhndl].PortIO;
    if (LMode == INPUT_PHOTO) {LMode = INPUT;}
    if (PPort < 500)
      {pinMode(PPort, LMode);}
    if (LMode == OUTPUT)
     {WriteSensor(Lhndl, LOW);}
    } // for Lhndl
} // InitPorts
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#if TRACE > 0
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void CheckPorts(){
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// check for errors in the port list
int ii, jj, err;
err = 0;
for (ii = 0; ii < NUMPORTS - 1; ++ii){
for (jj = ii+1; jj < NUMPORTS; ++jj){
	if (ii != jj && PortS[ii].PortAddr == PortS[jj].PortAddr){
      Serial.print   ("### Ports are dups: entries: " + String(ii) + " " + String(PortS[ii].PortAddr));
      Serial.print   (PortName(ii));
      Serial.print   (" " + String(jj) + " " + String(PortS[jj].PortAddr));
      Serial.println (PortName(jj));
	  err = 1;
	} // duplicate port
} // for jj
  #if USEIOX 
    if (PortS[ii].PortAddr == A4 || PortS[ii].PortAddr == A5)
	{Serial.print   ("### Port A4 - A5 cannot be used if IOX active: entry: " + String(ii) + " " + String(PortS[ii].PortAddr));
     Serial.println (PortName(ii));
     err = 1;
	}
  #endif
} // for ii
#if USEIOX 
// check for XADR problems
for (ii = 0; ii < NUMPORTS; ++ii){
  if (PortS [ii].PortAddr >= IOXABias){// if iox port
    jj = XBrd(PortS [ii].PortAddr);
    if (jj < 0x20 || jj > 0x27){// board must be between 20x - 27x
      Serial.print   ("### IOX Board address out of range 20x-27x entry: ");
      Serial.print   (String(ii) + " " + String(PortS[ii].PortAddr));
      Serial.print   (" board " + String(jj, HEX));
      Serial.println ("x" + PortName(ii));
      err = 1;
	} // bad board address
    jj = XBit(PortS [ii].PortAddr);
    if (jj > 15){// bit cant be > 15
      Serial.print   ("### IOX Bit address must be <= 15 entry: ");
      Serial.print   (String(ii) + " " + String(PortS[ii].PortAddr));
      Serial.print   (" bit " + String(jj));
      Serial.println (PortName(ii));
      err = 1;
	} // bad bit address
    if ((jj == 7 || jj == 15) && PortS [ii].PortIO != OUTPUT){// if iox port bit input & 7 or 15
      Serial.print   ("### IOX bit 7-15 cannot be input: entry: ");
      Serial.print   (String(ii) + " " + String(PortS[ii].PortAddr));
      Serial.print   (" bit " + String(jj));
      Serial.println (PortName(ii));
      err = 1;
	} // bad bit for input
  } // if extended port
} // for ii
  do {ii = 0;} while (err == 1); // dont continue -- loop forever if error
#endif
} // CheckPorts
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#endif
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void InitIOXBds(){

#if USEIOX 

  int Lbdx, Lbitx, Lbnkx, Lhndl;
  int baddr, bdata, bctrl; 
  
// initialize board array
  for (Lbdx= 0; Lbdx < IOXCnt; ++Lbdx){
	  for (Lbnkx = 0; Lbnkx <2; ++Lbnkx){
		IOXBords[Lbdx].IOXDIR[Lbnkx] = 0; // direction default OUTPUT
		IOXBords[Lbdx].IOXPUP[Lbnkx] = 0; // pullup default no
		IOXBords[Lbdx].IOXGIOC[Lbnkx] = 0; // current gpio data input
		IOXBords[Lbdx].IOXGIOL[Lbnkx] = 9999; // last gpio data input & board exists
	  } // for Lbnkx banks
    // new last time written
    IOXBords[Lbdx].IOXLTME = 0;
    } // for Lbdx < IOXCnt 

  // setup the direction/pullup/activelow bits
  for (Lhndl = 0; Lhndl < NUMPORTS; ++Lhndl){
	  if (PortS[Lhndl].PortAddr >= IOXABias){// if iox port
		Lbdx  = XBrd(PortS [Lhndl].PortAddr) - IOXBBase;
		Lbitx = XBit(PortS [Lhndl].PortAddr);
        IOXBords[Lbdx].IOXGIOL[0] = 0; // set board to active
		if (PortS [Lhndl].PortIO == INPUT)
		  {bitSet(IOXBords[Lbdx].IOXDIR[Lbitx / 8], Lbitx % 8);} 
		if (PortS [Lhndl].PortIO == INPUT_PULLUP)
		  {bitSet(IOXBords[Lbdx].IOXDIR[Lbitx / 8], Lbitx % 8);
		   bitSet(IOXBords[Lbdx].IOXPUP[Lbitx / 8], Lbitx % 8);} 
	  }// if IOX port 
  }// for NUMPORTS

    #if TRACE > 0
    for (Lbdx = 0; Lbdx < IOXCnt; ++Lbdx){
	  if (IOXBords[Lbdx].IOXGIOL[0] != 9999){
      Serial.print   ("InitIOXBds ");
      Serial.print   (IOXBBase + Lbdx, HEX);
      Serial.print   (" Direction: ");
      for (Lbnkx = 0; Lbnkx < 2; ++Lbnkx){// banks
        Serial.print (IOXBords[Lbdx].IOXDIR[Lbnkx], BIN);
	    Serial.print (" ");}
      Serial.print   ("Pullup: ");
      for (Lbnkx = 0; Lbnkx < 2; ++Lbnkx){// banks
        Serial.print (IOXBords[Lbdx].IOXPUP[Lbnkx], BIN);
	      Serial.print (" ");}
      Serial.print   (" IOC: ");
      for (Lbnkx = 0; Lbnkx < 2; ++Lbnkx){// banks
        Serial.print (IOXBords[Lbdx].IOXGIOC[Lbnkx], BIN);
	      Serial.print (" ");}
      Serial.print   (" IOL: ");
      for (Lbnkx = 0; Lbnkx < 2; ++Lbnkx){// banks
        Serial.print (IOXBords[Lbdx].IOXGIOL[Lbnkx], BIN);}
	    Serial.println (" ");
	  } // if active
    } // for IOXCnt
	#endif
	
// talk to the wire  
    Wire.begin(); // start as master
//
// now talk to the boards and set them up
  for (Lbdx= 0; Lbdx < IOXCnt; ++Lbdx){
    if(IOXBords[Lbdx].IOXGIOL[0] != 9999){// board active
	  baddr = Lbdx + IOXBBase;
		  
	  for (Lbnkx = 0; Lbnkx < 2; ++Lbnkx){// loop for banks
    // setup direction bits
	  bctrl = X_IODIR + Lbnkx; // bank
	  bdata = IOXBords[Lbdx].IOXDIR[Lbnkx];
      Wire.beginTransmission(baddr);  // Board Address
      Wire.write(bctrl);              // direction register
      Wire.write(bdata);              // control bits
      Wire.endTransmission();   
      #if TRACE > 0
         Serial.print   ("--Board ");
         Serial.print   (baddr, HEX);
         Serial.print   (" control ");
         Serial.print   (bctrl, HEX);
         Serial.print   ("X direction ");
         Serial.println (bdata, BIN);
         #endif
		  
	  // setup pullup bits
	  bctrl = X_GPPU + Lbnkx;
	  bdata = IOXBords[Lbdx].IOXPUP[Lbnkx];
      Wire.beginTransmission(baddr);  // Board Address
      Wire.write(bctrl);              // pullup register
      Wire.write(bdata);              // control bits
      Wire.endTransmission();   
      #if TRACE > 0
         Serial.print   ("----Pullup control ");
         Serial.print   (bctrl, HEX);
         Serial.print   ("X Pullup ");
         Serial.println (bdata, BIN);
         #endif

    // setup active low bits
	  bctrl = X_ALOW + Lbnkx;
      Wire.beginTransmission(baddr);  // Board Address
      Wire.write(bctrl);              // active low register
      Wire.write(bdata);              // control bits
      Err = Wire.endTransmission();   
      #if TRACE > 0
         Serial.print   ("----Active low control ");
         Serial.print   (bctrl, HEX);
         Serial.print   ("X active low " );
         Serial.print   (bdata, BIN);
         Serial.print   (" E ");
         Serial.println (Err);
         #endif
		  
	  // read GPIO bits
	  bctrl = X_GPIO + Lbnkx;
	  bdata = 0;
      Wire.beginTransmission(baddr);  // Board Address
      Wire.write(bctrl);              // gpio register
      Err = Wire.endTransmission();   
	  bdata = Wire.requestFrom(baddr,1);            // Data to read
      #if TRACE > 1
        Serial.print   ("InitIox request cnt ");
        Serial.print   (bdata);
        Serial.print   (" E ");
        Serial.println (Err);
        #endif
      bdata = Wire.read();
      IOXBords[Lbdx].IOXGIOC[Lbnkx] = bdata;
      IOXBords[Lbdx].IOXGIOL[Lbnkx] = bdata;
      #if TRACE > 0
         Serial.print   ("--Read GPIO in control ");
         Serial.print   (bctrl, HEX);
         Serial.print   ("X data ");
         Serial.println (bdata, BIN);
         #endif
	  } // for Lbnkx banks
	} // if active
  } // for IOXCnt  
#endif
} // InitIOXBds
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int ReadIOXBoards(){
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// read the IOX boards
#if USEIOX 
  int Lbdx, Lbnkx;
  int baddr, bdata, bctrl; 

	  Wire.end();
	  Wire.begin();
  for (Lbdx= 0; Lbdx < IOXCnt; ++Lbdx){
    if(IOXBords[Lbdx].IOXGIOL[0] != 9999){// board active
	  baddr = Lbdx + IOXBBase;
      #if TRACE > 1
         Serial.print ("ReadIOXBoards board ");
         Serial.println (baddr, HEX);
         #endif
	  // read GPIO bits
	  for (Lbnkx = 0; Lbnkx < 2; ++Lbnkx){// loop for banks
	  IOXBords[Lbdx].IOXGIOL[Lbnkx] = IOXBords[Lbdx].IOXGIOC[Lbnkx];
	  bctrl = X_GPIO + Lbnkx;
	  bdata = 0;
      Wire.beginTransmission(baddr);  // Board Address
      Wire.write(bctrl);              // gpio register
      Wire.endTransmission();   
	  bdata = Wire.requestFrom(baddr,1);            // Data to read
      bdata = Wire.read();
      IOXBords[Lbdx].IOXGIOC[Lbnkx] = bdata;
      #if TRACE > 1
         Serial.print ("--");
         Serial.print (Lbnkx);
         Serial.print (":");
         Serial.println (bdata, BIN);
          #endif
	  } // for Lbnkx
	} // if active
  }// for Lbidx
#endif
} // ReadIOXBoards
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int ReadPorts(){
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  int LMode, LHndl, Laddr, LValu;
  ReadIOXBoards(); // read boards before ports
  for (LHndl = 0; LHndl < NUMPORTS; ++ LHndl){
    Laddr = PortS[LHndl].PortAddr;
    PortVals[LHndl].ValuLast = PortVals[LHndl].ValuNow;
    LMode = PortS[LHndl].PortIO;
	if (LMode == INPUT_PHOTO || LMode == OUTPUT) 
	   {continue;}
	if (Laddr < NONE) 
	     {LValu = digitalRead(Laddr);}
	else {LValu = ReadIOXSensor(LHndl);} // external read
   if (LMode == INPUT_PULLUP && Laddr < NONE)
      {LValu = (LValu == LOW?HIGH:LOW);}
	PortVals[LHndl].ValuNow = LValu;
   #if TRACE > 1
     if (LMode != OUTPUT && LValu != 0) {
       Serial.print ("ReadPorts ");
       Serial.print (LHndl);
       Serial.print (PortName(LHndl));
       Serial.print (" value ");
	   Serial.println (LValu);}
     #endif
  } // for LHndl
  ReadPhotos();  
} // ReadPorts
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int ReadIOXSensor(int Hidx){
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// return the bit requested
#if USEIOX 
  int Lbdx, Lbitx;
  Lbdx =  XBrd(PortS[Hidx].PortAddr) - IOXBBase;
  Lbitx = XBit(PortS[Hidx].PortAddr);
   #if TRACE > 1
       Serial.print   ("ReadIOXSensor");
       Serial.println (PortName(Hidx));
	   if (PortS[Hidx].PortIO == OUTPUT){
         Serial.print   ("###ReadIOXSensor");
         Serial.println (PortName(Hidx) + " is OUTPUT");}
     #endif
  return bitRead(IOXBords[Lbdx].IOXGIOC[Lbitx / 8], Lbitx % 8);
#endif
} // ReadIOXSensor
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int WriteIOXBoards(){
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// write the IOX boards
#if USEIOX 
  int Lbidx, Lbnkx;
  int baddr, bdata, bctrl; 
  for (Lbidx= 0; Lbidx < IOXCnt; ++Lbidx){
    if(IOXBords[Lbidx].IOXGIOL[0] != 9999){// board active
    if(IOXBords[Lbidx].IOXLTME + IOXRefMS < millis()){// write refresh or changed
	  baddr = Lbidx + IOXBBase;
      #if TRACE > 0
         Serial.print ("WriteIOXBoards ");
         Serial.print (baddr, HEX);
         Serial.print ("x control ");
         Serial.print (bctrl, HEX);
         Serial.print ("x ");
          #endif
	  for (Lbnkx = 0; Lbnkx < 2; ++Lbnkx){
	  bctrl = X_GPIO + Lbnkx;
	  bdata = IOXBords[Lbidx].IOXGIOC[Lbnkx];
      #if TRACE > 0
         Serial.print(Lbnkx);
         Serial.print(":");
         Serial.print(IOXBords[Lbidx].IOXGIOC[Lbnkx], BIN);
         Serial.print("b ");
          #endif
      Wire.beginTransmission(baddr);  // Board Address
      Wire.write(bctrl);              // GPIO register
      Wire.write(bdata);              // data
      Wire.endTransmission();   
	  } // for jj
      #if TRACE > 0
	     Serial.println(" ");
         #endif
	  IOXBords[Lbidx].IOXLTME = millis(); // set refresh time
	} // if write changed
	} // if board active
  }// for Lbidx
#endif
} // WriteIOXBoards
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void WriteSensor(int Hidx, int Pval){
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  uint16_t PPort;
  PPort = PortS[Hidx].PortAddr;
   if (PPort < 500)
   {digitalWrite(PPort, Pval);
   #if TRACE > 0
     Serial.print   ("WriteSensor ");
     Serial.print   (PortName(Hidx));
     Serial.print   (" handle " + String(Hidx));
     Serial.println (Pval == LOW?" LOW":" HIGH");
	 if (PortS[Hidx].PortIO != OUTPUT){
         Serial.print   ("###WriteSensor");
         Serial.println (PortName(Hidx) + " is NOT OUTPUT");}
	 #endif
   } else {WriteIOXSensor(Hidx, Pval);}// write bits changed
} // WriteSensor
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int WriteIOXSensor(int Hidx, int Pval){
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// set the bit requested
#if USEIOX 
  int Lbdx, Lbitx;
  Lbdx  = XBrd(PortS[Hidx].PortAddr) - IOXBBase;
  Lbitx = XBit(PortS[Hidx].PortAddr);
  if (Pval == LOW)
     {bitClear(IOXBords[Lbdx].IOXGIOC[Lbitx / 8], Lbitx % 8);}
  else {bitSet(IOXBords[Lbdx].IOXGIOC[Lbitx / 8], Lbitx % 8);}
   #if TRACE > 0
     Serial.print   ("WriteIOXSensor ");
     Serial.print   (PortName(Hidx));
     Serial.println (Pval == LOW?" LOW":" HIGH");
	 #endif
   IOXBords[Lbdx].IOXLTME = 1;
   WriteIOXBoards();
#endif
} // WriteIOXSensor
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
String PortName(int Hidx){
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int ll, brd, bit;
String Pname;
 
  ll = PortS[Hidx].PortAddr;
  Pname = "??";
  if (ll < A0)
    {Pname = " port D" + String(ll);}
  else 
      {Pname = " port A" + String(ll - A0);}
  #if USEIOX 
	if (ll >= IOXABias)
    {
		brd = XBrd(PortS[Hidx].PortAddr);
		bit = XBit(PortS[Hidx].PortAddr);
		Pname = " port " + String(brd, HEX) + "x " + String(bit);
	  } // XIO
  #endif

  ll = PortS[Hidx].PortIO;
  if (ll == INPUT_PULLUP)
    {Pname = Pname + " INPU";}
  if (ll == INPUT_PHOTO)
    {Pname = Pname + " INPHO";}
  if (ll == INPUT)
    {Pname = Pname + " IN";}
  if (ll == OUTPUT)
    {Pname = Pname + " OUT";}
  return Pname;
} // PortName
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void InitPhoto(){
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#if PHOTOSENSE 
int ii;
for (ii = 0; ii < NUMPORTS; ++ii)
 {PhotoT[ii].Largest   = 0;
  PhotoT[ii].HoldOccTM = 0;
 } // for ii
ReadPhotos();
#endif
} // InitPhoto
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void ReadPhotos(){
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#if PHOTOSENSE 
  int LPtx;
  long int Valus[NUMPORTS];
  long int Smpls, Tpct;
  unsigned long CycleStrt;
  for (LPtx = 0; LPtx < NUMPORTS; ++LPtx)
    {Valus[LPtx] = 0;}
     Smpls = 0;
     CycleStrt = millis();
	 
  do { // do the integration over PhotoMS time
   for (LPtx = 0; LPtx < NUMPORTS; ++LPtx)
    {if (PortS[LPtx].PortIO != INPUT_PHOTO) continue;
     Valus[LPtx] += analogRead(PortS[LPtx].PortAddr);
    } // for LPtx
    ++Smpls;} // do
	while (CycleStrt + PhotoMS > millis());

   for (LPtx = 0; LPtx < NUMPORTS; ++LPtx)
    {if (PortS[LPtx].PortIO != INPUT_PHOTO) continue;
    Valus[LPtx] /= Smpls; // get average
    PhotoT[LPtx].Largest = max(PhotoT[LPtx].Largest, Valus[LPtx]);
    Tpct = 100. - (Valus[LPtx] * 100.)/PhotoT[LPtx].Largest;
    PortVals[LPtx].ValuLast = PortVals[LPtx].ValuNow;
	if (Tpct >= Sentivity) { // occupied
	  PortVals[LPtx].ValuNow = HIGH;
	  PhotoT[LPtx].HoldOccTM = millis();
	} else { // not occupied
	  if (PhotoT[LPtx].HoldOccTM + HoldOccMS <= millis()){// hold expired
	    PortVals[LPtx].ValuNow = LOW;
	  } // hold expired
	} // not occupied
	#if TRACE > 0
	if (PortVals[LPtx].ValuNow == HIGH){
     Serial.print   ("ReadPhotos");
     Serial.print   (PortName(LPtx));
     Serial.print   (" ");
	 Serial.println (PortVals[LPtx].ValuNow);}
	 #endif
	} // for LPtx
	#endif
} // ReadPhotos
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
