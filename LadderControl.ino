// LadderControl
//
// An Arduino-based sketch to control a model railroad turnout ladder
//
// *************************************************************************
// Revision history:
#define Version "LadderControl 0.1 2022/12/05"
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
// Structure to define each turnout 
typedef struct {
  // turnout id - text name of turnout
  String TurnSId; 
  // turnout type - STALL or MOMNTRY
  uint8_t TurnType;
  // turnout control port 
  uint32_t TurnPort;
  // turnout power port - required for MOMNTRY 
  uint32_t TurnPowrPort;
  // turnout occupied sensor port 
  uint32_t TurnOccPort;
  // turnout path value
  int TurnWeight;
  // upstream turnout id - next higher turnout
  String TurnUpSId;  
  // upstream turnout leg - which leg does this turnout connect to
  int TurnUpLeg; 
  // 2nd upstream turnout id - if an upward facing turnout,
  // which other turnout does it connect to?
  String TurnUpSId2;  
  // 2nd upstream turnout leg
  int TurnUpLeg2;
} TurnDef;
//
// Define track selection
typedef struct {
  // track select button port
  uint32_t ButtonPort;
  // turnout id - button selects which leg of which turnouot
  String TurnSId; 
  // turnout leg
  int TurnLeg;
} TrackDef;
//
// BEGIN Constants
// Turnout motor type
  // stall motor
  #define STALL 1
  #define STALLMS 3000 // duration of power on for STALL
  // momentary
  #define MOMNTRY 2
  #define MOMNTRYMS 200 // duration of power on for MOMNTRY
// Turnout Leg Direction
  #define TNORM 1 // LOW signal
  #define TDVRG 2 // HIGH
// time button live before action
#define FLASHMS       100 // StatusLED flash rate
#define DEBOUNCEINRMS   5 // checking debounce
#define DEBOUNCEMS     20 // must be continactive debounce
#define LADDERINTVMS  200 // time between turnout throws
#define LADDERPWRMS   400 // power time for turnout
// ***************** TRACE *****************
// DEBUG FLAGS
#define TRACE 1
#define TRACEDELAYSECS 5
  // false: run without executing - TRACE only
  #define EXECUTE false
  // #define EXECUTE true
//
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// USER DEFINITIONS
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// Example 1
//               |  T1
//            __/ \__
//           /       \   
//          /\ T2    /\ T3
//         /  \     /  \
//      T4/\   |   /\T5 |
//       /  \  |  /  \  |
//       0  1  2  3  4  5  Tracks
//
// USER: number of turnouts in ladder
#define NUMTURNOUTS 5
// USER: number of tracks 
#define NUMTRACKS   6
// USER: seconds between throws
#define THROWINTVMS    1000
// USER: power on for momentary throws
#define THROWPOWERMS   1500
//
// define the ladder
TurnDef Turnouts [NUMTURNOUTS] = {
// TurnSId TurnType TurnPort  TurnPowrPort  TurnOccPort TurnWeight TurnUpSId  TurnUpLeg  TurnUpSId2 TurnUpLeg2
    "T1",   STALL,    3,        0,            0,           0,         "",        0,        "",        0, 
    "T2",   STALL,    4,        0,            0,           0,       "T1",        TNORM,    "",        0, 
    "T3",   STALL,    5,        0,            0,           0,       "T1",        TDVRG,    "",        0, 
    "T4",   STALL,    6,        0,            0,           0,       "T2",        TNORM,    "",        0, 
    "T5",   STALL,    7,        0,            0,           0,       "T3",        TNORM,    "",        0
  }; // Turnouts
// define the destination tracks & selector ports
TrackDef Tracks [NUMTRACKS] = {
// ButtonPort  TurnSId  TurnLeg
// track 0
      A0,     "T4",      TNORM,
// track 1
      A1,     "T4",      TDVRG,
// track 2
      A2,     "T2",      TDVRG,
// track 3
      A3,     "T5",      TNORM,
// track 4
      A4,     "T5",      TDVRG,
// track 5
      A5,     "T3",      TDVRG
  }; // Tracks
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// GLOBAL VARIABLES
int ii,jj;
int ButtonIdx, Button, LastButton;
// data for finding paths up the ladder
typedef struct {
  // index to turnout
  int TurnUpIdx;  
  // upstream turnout leg
  int TurnUpLeg; 
  } PathDef;
String TSid;
#define PATHBEST 0
#define PATHTEST 1
PathDef  Path[NUMTURNOUTS][2];
long int PathWeight[2];
int NumPaths, Pidx, BestIdx, Tidx, Tleg;
// define a status LED if desired or -1
int StatusLED = {13};
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void InitExtrnPorts(){
// set up any external sensor/port devices
// ?????????????
} // InitExtrnPorts
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void InitPort(uint32_t PPort, int Pmode){
  int LPort;
if (PPort < 100)
  {LPort = PPort;
   pinMode(LPort, Pmode);}
  else ;// external init
} // InitPort
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int ReadSensor(uint32_t PPort){
  int LPort;
if (PPort < 100)
  {LPort = PPort;
   return(digitalRead(PPort));}
  else ; // external read
} // ReadSensor
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void WritePort(uint32_t PPort, int Pval){
  int LPort;
if (PPort < 100)
  {LPort = PPort;
   digitalWrite(LPort, Pval);}
  else ;// external write}
} // WritePort
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
int GetTurnoutIdx(String PTSid){
  int kk;
  for (kk = 0; kk < NUMTURNOUTS; ++kk){
    if (PTSid == Turnouts[kk].TurnSId)
    {return (kk);}
  } // for kk
  return (-1);  
} // GetTurnoutIdx
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void FindBestPath(){
  // initialize the arrays
    for (ii = 0; ii < 2; ++ii) {
	PathWeight[ii] = 0;
    for (jj = 0; jj < NUMTURNOUTS; ++jj) {
      Path[jj][ii].TurnUpIdx = -1;
      Path[jj][ii].TurnUpLeg = -1;
    } // for jj
    } // for ii
	// start tracing path from button/track
	TSid = Tracks[Button].TurnSId;
    Tleg = Tracks[Button].TurnLeg;
	Pidx = 0;
	BestIdx = 0;
	PathWeight[BestIdx] = 0;
    do {
      Tidx = GetTurnoutIdx(TSid);
      #if TRACE > 0
        Serial.print ("--Ties to turnout " + TSid + " index " + String(Tidx));
		#endif
      if (Tleg == TNORM) 
        {Serial.println (" normal leg");}
        else Serial.println (" divergent leg");
		// check path occupied
      if (Turnouts[Tidx].TurnOccPort  > 0) 
	    {if (ReadSensor(Turnouts[ii].TurnOccPort) == HIGH)
			{BestIdx = -1; // abort if HIGH
		     return;};
		} // check occupied
	  PathWeight[BestIdx] += 1 + Turnouts[Tidx].TurnWeight;
      Path[Pidx][BestIdx].TurnUpIdx = Tidx;
      Path[Pidx][BestIdx].TurnUpLeg = Tleg;
	  ++Pidx;
      TSid = Turnouts[Tidx].TurnUpSId;
      Tleg = Turnouts[Tidx].TurnUpLeg;
    } while (TSid != ""); // do while jj
    #if TRACE > 0
  	  Serial.print ("-- Weight " + String(PathWeight[BestIdx]));
      Serial.println ();
	  #endif

} // FindBestPath
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void GetButton(){
  // stay in this routine looping until a button is pushed 
  int Bval;
  unsigned long BTime;
  Button = -1;
  do {
      Bval = ReadSensor(Tracks[ButtonIdx].ButtonPort);
      if (Bval == HIGH){
        BTime = millis();
        do {
          delay(DEBOUNCEINRMS);
          Bval = ReadSensor(Tracks[ButtonIdx].ButtonPort);
          } while (Bval == HIGH && (BTime + DEBOUNCEMS > millis())); // do while Bval
        if (Bval == HIGH){
          Button = ButtonIdx;
		  #if TRACE > 0
             Serial.print  ("Active Button " + String(Button));
          #endif
          return;} // Bval HIGH
        } // do Bval HIGH
	++ButtonIdx;
	ButtonIdx = ButtonIdx < NUMTRACKS ? ButtonIdx : 0;
  } while (Button == -1);// do while button -1
} // Getbutton
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void ShowStatus(int Stat){
  if (StatusLED < 0) return;
  switch (Stat){
	  case 0: // 0 - normal - turn off
	    WritePort(StatusLED, LOW);
		break;
	  case 1: // 1 - button pushed
	    WritePort(StatusLED, HIGH);
		break;
	  default: // > 1 -- error code flashing
	    for (ii = 0; ii < Stat; ++ ii){
	    WritePort(StatusLED, LOW);
		delay(FLASHMS);
	    WritePort(StatusLED, HIGH);
		delay(FLASHMS);
		} // for ii
	    WritePort(StatusLED, LOW);
		break;
  } // switch Stat
  return;	
} // ShowStatus
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void ThrowPath(){
  // walk the best path throwing the turnouts
  for (ii = 0; ii < Pidx; ++ii){
	 ThrowTurnout(Path[Pidx][BestIdx].TurnUpIdx,Path[Pidx][BestIdx].TurnUpLeg);
	 delay(THROWINTVMS);
  } // for
} // ThrowPath
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
void ThrowTurnout(int PPidx, int PPleg){
  int NormDiv, PowrWait; // HIGH means diverge
  // throw the turnout
  #if TRACE > 0
    Serial.print("Throw turnout " + Turnouts[PPidx].TurnSId);
	if (PPleg == TNORM) 
        {Serial.println (" normal leg");}
        else Serial.println (" divergent leg");
	#endif
  NormDiv = PPleg == TDVRG ? HIGH : LOW;
  // check if there is a power relay for stall turnout motors
  PowrWait = 0;
  if (Turnouts[PPidx].TurnType == STALL
		&& Turnouts[PPidx].TurnPowrPort != 0) // turn on motor power
   {WritePort(Turnouts[PPidx].TurnPowrPort, HIGH);
   PowrWait = STALLMS;}
  // now set the motor throw
  WritePort(Turnouts[PPidx].TurnPort, NormDiv);
  // check for momentary turnout motors
  if (Turnouts[PPidx].TurnType == MOMNTRY 
		&& Turnouts[PPidx].TurnPowrPort != 0)
   {WritePort(Turnouts[PPidx].TurnPowrPort, HIGH);
    PowrWait = MOMNTRYMS;}
   if (PowrWait != 0)
      {delay(PowrWait);
	   WritePort(Turnouts[PPidx].TurnPowrPort, LOW);} 
} // ThrowTurnout
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void setup() {
  #if TRACE > 0
    Serial.begin(9600);
    Serial.print  (Version);
    Serial.print  ("******* Startup TRACE = ");
    Serial.println(TRACE);
    #endif // TRACE
  #if TRACE > 0
    Serial.println("Validate track structure");
    Serial.println("Walk the buttons");
    for (ii = 0; ii < NUMTRACKS; ++ii) {
    Serial.println ("Button/track " + String(ii));
    TSid = Tracks[ii].TurnSId;
    Tleg = Tracks[ii].TurnLeg;
	PathWeight[PATHBEST] = 0;
    do {
      Tidx = GetTurnoutIdx(TSid);
      Serial.print ("--Ties to turnout " + TSid + " index " + String(Tidx)
	    + " type " + (Turnouts[Tidx].TurnType == STALL ? "Stall" : "Momentary"));
	  if (Turnouts[Tidx].TurnType == MOMNTRY
	       && Turnouts[Tidx].TurnPowrPort == 0)
		   {Serial.print (" #### Missing Momentary Power Port");}
      if (Tleg == TNORM) 
        {Serial.println (" normal leg");}
        else Serial.println (" divergent leg");
	    PathWeight[PATHBEST] += 1 + Turnouts[Tidx].TurnWeight;
		// ??JJJ error check for MOMNTRY without Powerport
      TSid = Turnouts[Tidx].TurnUpSId;
      Tleg = Turnouts[Tidx].TurnUpLeg;
    } while (TSid != ""); // do while jj
	Serial.print ("-- Weight " + String(PathWeight[PATHBEST]));
    Serial.println ();
    } // for ii
    #endif // TRACE
  // Initialize the pins
    InitExtrnPorts();
    if (StatusLED > 0) {
		InitPort(StatusLED, OUTPUT);
	    WritePort(StatusLED, LOW);}
    for (ii = 0; ii < NUMTRACKS; ++ii) {
      InitPort(Tracks[ii].ButtonPort, INPUT);
      } // for ii
    for (ii = 0; ii < NUMTURNOUTS; ++ii) {
      InitPort(Turnouts[ii].TurnPort, OUTPUT);
      if (Turnouts[ii].TurnPowrPort > 0) 
		{InitPort (Turnouts[ii].TurnPowrPort, OUTPUT);
		 WritePort(Turnouts[ii].TurnPowrPort, LOW);}
      if (Turnouts[ii].TurnOccPort  > 0) {InitPort(Turnouts[ii].TurnOccPort,   INPUT);}
      } // for ii 
  LastButton = -1;
  ButtonIdx  =  0;
  #if TRACE > 0
    Serial.println("End setup");
    #endif // TRACE
} // setup
// #######################################################################################
void loop() {
  #if TRACE > 0 // TRACE
    Serial.println("Begin loop");
    #endif // TRACE
  // Scan the buttons
  GetButton();
  if (Button >= 0 && Button != LastButton){
	ShowStatus(1); // on - show work in process
    FindBestPath();
	if (BestIdx >= 0)
		ThrowPath();
  }// Button pushed
  	ShowStatus(0); // normal - off
  #if TRACE > 0 // TRACE
    Serial.println("End loop");
	delay(TRACEDELAYSECS * 1000);
    #endif // TRACE
} // Loop #######################################################################################
