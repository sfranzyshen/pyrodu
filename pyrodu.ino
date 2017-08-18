#include	<Adafruit_NeoPixel.h>
#include	<FS.h>

// general definitions            		
#define		TOTAL_NODES 91 // total number of nodes 
#define		VERSION 3 // 2017 neopixel version 
 // interval limits                   
#define   MIN_FRAME_INTERVAL 35
#define   DEFAULT_FRAME_INTERVAL 100 /// Final Decision? What should this be?
#define   MAX_FRAME_INTERVAL 10000

// duration limits                        
#define   MIN_FRAME_DURATION 10
#define   DEFAULT_FRAME_DURATION 75 /// Final Decision? What should this be?
#define   MAX_FRAME_DURATION 750

// neopixel info
#define   NEO_PIN D1
//#define   NEO_PIXELS 31 // 31 * 3 = 93 
#define   NEO_PIXELS 91 // using the model ... do not forget to set live above
                               		
// Control Mode
boolean		live = false; // Are we live or on the model ...
int		controlMode = 1; // 0 = random pattern 1 = indexed pattern 2 = manual control
boolean		active = true; // Are we even going there or not? ON/OFF switch.
boolean		status = false;
char  messageBuffer[8]; // Can't forsee more than 8c*s as long as we stay away from long pattern titles.
int bufferIndex = 0;
String  currentFile;

// FS stuff
Dir root;
File  pattern;

int totalFiles = 0;
int realFiles = 0; // corrupted data some how??

// runtime variables
long		nodeTimeStamps[TOTAL_NODES]; // Since defining arrays requires you put in the total number of elements, add 1
long		nodeDurations[TOTAL_NODES]; // Array for disparaging durations
long		frameInterval = DEFAULT_FRAME_INTERVAL; // Interval between frames
long		frameDuration  = DEFAULT_FRAME_DURATION; // Time a given  is on 

// Pattern Loop Settings.
static int8_t	loopThresh = 3;
static int8_t	loopCount = loopThresh + 1; //start out over thresh

//autoPilot (right now random, let's diversify!)
static boolean	autoPilot = true; // Initiates the pilot
static uint16_t	serialTimeout = 20000; // How long after serial is silent will autoPilot begin.
static long	lastSerialCMD = 0; // Last time a serial command was recieved
uint8_t		readMode = 0; // Wait

// timing.
static long	then = 0; // Last time a frame was refreshed
static long	now = 0; // Will be populated with the current millisecond at the beginning of the loop

// mapping for new neopixel format (12 different angles)
uint16_t Led_Map0[] =	{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90};
uint16_t Led_Map1[] =	{64,63,78,65,49,48,47,62,77,87,79,66,50,34,33,32,31,46,61,76,86,88,80,67,51,35,19,18,17,16,30,45,60,75,85,90,89,81,68,52,36,20,7,6,15,14,29,44,59,74,84,83,82,69,53,37,21,8,1,5,4,13,28,43,58,73,72,71,70,54,38,22,9,2,0,3,12,27,42,57,56,55,39,23,10,11,26,41,40,24,25};
uint16_t Led_Map2[] =	{61,75,76,62,46,60,59,74,85,86,77,63,47,31,45,44,43,58,73,84,90,87,78,64,48,32,16,30,29,28,27,42,57,72,83,89,88,79,65,49,33,17,15,14,13,12,26,41,56,71,82,81,80,66,50,34,18,6,5,4,3,11,25,40,55,70,69,68,67,51,35,19,7,1,0,2,10,24,39,54,53,52,36,20,8,9,23,38,37,21,22};
uint16_t Led_Map3[] =	{73,72,84,74,58,57,56,71,83,90,85,75,59,43,42,41,40,55,70,82,89,86,76,61,60,49,28,27,26,25,24,39,54,69,81,88,87,77,62,46,45,29,13,12,11,10,23,38,53,68,80,79,78,63,47,31,30,14,4,3,2,9,22,37,52,67,66,65,64,48,32,16,15,5,0,1,8,21,36,51,50,49,33,17,6,7,20,35,34,18,19};
uint16_t Led_Map4[] =	{70,69,82,71,55,54,53,68,81,89,83,72,56,40,39,38,37,52,67,80,88,90,84,73,57,41,25,24,23,22,21,36,51,66,79,87,86,85,74,58,42,26,11,10,9,8,20,35,50,65,78,77,76,75,59,43,27,12,3,2,1,7,19,34,49,64,63,62,61,60,44,28,13,4,0,5,6,18,33,48,47,46,45,29,14,15,17,32,31,30,16};
uint16_t Led_Map5[] =	{67,66,80,68,52,51,50,65,79,88,81,69,53,37,36,35,34,49,64,78,87,89,82,70,54,38,22,21,20,19,18,33,48,63,77,86,90,83,71,55,39,23,9,8,7,6,17,32,47,62,76,85,84,72,56,40,24,10,2,1,5,15,16,31,46,61,75,74,73,57,41,25,11,3,0,4,14,30,45,60,59,58,42,26,12,13,29,44,43,27,28};
uint16_t Led_Map6[] =	{28,27,43,44,29,13,12,26,42,58,59,60,45,30,14,4,0,3,11,25,41,57,73,74,75,61,46,31,16,15,5,1,2,10,24,40,56,72,84,85,76,62,47,32,17,6,7,8,9,23,39,55,71,83,90,86,77,63,48,33,18,19,20,21,22,38,54,70,82,89,87,78,64,49,34,35,36,37,53,69,81,88,79,65,50,51,52,68,80,66,67};
uint16_t Led_Map7[] =	{16,30,31,32,17,15,14,29,45,46,47,48,33,18,6,5,0,4,13,28,44,60,61,62,63,64,49,34,19,7,1,2,3,12,27,43,59,75,76,77,78,65,50,35,20,8,9,10,11,26,42,58,74,85,86,87,79,66,51,36,21,22,23,24,25,41,57,73,84,90,88,80,67,52,37,38,39,40,56,72,83,89,81,68,53,54,55,71,82,69,70};
uint16_t Led_Map8[] =	{19,18,34,35,20,7,6,17,33,49,50,51,36,21,8,1,0,5,15,16,32,48,64,65,66,67,52,37,22,9,2,3,4,14,30,31,47,63,78,79,80,68,53,38,23,10,11,12,13,29,45,46,62,77,87,88,81,69,54,39,24,25,26,27,28,49,60,61,76,86,89,82,70,55,40,41,42,43,59,75,85,90,83,71,56,57,58,74,84,72,73};
uint16_t Led_Map9[] =	{22,21,37,38,23,9,8,20,36,52,53,54,39,24,10,2,0,1,7,19,35,51,67,68,69,70,55,40,25,11,3,4,5,6,18,34,50,66,80,81,82,71,56,41,26,12,13,14,15,17,33,49,65,79,88,89,83,72,57,42,27,28,29,30,16,32,48,64,78,87,90,84,73,58,43,44,45,31,47,63,77,86,85,74,59,60,46,62,76,75,61};
uint16_t Led_Map10[] =	{25,24,40,41,26,11,10,23,39,55,56,57,42,27,12,3,0,2,9,22,38,54,70,71,72,73,58,43,28,13,4,5,1,8,21,37,53,69,82,83,84,74,59,44,29,14,15,6,7,20,36,52,68,81,89,90,85,75,60,45,30,16,17,18,19,35,51,67,80,88,86,76,61,46,31,32,33,34,50,66,79,87,77,62,47,48,49,65,78,63,64};
uint16_t Led_Map11[] =	{90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0};

uint16_t *Led_Map; // pointer to current mapping

uint16_t Ring0[] =	{0,1,2,3,4};
uint16_t Ring2[] =	{5,6,7,8,9,10,11,12,13,14};
uint16_t Ring3[] =	{15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};
uint16_t Ring4[] =	{30,31,32,33,34,35,36,37,38,39,40,41,42,43,44};
uint16_t Ring5[] =	{45,46,47,48,49,50,51,52,53,54,55,56,57,58,59};
uint16_t Ring6[] =	{60,61,62,63,64,65,66,67,68,69,70,71,72,73,74};
uint16_t Ring7[] =	{75,76,77,78,79,80,81,82,83,84};
uint16_t Ring8[] =	{85,86,87,88,89};
uint16_t Ring9[] =	{90};

// NeoPixelBus driver stuff
Adafruit_NeoPixel Led_Strip = Adafruit_NeoPixel(NEO_PIXELS, NEO_PIN, NEO_RGB + NEO_KHZ800);
uint8_t* Led_Array = (uint8_t*)Led_Strip.getPixels(); // get direct pointer to the led array

// Setup()
void setup(){                
	
	Serial.begin(115200);
	Serial.println("Setup ...");

	Led_Strip.begin();
	Led_Strip.show();

	//flash();
	status = mount();
	
	Serial.print("Mode: ");
	Serial.println(controlMode);
}

// Loop()
void loop() {
  
  while(Serial.available()) { // polling the serial connection
    char x = Serial.read();
    serialRouting(x);
    yield();
  }
  
  if(!status) {
    Serial.println("ReMounting...");
    //flash();
    status = mount();
    return;
  }  
  
	if (!active) { 
		ceaseFire();
    return;
	}
	
	now = millis(); 
	
	flameSustain(); // Sustains flame based on each nodes last timestamp and current frameDuration	
	modeSelektor(); // Select mode based on information.
	ignite(); // Send the 1011 and let the people have some fun.	
}

// Mount()
boolean mount() {
	bool error = false;
	File f;
	totalFiles = 0;
  realFiles = 0;
	//root.close();

	Serial.print("Initializing SPIFFS filesystem ... ");


	// see if the filesystem is present and can be initialized:
	if (!SPIFFS.begin()) {
    Serial.println("SPIFFS failed, or not present");
		error = true;
	} else {
		Serial.println("SPIFFS initialized.");
		root = SPIFFS.openDir("/patterns");
    while(root.next()) {
			totalFiles++;
   		Serial.print(root.fileName());
			Serial.print(" :");
			f.close();
    	f = root.openFile("r");
			if(!f) {
			  Serial.println("ERR");
        Serial.println("Error ... file open failed ");
			  //error = true;
			} else {
					//Serial.println(root.fileSize());// we don't have to open the file to get it's size ... but why not?			
    			Serial.println(f.size());
			}
    }
	}

	if(error) { 	
		Serial.println("Mount Failed, Trying again soon ..."); 
		return false;
	} else {
		Serial.print("Total Files: ");
		Serial.println(totalFiles);
    //realFiles = totalFiles;
		//root.close();
		root = SPIFFS.openDir("/patterns"); // Reset root to top
		return true;
	}
}

// serialRouting()
void serialRouting(char x) {
  
	//Flags, set read mode., begin
	if ( x == '!' ) { readMode = 1; } //Pattern
	else if ( x == '@' ) { readMode = 2; } //Frame Duration
	else if ( x == '#' ) { readMode = 3; } //Frame Interval
	else if ( x == '+' ) { readMode = 4; } //Nodes separated by comma (no whitespace)
	else if ( x == '-' ) { readMode = 5; } //Nodes separated by comma (no whitespace)
	else if ( x == '~' ) { readMode = 6; } //Mode 
	else if ( x == '*' ) { readMode = 7; } //Active 	
  else if ( x == '>' ) { nextPattern(); loopCount = 0; } //Next Pattern
	else if ( x == '/' ) { getFiles(); }		
	else if ( x == '?' ) { statusUpdate(); }			
	//Add custom flags here.
	
	//Finish up
	else if (x == '.') { 	//...
		
		//This will update the global variables accordingly.
		switch(readMode){
			case 1: setPattern(); break;
			case 2: setDuration(); break;
			case 3: setInterval(); break;
			case 4: setValveOn(); break;
			case 5: setValveOff(); break;
			case 6: setMode(); break;			
			case 7: setActive(); break;						
			default: break;	
		}
		
		lastSerialCMD = now; //Used for switching to autoPilot
		readMode = 0; //We're done reading. (until another.)
		autoPilot = false;
		bufferIndex = 0;
					
	} else { 
		messageBuffer[bufferIndex++] = x; //Magic.
	} 
}

// CeaseFire() - Sets everything to off.
void ceaseFire(){
	Led_Strip.clear();
	Led_Strip.show();
}

// FlameSustain()
void flameSustain(){
	long onFor;

	for(int i = 0; i < TOTAL_NODES; i++) { // This loop turns off nodes based on their timestamp and how long each is to be on
		onFor = now - nodeTimeStamps[i];
  
		if(nodeTimeStamps[i] <= 0) 
			continue; 
  
		if((controlMode != 2 && onFor > nodeDurations[i]) || onFor > MAX_FRAME_DURATION) {		
			nodeOff(i);
		} 				
	}
}

// ModeSelektor()
void modeSelektor(){
	long since = now - then;
  if (controlMode == 2) {	
    return; // ignore patterns
  }
	if(since > frameInterval || since > MAX_FRAME_INTERVAL){  
		nextFrame();// Go to next frame		
	}	
}

// NextFrame()
void nextFrame(){

	if( updateFrame() ) { // end of pattern ... test loop
		if(loopCount > loopThresh){
			if( controlMode == 0 ) { 
				randomPattern();
			} else {
				if( controlMode == 1 ) { 
					nextPattern(); 
				}
			}
			loopCount = 0;		
    		} else {
      			loopCount++;
    		}
	}
  	then = now;
}

// UpdateFrame()
boolean updateFrame(){
		
	size_t index = 0;
	size_t data_size = 92; // This variable sets the number of bytes to read from the file.
	byte fileContents[data_size]; // Temporary buffer to pull data from SPIFFS
    
	index = pattern.readBytes((char*)fileContents, data_size); //file_contents is the data buffer for storing data. data_size is a variable set to the amount of data to read.

	if(index != data_size) {
		pattern.seek(0, SeekSet); // Go back to beginning of file
    Serial.print("Rewinding ... ");
    Serial.println(index); 
		return 1;
	}

	for(int i = 0; i < data_size - 1 ; i++){ // -1 for cr at end of frame
		if(fileContents[i] == '1'){
			//int j = i;					
			//if(i >= 32){ // Dealing with the offsets in the file format
			//	j--;
			//}
			//if(i >= 64){ // Still dealing with the offsets in the file format
			//	j -= 2;
			//}
			nodeOn(i);
		}
	}
	return 0;
}

// NextPattern()
void nextPattern() {	

	if(!root.next()) { // test next dir entry ... if not rewind
		
		Serial.println("Rewinding ... "); 
		root = SPIFFS.openDir("/patterns"); // Reset root to top
		if(!root.next()) {
			Serial.println("Error: end of dir entries?");
		}
	}

	Serial.print("Next Pattern ... ");
  currentFile = root.fileName();
	Serial.println(currentFile);
	pattern.close();
	pattern = root.openFile("r"); // FIXME: test and handle failure
}

// RandomPattern()
void randomPattern() {

	Serial.print("Selecting Random Animation: "); 
	int randNum = random(totalFiles);
	root = SPIFFS.openDir("/patterns"); // Reset root to top
  Serial.print(randNum);
	// Run root.next() a random number of times to get to the file ...
  if(randNum == 0) {
    if(!root.next()) {
      Serial.println("Error: end of dir entries");
    }
  } else { 
	  for(int i = 0; i < randNum; i++){
		  if(!root.next()) {
			  Serial.println("Error: end of dir entries");
		  } 
	  }
  }
  currentFile = root.fileName();
  Serial.println(currentFile);   
	pattern.close();
	pattern = root.openFile("r"); // FIXME: test and handle failure
}

// GetFiles()
void getFiles() {      
  // while(root.readDir(&directory)) {      
  //  
  // }
}

// StatusUpdate()
void statusUpdate() {
  // Serial.print("Is Dave there?");
  Serial.println("<=== Pyrosphere Status Update ===>"); 
  Serial.print("Frame Interval: ");
  Serial.println(frameInterval); 
  Serial.print("Frame Duration: ");
  Serial.println(frameDuration); 
  Serial.print("System Mode: ");
  Serial.println(controlMode); 
  //Serial.print("Real Files: ");
  //Serial.println(realFiles);
  Serial.print("Total Files: ");
  Serial.println(totalFiles);
  Serial.print("Loaded Pattern: ");
  Serial.println(currentFile);
  Serial.print("Current Loop Count: ");
  Serial.println(loopCount);
  Serial.println("===>");
  
  resetMessageBuffer();
}

// SetPattern()
void setPattern(){
  
    //char *patternName = "/patterns/";
    char *patternName =  messageBuffer;
    //strcat(patternName, messageBuffer);
    strcat(patternName, ".dat");
    
    Serial.print("Setting Pattern: ");
    Serial.println(patternName);
    
    setMode(1);   // break out of auto.
    
    changePattern(patternName);
    
    resetMessageBuffer();
    
}

// SetDuration()
void setDuration(){
  
    frameDuration = atoi(messageBuffer);
    resetMessageBuffer();
    
    //Serial.print("Setting Duration: ");
    //Serial.println(frameDuration);
  
}  

// setInterval()
void setInterval(){
  
    frameInterval = atoi(messageBuffer);
    resetMessageBuffer();
    
    //Serial.print("Setting Interval: ");
    //Serial.println(frameInterval);
  
}

// setValveOn()
void setValveOn() {
    
  int valveID = atoi(messageBuffer);

  if (valveID <= TOTAL_NODES) {
    //Serial.print("Setting Valve On: ");
    //Serial.println(valveID);  
    nodeOn(valveID);
  }

  resetMessageBuffer();   

  if (controlMode != 2) {
    setMode(2);// controlMode = 2; /// This might be in the wrong place.
  }
}
  
// SetValveOff()
void setValveOff() {
    
  int valveID = atoi(messageBuffer);
    
  if (valveID <= TOTAL_NODES) {
    //Serial.print("Setting Valve Off: ");
    //Serial.println(valveID);     
    nodeOff(valveID);     
  }
  
  resetMessageBuffer(); 
   
  if (controlMode != 2) {
    setMode(2);// controlMode = 2; /// This might be in the wrong place.
  }
}

// setMode()
void setMode(){
    char *modeSig = messageBuffer;
    uint8_t mode = atoi(modeSig); 
    
    setMode(mode);
    
    resetMessageBuffer();
}

// setMode(uint8_t m)
void setMode(uint8_t m){
  
      
    //Serial.print("Setting Mode: ");
    //Serial.println(m);
    
    //This will update the global variables accordingly.
    switch(m){
      // Autopilot Random
      case 0:       controlMode = 0;      
        // frameInterval = DEFAULT_FRAME_INTERVAL;
        // frameDuration = DEFAULT_FRAME_DURATION;
        loopCount = 0;
        break; // Off.

      // Autopilot Progressive  
      case 1:       controlMode = 1;      
        // frameInterval = DEFAULT_FRAME_INTERVAL;
        // frameDuration = DEFAULT_FRAME_DURATION;
        loopCount = 0;
        break; 

      // Valve Control  
      case 2:       controlMode = 2;      
        // frameDuration = MAX_FRAME_DURATION;
        break; 

      case 3:       controlMode = 3;      
        break; // 
      case 4:       controlMode = 4;      
        break; // ...
      case 5:       controlMode = 5;      
        break; // ...
      default:                            
        break;  
    } 
    

    
}

// SetActive()
void setActive() {
  
    char *activeSig = messageBuffer;
    int sig = atoi(activeSig); 
      
    // Serial.print("Setting Switch: ");
    // Serial.println(sig);
    
    //This will update the global variables accordingly.
    switch(sig){
      case 0:       active = false;   break; // Off.
      case 1:       active = true;    break; // On.
      case 2:       resetPattern();   break; // Reset ///Do we want to activate here too?
      default:                        break;  
    } 
    
    resetMessageBuffer();
    
}

// ResetMessageBuffer()
void resetMessageBuffer(){
    memset( messageBuffer, '\0', sizeof(messageBuffer) );   
}

// ChangePattern()
void changePattern(char *fileName) {
  String stringOne =  String("/patterns/" + String(fileName));
  
  pattern.close(); // Close the current file
  Serial.print("Opening Pattern: "); 
  pattern = SPIFFS.open(stringOne, "r"); // Open the file in read mode
  
  if(pattern.name() == NULL) { // file open failed
      Serial.println("Error ");
      Serial.print("Couldn't open pattern: "); 
      Serial.println(fileName); 
  } else {
    currentFile = pattern.name();
    Serial.println(currentFile);
  }

}

// ResetPattern()
void resetPattern () {
  // FIXME: do something?
}

// Ignite()
void ignite(){
	Led_Strip.show();
}

// NodeOn()
void nodeOn(int8_t nodeNum){

	if(live) {
		Led_Array[nodeNum] = 255;
	} else {
		Led_Strip.setPixelColor(nodeNum, Led_Strip.Color(255,255,255));
	}

	nodeTimeStamps[nodeNum] = millis();
	nodeDurations[nodeNum] = frameDuration;
}

// NodeOff()
void nodeOff(int8_t nodeNum){

	if(live) {
		Led_Array[nodeNum] = 0;
	} else {
		Led_Strip.setPixelColor(nodeNum, Led_Strip.Color(0,0,0));
	}

	nodeTimeStamps[nodeNum] = -1;
	nodeDurations[nodeNum] = 0;
}

