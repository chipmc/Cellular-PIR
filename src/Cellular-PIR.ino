/*
* Project Cellular-PIR - converged software for Low Power and Solar
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland chip@mcclellands.org
* Sponsor: Simple Sense - alex@simplesense.io  www.simplesense.io
* Date:28 November 2017
*/

/*  The idea of this release is to unify the code base between PIR sensors
    Both implementations will move over to the finite state machine approach
    Both implementations will observe the park open and closing hours
    I will add two new states: 1) Low Power mode - maintains functionality but conserves battery by
    enabling sleep  2) Low Battery Mode - reduced functionality to preserve battery charge

    The mode will be set and recoded in the CONTROLREGISTER so resets will not change the mode
    Control Register - bits 7-4, 3 - Verbose Mode, 2- Solar Power Mode, 1 - Low Battery Mode, 0 - Low Power Mode
*/

// Easy place to change global numbers
//These defines let me change the memory map and configuration without hunting through the whole program
#define VERSIONNUMBER 8             // Increment this number each time the memory map is changed
#define WORDSIZE 8                  // For the Word size the number of bytes in a "word"
#define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
#define HOURLYOFFSET 24             // First word of hourly counts (remember we start counts at 1)
#define HOURLYCOUNTNUMBER 4064      // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
// First Word - 8 bytes for setting global values
#define VERSIONADDR 0x0             // Where we store the memory map version number
#define SENSITIVITYADDR 0x1         // Sensitivity for Accelerometer sensors
#define GRANULARITYADDR 0x2          // One uint8_t for granularity (stored in seconds)
#define RESETCOUNT 0x3              // This is where we keep track of how often the Electron was reset
#define KEEPSESSIONADDR 0x4         // Here we store the percent that we add to granularity to get the Keep Session value
#define TIMEZONEADDR  0x5           // Store the local time zone data
        //  OPEN BYTE
#define CONTROLREGISTER 0x7         // This is the control register for storing the current state - future use
//Second and Third words bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8  // Current Hourly Count - 16 bits
#define CURRENTHOURLYDURATIONADDR 0xA   // Current Hourly Duration Count - 16 bits
#define CURRENTDAILYCOUNTADDR 0xC   // Current Daily Count - 16 bits
#define CURRENTCOUNTSTIME 0xE       // Time of last count - 32 bits
#define HOURLYPOINTERADDR 0x11      // Two bytes for hourly pointer
                                    // Four open bytes here which takes us to the third word
//These are the hourly and daily offsets that make up the respective words
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "0.44"
#define PARKCLOSES 18
#define PARKOPENS 6


// Included Libraries
#include "Adafruit_FRAM_I2C.h"                           // Library for FRAM functions
#include "FRAM-Library-Extensions.h"                     // Extends the FRAM Library
#include "electrondoc.h"                                 // Documents pinout

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);    // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                      //Initalize the PMIC class so you can call the Power Management functions below.

// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, LOW_BATTERY_STATE, REPORTING_STATE, RESP_WAIT_STATE };
State state = INITIALIZATION_STATE;

// Pin Constants
const int tmp36Pin =      A0;               // Simple Analog temperature sensor
const int wakeUpPin =     A7;               // This is the Particle Electron WKP pin
const int enablePin =     B4;               // Hold low to power down the device - for GPS Equipped units
const int tmp36Shutdwn =  B5;               // Can turn off the TMP-36 to save energy
const int intPin =        D3;               // PIR Sensor Interrupt pin
const int hardResetPin =  D4;               // Power Cycles the Electron and the Carrier Board
const int userSwitch =    D5;               // User switch with a pull-up resistor
const int donePin =       D6;               // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;               // This LED is on the Electron itself


// Timing Variables
unsigned long publishTimeStamp = 0;         // Keep track of when we publish a webhook
unsigned long webhookWaitTime = 45000;      // How long will we let a webhook go before we give up
unsigned long resetWaitTimeStamp = 0;       // Starts the reset wait clock
unsigned long resetWaitTime = 30000;        // Will wait this lonk before resetting.
unsigned long sleepDelay = 90000;           // Longer delay before sleep when booting up or on the hour - gives time to flash
unsigned long timeTillSleep = 0;            // This will either be short or long depending on nap or sleep

bool waiting = false;                       // Keeps track of things that are in flight - enables non-blocking code
bool readyForBed = false;                   // Keeps track of the things that you do once before sleep

// Program Variables
int temperatureF;                           // Global variable so we can monitor via cloud variable
int resetCount;                             // Counts the number of times the Electron has had a pin reset
bool ledState = LOW;                        // variable used to store the last LED status, to toggle the light
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
int lowBattLimit = 30;                      // Trigger for Low Batt State
bool lowPowerMode;                          // Flag for Low Power Mode operations
bool lowBatteryMode;                        // Battery is critical - must not connect and will sleep
byte controlRegister;                       // Stores the control register values
bool solarPowerMode = false;                // Changes the PMIC settings
bool verboseMode = true;                    // Enables more active communications for configutation and setup
retained char Signal[17];             // Used to communicate Wireless RSSI and Description
char Status[17] = "";                 // Used to communciate updates on System Status
const char* levels[6] = {"Poor", "Low", "Medium", "Good", "Very Good", "Great"};

// FRAM and Unix time variables
time_t t;
byte lastHour = 0;                          // For recording the startup values
byte lastDate = 0;                          // These values make sure we record events if time has lapsed
int hourlyDurationCount = 0;                // This is where we count the duration "periods" which are defined by the value of granularity
int hourlyDurationCountSent = 0;            // Keep track of counts in flight
int hourlyPersonCount = 0;                  // hourly counter
int hourlyPersonCountSent = 0;              // Person count in flight to Ubidots
int dailyPersonCount = 0;                   // daily counter
bool dataInFlight = false;                  // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
int averageHourlyDuration = 0;              // Running average duration value
byte currentHourlyPeriod;                   // This is where we will know if the period changed
byte currentDailyPeriod;                    // We will keep daily counts as well as period counts

// PIR Sensor variables
volatile bool sensorDetect = false;         // This is the flag that an interrupt is triggered
volatile unsigned long lastEvent = 0;       // Keeps track of the last time there was an event
float keepSessionFactor;                    // This is the amount of time that we will wait before declaring a new session (expressed as a factor of granularity)
unsigned long napDelay;                     // Normal amount of time after event before taking a nap - it is defined as 10% more than the granularity * keepSessionFactor
unsigned long granularity;                  // Triggers less than this amount will be ignored
int keepSession;                            // The value of the time we will keep a session alive - in seconds


// Battery monitor
int stateOfCharge = 0;                      // stores battery charge level value


void setup()                                                      // Note: Disconnected Setup()
{
  pinMode(enablePin,OUTPUT);                                      // For GPS enabled units
  digitalWrite(enablePin,LOW);                                    // Turn off GPS to save battery
  pinMode(intPin,INPUT);                                          // PIR Sensor Interrupt pin
  pinMode(wakeUpPin,INPUT);                                       // This pin is active HIGH
  pinMode(userSwitch,INPUT);                                      // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);                                   // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);                               // Turns on the temp sensor
  pinMode(donePin,OUTPUT);                                        // Allows us to pet the watchdog
  digitalWrite(donePin,HIGH);
  digitalWrite(donePin,LOW);                                      // Pet the watchdog
  pinMode(hardResetPin,OUTPUT);                                   // For a hard reset active HIGH

  attachInterrupt(wakeUpPin, watchdogISR, RISING);   // The watchdog timer will signal us and we have to respond
  attachInterrupt(intPin,sensorISR,RISING);   // Will know when the PIR sensor is triggered

  char responseTopic[125];
  String deviceID = System.deviceID();                                // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", hourlyPersonCount);                // Define my Particle variables
  Particle.variable("DailyCount", dailyPersonCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("granularity", granularity);
  Particle.variable("keepSession",keepSession);
  Particle.variable("Signal", Signal);
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Temperature",temperatureF);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerMode);

  Particle.function("Reset-FRAM", resetFRAM);
  Particle.function("Reset-Counts",resetCounts);
  Particle.function("Soft-Reset",resetNow);
  Particle.function("Hard-Reset",hardResetNow);
  Particle.function("Granularity",setGranularity);
  Particle.function("KeepSession",setKeepSession);
  Particle.function("Send-Now",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);

  if (!fram.begin()) {                                                  // You can stick the new i2c addr in here, e.g. begin(0x51);
    snprintf(Status,13,"Missing FRAM");                                 // Can't communicate with FRAM - fatal error
    state = ERROR_STATE;
  }
  else if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {                   // Check to see if the memory map in the sketch matches the data on the chip
    snprintf(Status,13,"Erasing FRAM");
    ResetFRAM();                                                        // Reset the FRAM to correct the issue
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) state = ERROR_STATE;   // Resetting did not fix the issue
    else {
      FRAMwrite8(CONTROLREGISTER,0);                                    // Need to reset so not in low power or low battery mode
      FRAMwrite8(TIMEZONEADDR,-5);                                      // Set the timezone to EST - sorry at least I know what it is
    }
  }

  resetCount = FRAMread8(RESETCOUNT);                                   // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET)                   // Check to see if we are starting from a pin reset
  {
    resetCount++;
    FRAMwrite8(RESETCOUNT,static_cast<uint8_t>(resetCount));            // If so, store incremented number - watchdog must have done This
  }

  // Here we load the values for granularity and keepSession from FRAM
  granularity = FRAMread8(GRANULARITYADDR);                                // Load granularity value from FRAM - seconds
  keepSessionFactor = 1 + ((float)FRAMread8(KEEPSESSIONADDR)/100.0);              // Load the value (in percent) that will scale granularity
  keepSession = int(granularity * keepSessionFactor);                   // keepSession - The time to keep a session alive - in seconds
  napDelay = int(granularity * (keepSessionFactor+0.1));                // NapDelay is 10% longer than the keepSession value - in seconds

  int8_t tempTimeZoneOffset = FRAMread8(TIMEZONEADDR);                  // Load Time zone data from FRAM
  Time.zone((float)tempTimeZoneOffset);

  controlRegister = FRAMread8(CONTROLREGISTER);                         // Read the Control Register for system modes so they stick even after reset
  lowPowerMode = (0b00000001 & controlRegister);                        // lowPowerMode
  lowBatteryMode = (0b000000010 & controlRegister);                     // lowBatteryMode
  solarPowerMode = (0b00000100 & controlRegister);                      // solarPowerMode
  verboseMode = (0b00001000 & controlRegister);                         // verboseMode

  PMICreset();                                                          // Executes commands that set up the PMIC for Solar charging - once we know the Solar Mode

  if (!digitalRead(userSwitch) && lowPowerMode) {                      // Rescue mode to locally take lowPowerMode so you can connect to device
    lowPowerMode = false;                                               // Press the user switch while resetting the device
    controlRegister = (0b1111110 & controlRegister);                    // Turn off Low power mode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                        // Write it to the register
  }

  if (!lowPowerMode && !lowBatteryMode && !(Time.hour() >= PARKCLOSES || Time.hour() < PARKOPENS)) connectToParticle();  // If not lowpower or sleeping, we can connect

  takeMeasurements();
  StartStopTest(1);                                                     // Default action is for the test to be running
  timeTillSleep = sleepDelay;                                           // Set initial delay for 60 seconds
  lastEvent = millis();

  if (state != ERROR_STATE) state = IDLE_STATE;                         // IDLE unless error from above code
}

void loop()
{
  switch(state) {
  case IDLE_STATE:
    if(hourlyPersonCountSent) {   // Cleared here as there could be counts coming in while "in Flight"
      hourlyPersonCount -= hourlyPersonCountSent;    // Confirmed that count was recevied - clearing
      FRAMwrite16(CURRENTHOURLYCOUNTADDR, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
      hourlyDurationCount -= hourlyDurationCountSent;    // Confirmed that count was recevied - clearing
      FRAMwrite16(CURRENTHOURLYDURATIONADDR, static_cast<uint16_t>(hourlyDurationCount));  // Load Hourly Duration Count to memory
      hourlyPersonCountSent = hourlyDurationCountSent = 0;
    }
    if (sensorDetect) recordCount();                                                                    // The ISR had raised the sensor flag
    if ((millis() >= (lastEvent + timeTillSleep)) && lowPowerMode) state = NAPPING_STATE;               // Too long since last sensor flag - time to nap
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;                                    // We want to report on the hour but not after bedtime
    if ((Time.hour() >= PARKCLOSES || Time.hour() < PARKOPENS)) state = SLEEPING_STATE;                 // The park is closed, time to sleep
    if (stateOfCharge <= lowBattLimit) LOW_BATTERY_STATE;                                               // The battery is low - sleep
    break;

  case SLEEPING_STATE: {                                        // This state is triggered once the park closes and runs until it opens
    if (!readyForBed)                                           // Only do these things once - at bedtime
    {
      if (hourlyPersonCount) {                                  // If this number is not zero then we need to send this last count
        state = REPORTING_STATE;
        break;
      }
      if (Particle.connected()) {
        disconnectFromParticle();                               // If connected, we need to disconned and power down the modem
      }
      detachInterrupt(intPin);                                  // Done sensing for the day
      dailyPersonCount = 0;                                     // All the counts have been reported so time to zero everything
      FRAMwrite16(CURRENTDAILYCOUNTADDR, 0);                    // Reset the counts in FRAM as well
      resetCount = 0;
      FRAMwrite8(RESETCOUNT,resetCount);
      hourlyPersonCount = 0;
      FRAMwrite16(CURRENTHOURLYCOUNTADDR, 0);
      hourlyDurationCount = 0;
      FRAMwrite16(CURRENTHOURLYDURATIONADDR, 0);
      ledState = false;
      digitalWrite(blueLED,LOW);                                // Turn off the LED
      digitalWrite(tmp36Shutdwn, LOW);                          // Turns off the temp sensor
      digitalWrite(donePin,HIGH);
      digitalWrite(donePin,LOW);                                // Pet the watchdog
      readyForBed = true;                                       // Set the flag for the night
    }
    int secondsToHour = (60*(60 - Time.minute()));              // Time till the top of the hour
    System.sleep(SLEEP_MODE_DEEP,secondsToHour);                // Very deep sleep till the next hour - then resets
    } break;

  case NAPPING_STATE: {
      if (Particle.connected())
      {
        disconnectFromParticle();                              // If connected, we need to disconned and power down the modem
      }
      timeTillSleep = napDelay;                                // Set the time we will wait before napping again
      lastEvent = millis();                                    // Reset millis so we don't wake and then nap again
      ledState = false;                                        // Turn out the light
      digitalWrite(blueLED,LOW);                               // Turn off the LED
      sensorDetect = true;                                     // Woke up so there must have been an event
      int secondsToHour = (60*(60 - Time.minute()));           // Time till the top of the hour
      System.sleep(intPin, RISING, secondsToHour);             // Sensor will wake us with an interrupt
      state = IDLE_STATE;                                      // Back to the IDLE_STATE after a nap
    } break;

  case LOW_BATTERY_STATE: {
      if (Particle.connected()) {
        disconnectFromParticle();                               // If connected, we need to disconned and power down the modem
      }
      detachInterrupt(intPin);                                  // Done sensing for the day
      ledState = false;
      digitalWrite(blueLED,LOW);                                // Turn off the LED
      digitalWrite(tmp36Shutdwn, LOW);                          // Turns off the temp sensor
      digitalWrite(donePin,HIGH);
      digitalWrite(donePin,LOW);                                // Pet the watchdog
      int secondsToHour = (60*(60 - Time.minute()));            // Time till the top of the hour
      System.sleep(SLEEP_MODE_DEEP,secondsToHour);              // Very deep sleep till the next hour - then resets
    } break;

  case REPORTING_STATE: {
    timeTillSleep = sleepDelay;                              // Sets the sleep delay to give time to flash if needed
    if (!Particle.connected()) {
      if (!connectToParticle()) {
        state = ERROR_STATE;
        break;
      }
    }
    takeMeasurements();
    if (Status[0] != '\0')
    {
      if (verboseMode) Particle.publish("Status",Status);  // Will continue - even if not connected.
      Status[0] = '\0';
    }
    LogHourlyEvent();
    sendEvent();
    publishTimeStamp = millis();
    digitalWrite(donePin,HIGH);
    digitalWrite(donePin,LOW);                          // Pet the watchdog once an hour
    if (verboseMode) Particle.publish("State","Waiting for Response");
    state = RESP_WAIT_STATE;                            // Wait for Response
    } break;

  case RESP_WAIT_STATE:
    if (!dataInFlight)                                  // Response received
    {
      state = IDLE_STATE;
      if (verboseMode) Particle.publish("State","Idle");
    }
    else if (millis() >= (publishTimeStamp + webhookWaitTime)) {
      state = ERROR_STATE;  // Response timed out
      Particle.publish("State","Response Timeout Error");
    }
    break;

  case ERROR_STATE:                                          // To be enhanced - where we deal with errors
    if (!waiting)                                            // Will use this flag to wait 30 seconds before reset
    {
      waiting = true;
      resetWaitTimeStamp = millis();
    }
    if (millis() >= (resetWaitTimeStamp + resetWaitTime))
    {
      if (resetCount <= 3)  System.reset();                 // Today, only way out is reset
      else {
        FRAMwrite8(RESETCOUNT,0);                           // Time for a hard reset
        digitalWrite(hardResetPin,HIGH);                    // Zero the count so only every three
      }
    }
    break;
  }
}

void recordCount()                                          // Handles counting when the sensor triggers
{
  char data[256];                                           // Store the date in this character array - not global
  sensorDetect = false;                                     // Reset the flag
  if (millis() - lastEvent > keepSession*1000) {            // Check to see if this is a new session or just a keep session event
    t = Time.now();
    lastEvent = millis();                                   // Each time this routine is triggered, it is an event which keeps the session alive
    hourlyPersonCount++;                                    // Increment the PersonCount
    hourlyDurationCount++;                                  // Increment this as well since we don't count the last one before napping
    FRAMwrite16(CURRENTHOURLYCOUNTADDR, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
    hourlyDurationCount++;                                  // Increment the duration counter as well
    FRAMwrite16(CURRENTHOURLYDURATIONADDR, static_cast<uint16_t>(hourlyDurationCount));  // Load Hourly Count to memory
    dailyPersonCount++;                                     // Increment the PersonCount
    FRAMwrite16(CURRENTDAILYCOUNTADDR, static_cast<uint16_t>(dailyPersonCount));   // Load Daily Count to memory
    FRAMwrite32(CURRENTCOUNTSTIME, t);                      // Write to FRAM - this is so we know when the last counts were saved
    ledState = !ledState;                                   // toggle the status of the LEDPIN:
    digitalWrite(blueLED, ledState);                        // update the LED pin itself
    snprintf(data, sizeof(data), "New visit, hourlry count: %i",hourlyPersonCount);
    if (verboseMode) Particle.publish("Count",data);
  }
  else if (millis() - lastEvent > granularity*1000) {      // In this case, it is the same person who is loitering in the detection area
    lastEvent = millis();                                  // Each time this routine is triggered, it is an event which keeps the session alive
    hourlyDurationCount++;                                 // Increment the duration counter only
    FRAMwrite16(CURRENTHOURLYDURATIONADDR, static_cast<uint16_t>(hourlyDurationCount));  // Load Hourly Count to memory
    averageHourlyDuration = int((hourlyDurationCount * granularity) / hourlyPersonCount);
    snprintf(data, sizeof(data), "Same visit, hourly average duration: %i (duration / person) (%i / %i)",averageHourlyDuration,hourlyDurationCount,hourlyPersonCount);
    if (verboseMode) Particle.publish("Count",data);
  }
  if (!digitalRead(userSwitch)) {                         // A low value means someone is pushing this button - will trigger a send to Ubidots and take out of low power mode
    if (lowPowerMode) {
      Particle.publish("Mode","Normal Operations");
      controlRegister = (0b1111110 & controlRegister);     // Will set the lowPowerMode bit to zero
      lowPowerMode = false;
    }
    state = REPORTING_STATE;                              // If so, connect and send data - this let's us interact with the device if needed
  }

}

void StartStopTest(boolean startTest)  // Since the test can be started from the serial menu or the Simblee - created a function
{
 if (startTest) {
     currentHourlyPeriod = Time.hour();   // Sets the hour period for when the count starts (see #defines)
     currentDailyPeriod = Time.day();     // And the day  (see #defines)
     // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
     time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);
     lastHour = Time.hour(unixTime);
     lastDate = Time.day(unixTime);
     dailyPersonCount = FRAMread16(CURRENTDAILYCOUNTADDR);  // Load Daily Count from memory
     hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNTADDR);  // Load Hourly Count from memory
     hourlyDurationCount = FRAMread16(CURRENTHOURLYDURATIONADDR);  // Load Hourly Duration Count from memory
     if (currentHourlyPeriod != lastHour) LogHourlyEvent();
 }
 else {
     t = Time.now();
     FRAMwrite16(CURRENTDAILYCOUNTADDR, static_cast<uint16_t>(dailyPersonCount));   // Load Daily Count to memory
     FRAMwrite16(CURRENTHOURLYCOUNTADDR, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
     FRAMwrite16(CURRENTHOURLYDURATIONADDR, static_cast<uint16_t>(hourlyDurationCount));  // Load Hourly Duration Count to memory
     FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
     hourlyPersonCount = 0;        // Reset Person Count
     dailyPersonCount = 0;         // Reset Person Count
 }
}

void LogHourlyEvent() // Log Hourly Event()
{
  time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);     // This is the last event recorded - this sets the hourly period
  unsigned int pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
  LogTime -= (60*Time.minute(LogTime) + Time.second(LogTime)); // So, we need to subtract the minutes and seconds needed to take to the top of the hour
  FRAMwrite32(pointer, LogTime);   // Write to FRAM - this is the end of the period
  FRAMwrite16(pointer+HOURLYCOUNTOFFSET,static_cast<uint16_t>(hourlyPersonCount));
  FRAMwrite8(pointer+HOURLYBATTOFFSET,stateOfCharge);
  unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
  FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
}

void sendEvent()
{
  char data[256];                                         // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"avgduration\":%i, \"daily\":%i,\"battery\":%i, \"temp\":%i, \"resets\":%i}",hourlyPersonCount, averageHourlyDuration, dailyPersonCount, stateOfCharge, temperatureF,resetCount);
  Particle.publish("Occupancy_Hook", data, PRIVATE);
  hourlyPersonCountSent = hourlyPersonCount; // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
  hourlyDurationCountSent = hourlyDurationCount;
  currentHourlyPeriod = Time.hour();  // Change the time period
  dataInFlight = true; // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data)  // Looks at the response from Ubidots - Will reset Photon if no successful response
{
  // Response Template: "{{hourly.0.status_code}}"
  if (!data) {                                            // First check to see if there is any data
    Particle.publish("Ubidots Hook", "No Data");
    return;
  }
  int responseCode = atoi(data);                          // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    Particle.publish("State","Response Received");
    dataInFlight = false;                                 // Data has been received
  }
  else Particle.publish("Ubidots Hook", data);             // Publish the response code
}

void getSignalStrength()
{
    CellularSignal sig = Cellular.RSSI();  // Prototype for Cellular Signal Montoring
    int rssi = sig.rssi;
    int strength = map(rssi, -131, -51, 0, 5);
    snprintf(Signal,17, "%s: %d", levels[strength], rssi);
}

int getTemperature()
{
  int reading = analogRead(tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);  // now convert to Fahrenheit
  return temperatureF;
}

void sensorISR()
{
  sensorDetect = true;                                    // sets the sensor flag for the main loop
}

void watchdogISR()
{
  digitalWrite(donePin, HIGH);                              // Pet the watchdog
  digitalWrite(donePin, LOW);
}

bool connectToParticle()
{
  if (!Cellular.ready())
  {
    Cellular.on();                                           // turn on the Modem
    Cellular.connect();                                      // Connect to the cellular network
    if(!waitFor(Cellular.ready,90000)) return false;         // Connect to cellular - give it 90 seconds
  }
  Particle.process();
  Particle.connect();                                      // Connect to Particle
  if(!waitFor(Particle.connected,30000)) return false;     // Connect to Particle - give it 30 seconds
  Particle.process();
  return true;
}

bool disconnectFromParticle()
{
  Particle.disconnect();                                   // Disconnect from Particle in prep for sleep
  waitFor(notConnected,10000);
  Cellular.disconnect();                                   // Disconnect from the cellular network
  delay(3000);
  Cellular.off();                                           // Turn off the cellular modem
  return true;
}

bool notConnected() {
  return !Particle.connected();                             // This is a requirement to use waitFor
}

void takeMeasurements() {
  if (Cellular.ready()) getSignalStrength();                // Test signal strength if the cellular modem is on and ready
  getTemperature();                                         // Get Temperature at startup as well
  stateOfCharge = int(batteryMonitor.getSoC());             // Percentage of full charge
}

void PMICreset() {
  power.begin();                                            // Settings for Solar powered power management
  power.disableWatchdog();
  if (solarPowerMode) {
    power.setInputVoltageLimit(4840);                       // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    power.setInputCurrentLimit(900);                        // default is 900mA
    power.setChargeCurrent(0,0,1,0,0,0);                    // default is 512mA matches my 3W panel
    power.setChargeVoltage(4208);                           // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    power.setInputVoltageLimit(4208);                       // This is the default value for the Electron
    power.setInputCurrentLimit(1500);                       // default is 900mA this let's me charge faster
    power.setChargeCurrent(0,1,1,0,0,0);                    // default is 2048mA (011000) = 512mA+1024mA+512mA)
    power.setChargeVoltage(4112);                           // default is 4.112V termination voltage
  }
}


/* These are the particle functions that allow you to configure and run the device
 * They are intended to allow for customization and control during installations
 * and to allow for management.
*/

int resetFRAM(String command)   // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)   // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    FRAMwrite16(CURRENTDAILYCOUNTADDR, 0);   // Reset Daily Count in memory
    FRAMwrite16(CURRENTHOURLYCOUNTADDR, 0);  // Reset Hourly Count in memory
    FRAMwrite16(CURRENTHOURLYDURATIONADDR, 0);  // Reset Hourly Duration Count in memory
    FRAMwrite8(RESETCOUNT,0);          // If so, store incremented number - watchdog must have done This
    resetCount = 0;
    hourlyPersonCount = hourlyDurationCount = 0;                    // Reset count variables
    dailyPersonCount = 0;
    hourlyPersonCountSent = hourlyDurationCountSent = 0;                // In the off-chance there is data in flight
    dataInFlight = false;
    return 1;
  }
  else return 0;
}

int resetNow(String command)   // Will reset the Electron
{
  if (command == "1")
  {
    System.reset();                           // This will reset the Electron Only
    return 1;                                 // Unfortunately, this will never be sent
  }
  else return 0;
}

int hardResetNow(String command)   // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(hardResetPin,HIGH);          // This will cut all power to the Electron AND the carrir board
    return 1;                                 // Unfortunately, this will never be sent
  }
  else return 0;
}

int setGranularity(String command)  // Will accept a new granularity value in the form "xxx" where xxx is an integer for delay in seconds
{
  char * pEND;
  char data[256];
  byte tempGranularity = strtol(command,&pEND,10);                      // Looks for the first integer and interprets it
  if ((tempGranularity < 0) | (tempGranularity > 255)) return 0;        // Make sure it falls in a valid range or send a "fail" result
  granularity = tempGranularity;                                        // Valid value
  FRAMwrite8(GRANULARITYADDR, granularity);                             // Remember we store granularity in Sec
  keepSession = int(granularity * keepSessionFactor);                   // keepSession - The time to keep a session alive - in seconds
  napDelay = int(granularity * (keepSessionFactor + 0.1));              // These two are related
  snprintf(data, sizeof(data), "Values are: granularity: %i, keepSessionFactor: %.1f, keepSession: %i, napDelay: %i",granularity,keepSessionFactor,keepSession,napDelay);
  if (verboseMode) Particle.publish("Variables",data);
  return 1;
}

int setKeepSession(String command)  // Will accept a new keep session value in the form "xx" where xxx is the amount in % that keep alive is greater than granularity
{
  char * pEND;
  char data[256];
  byte tempKeepSessionPercent = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempKeepSessionPercent < 0) | (tempKeepSessionPercent > 200)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  FRAMwrite8(KEEPSESSIONADDR,tempKeepSessionPercent);
  keepSessionFactor = 1.0 + ((float)tempKeepSessionPercent/100.0);
  keepSession = int(granularity * keepSessionFactor);                   // keepSession - The time to keep a session alive - in seconds
  napDelay = int(granularity * (keepSessionFactor+0.1));                // NapDelay is 10% longer than the keepSession value - in seconds
  snprintf(data, sizeof(data), "Values are: granularity: %i, keepSessionFactor: %.2f, keepSession: %i, napDelay: %i",granularity,keepSessionFactor,keepSession,napDelay);
  if (verboseMode) Particle.publish("Variables",data);
  return 1;
}


int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    solarPowerMode = true;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b00000100 | controlRegister);          // Turn on solarPowerMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);               // Write it to the register
    PMICreset();                                               // Change the power management Settings
    Particle.publish("Mode","Set Solar Powered Mode");
    return 1;
  }
  else if (command == "0")
  {
    solarPowerMode = false;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b11111011 & controlRegister);           // Turn off solarPowerMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                // Write it to the register
    PMICreset();                                                // Change the power management settings
    Particle.publish("Mode","Cleared Solar Powered Mode");
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b00001000 | controlRegister);                    // Turn on verboseMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                        // Write it to the register
    Particle.publish("Mode","Set Verbose Mode");
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    FRAMread8(CONTROLREGISTER);
    controlRegister = (0b11110111 & controlRegister);                    // Turn off verboseMode
    FRAMwrite8(CONTROLREGISTER,controlRegister);                        // Write it to the register
    Particle.publish("Mode","Cleared Verbose Mode");
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  FRAMwrite8(TIMEZONEADDR,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  t = Time.now();
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  Particle.publish("Time",data);
  delay(1000);
  Particle.publish("Time",Time.timeStr(t));
  return 1;
}

int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  controlRegister = FRAMread8(CONTROLREGISTER);                       // Get the control register (generla approach)
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    Particle.publish("Mode","Low Power");
    controlRegister = (0b00000001 | controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    Particle.publish("Mode","Normal Operations");
    controlRegister = (0b1111110 & controlRegister);                  // If so, flip the lowPowerMode bit
    lowPowerMode = false;
  }
  FRAMwrite8(CONTROLREGISTER,controlRegister);                         // Write to the control register
  return 1;
}
