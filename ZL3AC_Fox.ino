// ZL3AC FOX
// ---------
//
// Compile with the following Arduino IDE settings:
//
// Board: Generic STM32F1 series
// Board Part Number: Blue Pill F103C8
// U(S)ART support: Enable (generic 'Serial')
// USB Support: None


#include <TinyGPS.h>
#include <Timezone.h>   // https://github.com/JChristensen/Timezone
#include <TimeLib.h>    // https://github.com/PaulStoffregen/Time
#include <SPI.h>
#include <SdFat.h>
#include "stdlib.h" //for atoi

// Setup audio file playback variables
// Chip select PA4, shared SPI, 18 MHz, port 1.
#define SD1_CONFIG SdSpiConfig(PA4, DEDICATED_SPI, SD_SCK_MHZ(18), &SPI)
SdFat sd;
File fileHandle;
#define BUFFERSIZE 2048

//This is the amount of data to be fetched from the SD card for each read.
#define BUFFERSIZE 2048
char buffer0[BUFFERSIZE], buffer1[BUFFERSIZE];  //Two cycling buffers which contain the WAV data.

volatile uint8_t note=0;    //Holds the current voltage value to be sent.
uint8_t play_buffer=0;         //Keeps track of which buffer is currently being used
uint8_t new_buffer_ready=0;      //Flag used by 'Loop' code to tell the Interrupt that new data is ready in the buffer.
volatile uint16_t byte_count=0; //Keeps track of the number of bytes read from the current buffer.
volatile uint8_t need_new_data=0;    //Flag used by Interrupt to tell 'Loop' code that a buffer is empty and needs to be refilled.
volatile uint8_t playing=0;  //Flag to see when we need to finish playing a file.

TIM_TypeDef *PWMInstance = TIM2; //timer for PWM frequency
HardwareTimer *PWMTimer = new HardwareTimer(PWMInstance);
uint32_t PWMchannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PA0), PinMap_PWM));

TIM_TypeDef *SampleInstance = TIM3; //timer for sample frequency
HardwareTimer *SampleTimer = new HardwareTimer(SampleInstance);

/* GPS setup, change TinyGPS to recognise GNRMC */
TinyGPS gps;
HardwareSerial SerialGPS(USART2);
#define GPSEnable PB0

/* New Zealand Time Zone setup*/
TimeChangeRule nzSTD = {"NZST", First, Sun, Apr, 3, 720};   // UTC + 12 hours
TimeChangeRule nzDST = {"NZDT", Last, Sun, Sep, 2, 780};    // UTC + 13 hours
Timezone nz(nzDST, nzSTD);
time_t startTime;
uint32_t txInterval = 0;

HardwareSerial SerialRadio(USART3);
#define lowPower PA8
#define PTT PA15
#define powerDown PB15
#define sqIn PB12
char freqBuffer[12];

#define LED PA1

inline void enableGPS(uint8_t en) { digitalWrite(GPSEnable, en); }

inline void disableRadio() { 
  digitalWrite(powerDown, LOW); 
  digitalWrite(PTT, HIGH); // now open drain output
  digitalWrite(lowPower, LOW);
  //SerialRadio.end();  // seems to disable all serial ports
  //Serial.begin(115200); // so start up the ones we need
  //SerialGPS.begin(9600);
  pinMode(PB10, INPUT);
}

inline void enableRadio() { 
  digitalWrite(PTT, HIGH); 
  digitalWrite(powerDown, HIGH);
  digitalWrite(lowPower, HIGH);
  SerialRadio.begin(9600);
}

inline void transmit(uint8_t tx) { digitalWrite(PTT, tx?LOW:HIGH); }

inline void txLowPower(uint8_t power) { digitalWrite(lowPower, power?HIGH:LOW);}

inline void LEDon(uint8_t led) { digitalWrite(LED, led); }

void flashError(uint8_t flashes)
{
  uint8_t n=0;

  while(1) {
    for(n=0;n<flashes;n++){
      LEDon(true);
      delay(150);
      LEDon(false);
      delay(300);
    }
    delay(1000);
  }
}

// This timer interrupt will run every 1/48000Hz. 
// This is frequency of the WAV files that will be played by this sketch.
void UpdateAudioSample(void)
{
  //Check to see if we've read all of the data in the current buffer
  if(byte_count==BUFFERSIZE)
  {
    need_new_data=1;  //Set a flag to tell the 'loop' code to refill the current buffer.
    byte_count=0;   //Reset the count
    //Check to see if new data exists in the alternate buffer
    if(new_buffer_ready==1) {
      //If new data is available, reassign the play buffer.
      if(play_buffer==0)play_buffer=1;
      else play_buffer=0;
    } else {
      //If no new data is available then wait for it!
      Serial.print("UF");
      return;
    }
  }
  
  //Find out which buffer is being used, and get data from it.
  if(play_buffer==0) {
      PWMTimer->setCaptureCompare(PWMchannel, buffer0[byte_count], RESOLUTION_8B_COMPARE_FORMAT);
  }
  else {
    PWMTimer->setCaptureCompare(PWMchannel, buffer1[byte_count], RESOLUTION_8B_COMPARE_FORMAT);
  }
  
  //Increase the byte_count since we've taken the current data.
  byte_count +=1;
}  

void printDateTime(time_t t)
{
    char buf[32];
    char m[4];    // temporary storage for month string (DateStrings.cpp uses shared buffer)
    strcpy(m, monthShortStr(month(t)));
    sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d",
        hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), m, year(t));
    Serial1.println(buf);
}

void setup() {
  // Debug serial port
  Serial.begin(115200);
  Serial.println("****");
  // Radio serial port
  //SerialRadio.begin(9600); run when module enabled
 
  // Set default pin states
  pinMode(GPSEnable, OUTPUT);
  enableGPS(false);
  
  pinMode(powerDown, OUTPUT);
  pinMode(PTT, OUTPUT_OPEN_DRAIN);
  pinMode(lowPower, OUTPUT);
  disableRadio();

  //power up LED notification
  pinMode(LED, OUTPUT);
  LEDon(true);
  delay(1000);
  LEDon(false);
  delay(500);
  LEDon(true);
  delay(1000);
  LEDon(false);
  delay(500);
  
  // Start the CD card
  if (!sd.begin(SD1_CONFIG)) {
    Serial.println("SD initialization failed!");
    flashError(1);
  }
  Serial.println("SD initialization done.");

  // Can we open the audio file
  Serial.print("Opening audio.wav...");
  // open the audio file. note that only one file can be open at a time,
  fileHandle = sd.open("audio.wav", FILE_READ);
  if (!fileHandle) {
    Serial.println("open audio.wav failed!");
    flashError(2);
  }
  Serial.println("audio.wav valid.");
  fileHandle.close();

  // Get the frequency
  fileHandle = sd.open("freq.txt", FILE_READ);
  if (!fileHandle) {
    Serial.println("open freq.txt failed!");
    flashError(3);
  }
  Serial.print("freq.txt opened....");

  fileHandle.read(freqBuffer, 8);
  fileHandle.close();
  freqBuffer[8] = 0;
  if ((freqBuffer[0] != '1') ||
      (freqBuffer[1] != '4') ||
      (freqBuffer[3] != '.'))  flashError(4);
  Serial.print("freq.txt parsed - ");
  Serial.println(freqBuffer);

  // Get start time
  fileHandle = sd.open("start.txt", FILE_READ);
  if (!fileHandle) {
    Serial.println("open start.txt failed!");
    flashError(4);
  }
  Serial.print("start.txt opened....");

  fileHandle.read(buffer0, 64);
  fileHandle.close();
  
  tmElements_t tm;
  tm.Year = atoi(buffer0) - 1970;
  tm.Month = atoi(buffer0+5);
  tm.Day = atoi(buffer0+8);
  tm.Hour = atoi(buffer0+11);
  tm.Minute = atoi(buffer0+14);
  tm.Second = atoi(buffer0+17);
  startTime = makeTime(tm);
  Serial.print("start.txt parsed - ");
  printDateTime(startTime);

  // see if tx time is in the past.
  Serial.print("Testing date....");
  enableGPS(true);
  SerialGPS.begin(9600);
  uint8_t dateInvalid = 1;
  while (dateInvalid) {
    while (SerialGPS.available()) {
      char gps_char = SerialGPS.read();
      if (gps_char == '$') Serial.print('.');
      if (gps.encode(gps_char)) { // process gps messages
        // when TinyGPS reports new data...
  
        tmElements_t tm;
        int year;
        gps.crack_datetime(&year, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second, NULL, NULL);
        tm.Year = year - 1970; 
        time_t time = makeTime(tm);
  
        time_t localTime = nz.toLocal(time);
        printDateTime(localTime);
        
        if (localTime > startTime) {
          Serial.println("date invalid.");
          flashError(8);
        } else {
          dateInvalid = 0;
          Serial.println("date valid");
        }
        
        }
    }
  }

  // Get tx interval time
  fileHandle = sd.open("interval.txt", FILE_READ);
  if (!fileHandle) {
    Serial.println("Open interval.txt failed!");
    flashError(4);
  }
  Serial.print("start.txt opened....");

  fileHandle.read(buffer0, 64);
  fileHandle.close();

  txInterval = atoi(buffer0);
  if (txInterval == 0) {
    Serial.println("Could not parse.");
    flashError(5);
  }
  if ( (txInterval < 15) || (txInterval > 300) ) {
    Serial.println("Number to large or small.");
  }
  Serial.print("interval.txt parsed - ");
  Serial.print(txInterval);
  Serial.println(" seconds");
  


  enableRadio();
  Serial.println("Radio enabled");
  delay(1000);

  // set parameters for DRA818V
  SerialRadio.print("AT+DMOSETGROUP=");         // begin message
  SerialRadio.print("1"); // bandwith in KHz ( 0= 12.5KHz or 1= 25KHz )
  SerialRadio.print(",");
  SerialRadio.print(freqBuffer);
  SerialRadio.print(",");
  SerialRadio.print(freqBuffer);
  SerialRadio.print(",");
  SerialRadio.print("0000"); // ctcss frequency ( 0000 - 0038 ); 0000 = "no CTCSS"
  SerialRadio.print(",");
  SerialRadio.print("0"); //squelch level 0 (0-8)
  SerialRadio.print(",");
  SerialRadio.println("0000"); // ctcss frequency ( 0000 - 0038 ); 0000 = "no CTCSS"
  
  long start = millis();
  do  {
    if (SerialRadio.available()) {
      Serial.write (SerialRadio.read());
    } 
  }while ( (millis() - start)  < 2000);

  Serial.println("Removing tail tone.");
  SerialRadio.println("AT+SETTAIL=0");
  start = millis();
  do  {
    if (SerialRadio.available()) {
      Serial.write (SerialRadio.read());
    } 
  }while ( (millis() - start)  < 2000);
  
  Serial.println("Radio programmed");

  fileHandle = sd.open("audio.wav", FILE_READ);
  if (!fileHandle) {
    flashError(5);
  }
  Serial.println("Audio file opened for playback.");
  delay(100);
  
  //Setup PWM timer
  PWMTimer->setPWM(PWMchannel, PA0, 192000, 50); // 384kHz Hertz, 50% dutycycle
  Serial.println("PWM timer setup.");
  delay(100);

  SampleTimer->setOverflow(48000, HERTZ_FORMAT); // 48kHz smaple rate
  SampleTimer->attachInterrupt(UpdateAudioSample);
  SampleTimer->pause();
  Serial.println("Sample timer setup.");
  delay(100);

  Serial.println("Reading header...");
  delay(100);
  //Read the header information. Alternate purpose is to get to the DATA offset of the file.
  fileHandle.read(buffer1, 44);
  Serial.println("Filling buffers...");
  delay(100);  
  //Set the initial play buffer, and grab the initial data from the SD card.
  play_buffer=0;
  fileHandle.read(buffer0, BUFFERSIZE);
  fileHandle.read(buffer1, BUFFERSIZE);
  new_buffer_ready=1;

  Serial.println("Starting Playback.");
  transmit(true);
  delay(1200); //module takes time to TX
  
  //Enable interrupts to start the wav playback.
  SampleTimer->resume();
  playing = 1;
  while(playing){
    if(need_new_data==1)  //need_new_data flag is set by ISR to indicate a buffer is empty and should be refilled
    {
      need_new_data=0;  //Clear the flag.
      if(play_buffer==0)  //play_buffer indicates which buffer is playing
      {
        //Get the next BUFFERSIZE bytes from the file.
        fileHandle.read(buffer1, BUFFERSIZE);
      }
      else
      {
        //Get the next BUFFERSIZE bytes from the file.
        fileHandle.read(buffer0, BUFFERSIZE);
      }
      new_buffer_ready=1; //new_buffer_ready flag tells the ISR that the buffer has been filled.
      
      //If file_read returns 0 or -1 file is over.
      if(!fileHandle.available())
      {
        playing = 0; //we have finished
        transmit(false);
        //PWMTimer->setCaptureCompare(PWMchannel, 50, PERCENT_COMPARE_FORMAT);
        PWMTimer->pause();
        SampleTimer->pause(); //Disable interrupts to stop playback.
        fileHandle.close();
        Serial.println("Finished playing.");
      }
    }
  }
  delay(100); //spit out serial before end()
  //disableRadio();
  
  
}


void loop() {
  char gps_char;
  
  while (SerialGPS.available()) {
    gps_char = SerialGPS.read();
    //Serial.print(gps_char);
    if (gps.encode(gps_char)) { // process gps messages
      // when TinyGPS reports new data...

      tmElements_t tm;
      int year;
      gps.crack_datetime(&year, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second, NULL, NULL);
      tm.Year = year - 1970; 
      time_t time = makeTime(tm);

      time_t localTime = nz.toLocal(time);
      printDateTime(localTime);
      
      if (localTime >= startTime) {
        fileHandle = sd.open("audio.wav", FILE_READ);
        if (!fileHandle) {
          flashError(5);
        }
        Serial.println("Audio file opened for playback.");
        delay(100);
        
        
        Serial.println("Reading header...");
        delay(100);
        //Read the header information. Alternate purpose is to get to the DATA offset of the file.
        fileHandle.read(buffer1, 44);
        Serial.println("Filling buffers...");
        delay(100);  
        //Set the initial play buffer, and grab the initial data from the SD card.
        play_buffer=0;
        fileHandle.read(buffer0, BUFFERSIZE);
        fileHandle.read(buffer1, BUFFERSIZE);
        new_buffer_ready=1;
      
        Serial.println("Starting Playback.");
        transmit(true);
        delay(1200); //module takes time to TX
        
        //Enable interrupts to start the wav playback.
        PWMTimer->resume();
        SampleTimer->resume();
        playing = 1;
        while(playing){
          if(need_new_data==1)  //need_new_data flag is set by ISR to indicate a buffer is empty and should be refilled
          {
            need_new_data=0;  //Clear the flag.
            if(play_buffer==0)  //play_buffer indicates which buffer is playing
            {
              //Get the next BUFFERSIZE bytes from the file.
              fileHandle.read(buffer1, BUFFERSIZE);
            }
            else
            {
              //Get the next BUFFERSIZE bytes from the file.
              fileHandle.read(buffer0, BUFFERSIZE);
            }
            new_buffer_ready=1; //new_buffer_ready flag tells the ISR that the buffer has been filled.
            
            //If file_read returns 0 or -1 file is over.
            if(!fileHandle.available())
            {
              playing = 0; //we have finished
              transmit(false);
              //PWMTimer->setCaptureCompare(PWMchannel, 50, PERCENT_COMPARE_FORMAT);
              PWMTimer->pause();
              SampleTimer->pause(); //Disable interrupts to stop playback.
              fileHandle.close();
              Serial.println("Finished playing.");
            }
          }
        }        



        startTime += txInterval;
      }
      
      }
  }
}
