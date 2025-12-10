#include "SdFat.h"
#include "FreeStack.h"
#include "ExFatLogger_20X.h"
#include "RTClib.h"
#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

QWIICMUX myMux;
RTC_PCF8523 rtc;
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire, -1, 400000, 400000);

#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5
  
int NUM_BOARDS = 5;
int port_order[] = {0,2,3,5,7}; // 0,2,3,5,7
boolean BUTTONA_FLAG = false;
boolean BUTTONB_FLAG = false;
boolean BUTTONC_FLAG = false;

DateTime now;
int inyear, inmonth, inday;
int inhour, inminute, insecond;
// You may modify the log file name up to 40 characters.
// Digits before the dot are file versions, don't edit them!
char binName[] = "GloveSDandUSB00.txt";

//------------------------------------------------------------------------------
// This example was designed for exFAT but will support FAT16/FAT32.
// Note: Uno will not support SD_FAT_TYPE = 3.
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 2
#if SD_FAT_TYPE == 0
typedef SdFat sd_t;
typedef File file_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

//------------------------------------------------------------------------------
// Interval between data records in microseconds.
const uint16_t LOG_INTERVAL_USEC = 10000; //8ms             
// Use to compare timestamps for missed packets
const uint16_t MAX_INTERVAL_USEC = 15000; //10ms
//------------------------------------------------------------------------------

// Initial time before logging starts, set once logging has begun
// And total log time of session, used to print to csv file once
// converted.
uint32_t t0, log_time;
// Init the time delta to track missed packets
uint32_t delta = 0;


// LED to light if overruns occur, define if you have one setup
#define ERROR_LED_PIN -1
//------------------------------------------------------------------------------
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
// Teensy boards have pre-defined SS = BUILTIN_SDCARD = 254
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = 10;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
//------------------------------------------------------------------------------
// FIFO SIZE - 512 byte sectors.  Modify for your board.

#define FIFO_SIZE_SECTORS 16

//------------------------------------------------------------------------------
// Preallocate 1GiB file.
const uint32_t PREALLOCATE_SIZE_MiB = 1024UL;
// Conversion to 64b variable to match the library param
const uint64_t PREALLOCATE_SIZE = (uint64_t)PREALLOCATE_SIZE_MiB << 20;
//------------------------------------------------------------------------------
// Select the appropriate SPI configuration. 
// ENABLE_DEDICATED_SPI default is true for Teensy boards, change this in
// SdFatConfig.h to zero if you want a (slower) shared SPI bus.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(12))
#else  // ENABLE_DEDICATED_SPI
// Shared SPI bus, MAY need to alter the 50MHz depending on if it's already declared
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12))
#endif  // ENABLE_DEDICATED_SPI
//------------------------------------------------------------------------------

// Max length of file name including zero byte.
#define FILE_NAME_DIM 40

// Max number of records to buffer while SD is busy. Should alter factors to result
// in an integer! (Faster writes)
const size_t FIFO_DIM = 512 * FIFO_SIZE_SECTORS / sizeof(data_t);
int SWITCH_PIN = 6;
int SWITCH_VALUE;
int rtc_start, rtc_stop;

// Create single sd type
sd_t sd;

// Create two filetypes
file_t binFile;
file_t csvFile;

// Boolean used to track whether or not you're just testing the sensors. Won't print
// the "Missed packet(s)" everytime when testing, otherwise printed in data logging.
bool test = false;
unsigned long startTime, stopTime;

//==============================================================================
// Replace logRecord(), printRecord(), and ExFatLogger.h for your sensors.
// updates values in our struct, writes to serial, and returns data for SD write
void logRecord(data_t* data) {
  
  data->t = (micros() - t0);
  now = rtc.now();
  data->rtcyear = now.year();
  data->rtcmonth = now.month();
  data->rtcday = now.day();
  data->rtchour = now.hour();
  data->rtcminute = now.minute();
  data->rtcsecond = now.second();

  myMux.setPort(0);
  delay(0.1);
  mlx0.readBurstData(data->data0);
  mlx1.readBurstData(data->data1);
  mlx2.readBurstData(data->data2);
  mlx3.readBurstData(data->data3);
  
  myMux.setPort(2);
  delay(0.1);
  mlx0.readBurstData(data->data5);
  mlx1.readBurstData(data->data6);
  mlx2.readBurstData(data->data7);
  mlx3.readBurstData(data->data8);
  
  myMux.setPort(3);
  mlx0.readBurstData(data->data10); 
  mlx1.readBurstData(data->data11);
  mlx2.readBurstData(data->data12);
  mlx3.readBurstData(data->data13);
  
  myMux.setPort(5);
  mlx0.readBurstData(data->data15);
  mlx1.readBurstData(data->data16);
  mlx2.readBurstData(data->data17); 
  mlx3.readBurstData(data->data18);
  
  myMux.setPort(7);
  mlx0.readBurstData(data->data20);
  mlx1.readBurstData(data->data21);
  mlx2.readBurstData(data->data22);
  mlx3.readBurstData(data->data23);

//  Serial.write((byte*)&data, sizeof(data_t)); 
//  Serial.write("\n");

}

void printHeader(Print* pr){
  uint8_t gain_sel;
  uint8_t hall_conf;
  uint8_t res_x, res_y, res_z;
  uint8_t tcmp_en;

  //this assumes all chips take the same setup inputs
  myMux.setPort(0);
  delay(5);
  mlx0.getGainSel(gain_sel);
  mlx0.getHallConf(hall_conf);
  mlx0.getResolution(res_x, res_y, res_z);
  mlx0.getTemperatureCompensation(tcmp_en);

  pr->print(gain_sel); pr->print(" "); 
  pr->print(hall_conf); pr->print(" "); 
  pr->print(res_x); pr->print(" "); 
  pr->print(res_y); pr->print(" "); 
  pr->print(res_z); pr->print(" "); 
  pr->print(tcmp_en); pr->print(" ");
  pr->print(LOG_INTERVAL_USEC); pr->println();
}
//------------------------------------------------------------------------------
void printRecord(Print* pr, data_t* data, bool test_) {
  static uint32_t nr = 0;

  // Print data header when no data is sent to printRecord()
  if (!data){
    // File info lines
    pr->print(F("LOG INTERVAL,"));
    pr->print(LOG_INTERVAL_USEC);
    pr->print(F(",microseconds"));
    pr->println();
    pr->print(F("TOTAL LOG TIME,"));
    pr->print(log_time);
    pr->print(F(",seconds"));
    pr->println();

    // Dataset Titles
    pr->print(F("TRANSFER #"));
    pr->print(F(",TIME"));
    pr->print(F(",TIME DELTA"));
    pr->print(F(",M1X"));
    pr->print(F(",M1Y"));
    pr->print(F(",M1Z"));
    pr->print(F(",M1T"));
    pr->println();
    nr = 0;
    return;
  }

  // Test if the delta is too high
  if (data->t - delta >= MAX_INTERVAL_USEC && !test_) {
    pr->print(F("Missed Packet(s)\n"));
  }
  
  // Print data packet
  pr->print(nr++);
  pr->write(','); pr->print(data->t);
  pr->write(','); pr->print(data->t-delta);
  pr->write(','); pr->print(data->data0.x);
  pr->write(','); pr->print(data->data0.y);
  pr->write(','); pr->print(data->data0.z);
  pr->write(','); pr->print(data->data0.t);
  pr->write(','); pr->print(data->data1.x);
  pr->write(','); pr->print(data->data1.y);
  pr->write(','); pr->print(data->data1.z);
  pr->write(','); pr->print(data->data1.t);
  pr->write(','); pr->print(data->data2.x);
  pr->write(','); pr->print(data->data2.y);
  pr->write(','); pr->print(data->data2.z);
  pr->write(','); pr->print(data->data2.t);
  pr->write(','); pr->print(data->data3.x);
  pr->write(','); pr->print(data->data3.y);
  pr->write(','); pr->print(data->data3.z);
  pr->write(','); pr->print(data->data3.t);
  pr->println();

  // Reset delta to hold time for the next packet
  delta = data->t;
}
//==============================================================================
//------------------------------------------------------------------------------
#define error(s) sd.errorHalt(&Serial, F(s))
#define dbgAssert(e) ((e) ? (void)0 : error("assert " #e))
//------------------------------------------------------------------------------
// Convert binary file to csv file.
void binaryToCsv() {
  uint8_t lastPct = 0;
  uint32_t start = millis();
  data_t binData[FIFO_DIM];

  if (!binFile.seekSet(512)) {
    error("binFile.seek failed");
  }
  uint32_t tPct = millis();
  
  // Prints header for csv file
  printRecord(&csvFile, nullptr, test);

  // Loop runs until user types a character or
  // once the conversion is complete
  while (!Serial.available() && binFile.available()) {
    
    // Returns the number of bytes read in binData
    int nb = binFile.read(binData, sizeof(binData));
    if (nb <= 0 ) {
      error("read binFile failed");
    }
    // nr is the number of data_t instances logged
    size_t nr = nb/sizeof(data_t);
    for (size_t i = 0; i < nr; i++) {
      printRecord(&csvFile, &binData[i], test);
    }

    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition()/(binFile.fileSize()/100);
      if (pct != lastPct) {
        tPct = millis();
        lastPct = pct;
        Serial.print(pct, DEC);
        Serial.println('%');
        csvFile.sync();
      }
    }
    if (Serial.available()) {
      break;
    }
  }
  csvFile.close();
  Serial.print(F("Done: "));
  Serial.print(0.001*(millis() - start));
  Serial.println(F(" Seconds"));
}
//-------------------------------------------------------------------------------
void createBinFile() {
  
  binFile.close();

  // Increment file name number from 0 to 99
  while (sd.exists(binName)) {
    char* p = strchr(binName, '.');
    if (!p) {
      error("no dot in filename");
    }
    while (true) {
      p--;
      if (p < binName || *p < '0' || *p > '9') {
        error("Can't create file name");
      }
      if (p[0] != '9') {
        p[0]++;
        break;
      }
      p[0] = '0';
    }
  }
  
  if (!binFile.open(binName, O_RDWR | O_CREAT)) { //O_CREAT | O_WRITE | O_APPEND
    error("open binName failed");
  }

  // Preallocate file size while writing to maximize speed/write consistently
  Serial.println(binName);
  if (!binFile.preAllocate(PREALLOCATE_SIZE)) {
    error("preAllocate failed");
  }

  Serial.print(F("preAllocated: "));
  Serial.print(PREALLOCATE_SIZE_MiB);
  Serial.println(F(" MiB"));
}
//-------------------------------------------------------------------------------
bool createCsvFile() {
  char csvName[FILE_NAME_DIM];
  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return false;
  }

  // Create a new csvFile.
  binFile.getName(csvName, sizeof(csvName));
  char* dot = strchr(csvName, '.');
  if (!dot) {
    error("no dot in filename");
  }
  strcpy(dot + 1, "csv");
  if (!csvFile.open(csvName, O_WRONLY | O_CREAT | O_TRUNC)) {
    error("open csvFile failed");
  }
  serialClearInput();
  Serial.print(F("Writing: "));
  Serial.print(csvName);
  Serial.println(F(" - type any character to stop"));
  return true;
}
//-------------------------------------------------------------------------------
void logData() {
  int32_t delta;  // Jitter in log time.
  int32_t maxDelta = 0;
  uint32_t maxLogMicros = 0;
  uint32_t maxWriteMicros = 0;
  size_t maxFifoUse = 0;
  size_t fifoCount = 0;
  size_t fifoHead = 0;
  size_t fifoTail = 0;
  uint16_t overrun = 0;
  uint16_t maxOverrun = 0;
  uint32_t totalOverrun = 0;
  uint32_t fifoBuf[128*FIFO_SIZE_SECTORS];
  data_t* fifoData = (data_t*)fifoBuf;

  // Write dummy sector to start multi-block write.
  dbgAssert(sizeof(fifoBuf) >= 512);
  memset(fifoBuf, 0, sizeof(fifoBuf));
  if (binFile.write(fifoBuf, 512) != 512) {
    error("write first sector failed");
  }
  
  serialClearInput();
  Serial.println(F("Type any character to stop"));

  // Wait until SD is not busy.
  while (sd.card()->isBusy()) {}
  
  // Start time for log file.
  uint32_t m = millis();

  //---start RTC
  //wait for RTC second to rollover
  now = rtc.now();
  rtc_start = now.second();
  rtc_stop = rtc_start;
  Serial.println(rtc_start);
  while(rtc_start==rtc_stop)
  {
    rtc_stop = rtc.now().second();
  }
  Serial.println(rtc_stop);
  //save RTC time and corresponding t0 time
  //write values to first line of datafile
  //---end RTC
  
  t0 = micros();
  // Time to log next record.
  uint32_t logTime = micros();
  while (true) {
    // Time for next data record.
    logTime += LOG_INTERVAL_USEC;

    // Wait until time to log data.
    delta = micros() - logTime;
    if (delta > 0) {
      Serial.print(F("delta: "));
      Serial.println(delta);
      error("Rate too fast, lower LOG_INTERVAL_USEC");
    }
    while (delta < 0) {
      delta = micros() - logTime;
    }

    if (fifoCount < FIFO_DIM) {
      uint32_t m = micros();
      logRecord(fifoData + fifoHead);
      m = micros() - m;
      if (m > maxLogMicros) {
        maxLogMicros = m;
      }
      fifoHead = fifoHead < (FIFO_DIM - 1) ? fifoHead + 1 : 0;
      fifoCount++;
      if (overrun) {
        if (overrun > maxOverrun) {
          maxOverrun = overrun;
        }
        overrun = 0;
      }
    } else {
      totalOverrun++;
      overrun++;
      if (overrun > 0XFFF) {
        error("too many overruns");
      }
      if (ERROR_LED_PIN >= 0) {
        digitalWrite(ERROR_LED_PIN, HIGH);
      }
    }
    // Save max jitter.
    if (delta > maxDelta) {
      maxDelta = delta;
    }
    // Write data if SD is not busy.
    if (!sd.card()->isBusy()) {
      size_t nw = fifoHead > fifoTail ? fifoCount : FIFO_DIM - fifoTail;
      // Limit write time by not writing more than 512 bytes.
      const size_t MAX_WRITE = 512/sizeof(data_t);
      if (nw > MAX_WRITE) nw = MAX_WRITE;
      size_t nb = nw*sizeof(data_t);
      uint32_t usec = micros();
      if (nb != binFile.write(fifoData + fifoTail, nb)) {
        error("write binFile failed");
      }
      usec = micros() - usec;
      if (usec > maxWriteMicros) {
        maxWriteMicros = usec;
      }
      fifoTail = (fifoTail + nw) < FIFO_DIM ? fifoTail + nw : 0;
      if (fifoCount > maxFifoUse) {
        maxFifoUse = fifoCount;
      }
      fifoCount -= nw;
      if (Serial.available()) {
        break;
      }
      if(!digitalRead(BUTTON_A)){
        //BUTTONA_FLAG = !(BUTTONA_FLAG);
        //
        break;
      }
    }
  }
  // Compute total log time in seconds
  log_time = 0.001 * (millis() - m);
  displayStartScreen();
  Serial.print(F("\nLog time: "));
  Serial.print(log_time);
  Serial.println(F(" Seconds"));
  binFile.truncate();
  binFile.sync();
  Serial.print(("File size: "));
  // Warning cast used for print since fileSize is uint64_t.
  Serial.print((uint32_t)binFile.fileSize());
  Serial.println(F(" bytes"));
  Serial.print(F("totalOverrun: "));
  Serial.println(totalOverrun);
  Serial.print(F("FIFO_DIM: "));
  Serial.println(FIFO_DIM);
  Serial.print(F("maxFifoUse: "));
  Serial.println(maxFifoUse);
  Serial.print(F("maxLogMicros: "));
  Serial.println(maxLogMicros);
  Serial.print(F("maxWriteMicros: "));
  Serial.println(maxWriteMicros);
  Serial.print(F("Log interval: "));
  Serial.print(LOG_INTERVAL_USEC);
  Serial.print(F(" micros\nmaxDelta: "));
  Serial.print(maxDelta);
  Serial.println(F(" micros"));
}
//------------------------------------------------------------------------------
void openBinFile() {
  char name[FILE_NAME_DIM];
  serialClearInput();
  Serial.println(F("Enter file name"));
  if (!serialReadLine(name, sizeof(name))) {
    return;
  }
  if (!sd.exists(name)) {
    Serial.println(name);
    Serial.println(F("File does not exist"));
    return;
  }
  binFile.close();
  if (!binFile.open(name, O_RDONLY)) {
    Serial.println(name);
    Serial.println(F("open failed"));
    return;
  }
  Serial.println(F("File opened"));
}
//-----------------------------------------------------------------------------
void printData() {
  if (!binFile.isOpen()) {
    Serial.println(F("No current binary file"));
    return;
  }
  // Skip first dummy sector.
  if (!binFile.seekSet(512)) {
    error("seek failed");
  }
  serialClearInput();
  Serial.println(F("type any character to stop\n"));
  delay(1000);
  printRecord(&Serial, nullptr, test);
  while (binFile.available() && !Serial.available()) {
    data_t record;
    if (binFile.read(&record, sizeof(data_t)) != sizeof(data_t)) {
      error("read binFile failed");
    }
    printRecord(&Serial, &record, test);
  }
}
//------------------------------------------------------------------------------
void printUnusedStack() {
#if HAS_UNUSED_STACK  
  Serial.print(F("\nUnused stack: "));
  Serial.println(UnusedStack());
#endif  // HAS_UNUSED_STACK 
}
//------------------------------------------------------------------------------
void serialClearInput() {
  do {
    delay(10);
  } while (Serial.read() >= 0);
}
//------------------------------------------------------------------------------
bool serialReadLine(char* str, size_t size) {
  size_t n = 0;
  while(!Serial.available()) {
    yield();
  }
  while (true) {
    int c = Serial.read();
    if (c < ' ') break;
    str[n++] = c;
    if (n >= size) {
      Serial.println(F("input too long"));
      return false;
    }
    uint32_t m = millis();
    while (!Serial.available() && (millis() - m) < 100){}
    if (!Serial.available()) break;
  }
  str[n] = 0;
  return true;
}
//------------------------------------------------------------------------------
void testSensor() {
  const uint32_t interval = 200000;
  int32_t diff;
  data_t data;
  serialClearInput();
  Serial.println(F("\nTesting - type any character to stop\n"));
  delay(1000);
  printRecord(&Serial, nullptr, test);
  uint32_t m = micros();
  while (!Serial.available()) {
    m += interval;
    do {
      diff = m - micros();
    } while (diff > 0);
    logRecord(&data);
    printRecord(&Serial, &data, test);
  }
}
//------------------------------------------------------------------------------
void setup() {
  
  Serial.begin(250000);
  Wire.begin();

  // Setup LED, turn LED off
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Setup buttons on OLED shield
  pinMode(SWITCH_PIN, INPUT);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // Initialize SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
    Serial.println("Error: Could not open SD card. Freezing...");
    // Add oled error message
  }
  
  //Initialize OLED Display
  display.begin(0x3C, true);
  display.display();
  delay(1000);
  display.setRotation(1);
  
  //Initalize Mux
  if (myMux.begin() == false)
  {
    // Add oled error message
    Serial.println("Error: Mux not detected. Freezing...");
    while(1);
  }
  
  //Initialize RealTimeClock (RTC)
  if (!rtc.begin()) {
    // Add oled error message
    Serial.println("Couldn't find RTC. Freezing...");
    while(1);
  }

  //Initialize Magnetometers
  Wire.setClock(400000);
  //TODO setup_mags
  for(int idx=0; idx < NUM_BOARDS; idx++){
    Serial.print("Setting up Board #");
    Serial.println(port_order[idx]);
    myMux.setPort(port_order[idx]);
    delay(2);
    setup_mags(); // Init with the rate
  }

  //start the clock and start screen
  rtc.start();
  displayStartScreen();
  Serial.println("Finished all setup successfully.");
  
}
//------------------------------------------------------------------------------
void loop() {

  if(!digitalRead(BUTTON_A)){

      createBinFile();
      stopAllBurst();

      // print glove settings to first line on SD card and to serial port
      printHeader(&binFile);
      printHeader(&Serial);
      displayRecordingScreen(); 
      startAllBurst();
      //delay(2000);
      logData(); 
  }
  if(!digitalRead(BUTTON_B)){
      displayHealthCheck();
  }
  if(!digitalRead(BUTTON_C)){
      syncTime();
  }
  delay(200);
}
//------------------------------------------------------------------------------
// Helpers for the magnetometers
void startAllBurst(){
  
  for(int idx=0; idx < NUM_BOARDS; idx++){
    myMux.setPort(port_order[idx]);
    delay(5);
    mlx0.startBurst(0xF);
    mlx1.startBurst(0xF);
    mlx2.startBurst(0xF);
    mlx3.startBurst(0xF);
  }
}

void stopAllBurst(){
  
  for(int idx=0; idx < NUM_BOARDS; idx++){
    myMux.setPort(port_order[idx]);
    delay(5);
    mlx0.exit();
    mlx1.exit();
    mlx2.exit();
    mlx3.exit();
  }
  
}
void setup_mags(){

  byte status = mlx0.begin(mlx0_i2c, -1, Wire);
  Serial.print("Start status: 0x");
  if(status < 0x10) Serial.print("0"); 
  Serial.println(status, BIN);
  if(status > 0){ blink_once();} // blink LED for error
  
  status = mlx1.begin(mlx1_i2c, -1, Wire);
  Serial.print("Start status: 0x");
  if(status < 0x10) Serial.print("0"); 
  Serial.println(status, BIN);
  if(status > 0){ blink_once();} // blink LED for error
  
  status = mlx2.begin(mlx2_i2c, -1, Wire);
  Serial.print("Start status: 0x");
  if(status < 0x10) Serial.print("0");
  Serial.println(status, BIN);
  if(status > 0){ blink_once();} // blink LED for error
  
  status = mlx3.begin(mlx3_i2c, -1, Wire);
  Serial.print("Start status: 0x");
  if(status < 0x10) Serial.print("0"); 
  Serial.println(status, BIN);
  if(status > 0){ blink_once();} // blink LED for error

  
}

void blink_once(){

  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(200);
}

void displayStartScreen(){
  
  display.clearDisplay();
  display.display();
  
  display.setTextSize(1.5);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("Force Glove Logger");
  display.setCursor(0,10);
  display.println("<-Press A to START ");
  display.println("recording to SD card.");
  display.setCursor(0,30);
  display.println("<-Press B to display a health check");
  display.setCursor(0,50);
  display.println("<-Press C to sync RTC");
  display.display(); // actually display all of the above
  //delay(500);
}

void displayRecordingScreen(){
  
  display.clearDisplay();
  display.display();

  display.setCursor(0,0);
  display.println("Force Glove Logger");

  display.setCursor(0,10);
  display.println("<-Press A to STOP ");
  display.println("recording to sd card.");
  
  display.setCursor(0,30);
  display.println("Recording to file: ");
  display.println(binName);

  display.setCursor(0,50);
  display.print("Started:");
  now = rtc.now();
  display.print(now.hour(), DEC);
  display.print(':');
  display.print(now.minute(), DEC);
  display.print(':');
  display.println(now.second(), DEC);
  display.display();
}

void displayHealthCheck(){

  stopAllBurst();
  
  uint8_t gain_sel;
  uint8_t hall_conf;
  uint8_t res_x, res_y, res_z;
  uint8_t tcmp_en;
  
  myMux.setPort(0);
  delay(5);
  uint8_t status1 = mlx0.getGainSel(gain_sel);
  uint8_t status2 = mlx0.getHallConf(hall_conf);
  uint8_t status3 = mlx0.getResolution(res_x, res_y, res_z);
  uint8_t status4 = mlx0.getTemperatureCompensation(tcmp_en);

  if(status1 | status2 | status3 | status4){
      status1 = mlx2.getGainSel(gain_sel);
      status2 = mlx2.getHallConf(hall_conf);
      status3 = mlx2.getResolution(res_x, res_y, res_z);
      status4 = mlx2.getTemperatureCompensation(tcmp_en);
  }
  
  display.clearDisplay();
  display.display();

  display.setCursor(0,0);
  display.println("Force Glove Logger");
  //add info
  
  display.setCursor(0,10);
  display.print(status1); display.print(" "); 
  display.print(status2); display.print(" "); 
  display.print(status3); display.print(" "); 
  display.println(status4); 
  
  display.setCursor(0,25);
  display.print(gain_sel); display.print(" "); 
  display.print(hall_conf); display.print(" "); 
  display.print(tcmp_en); display.print(" ");
  display.print(res_x); display.print(" "); 
  display.print(res_y); display.print(" "); 
  display.println(res_z);

  display.setCursor(0,40);
  display.println("Press B to return");
  
  display.setCursor(0,55);
  display.print("Checked at:");
  now = rtc.now();
  display.print(now.hour(), DEC);
  display.print(':');
  display.print(now.minute(), DEC);
  display.print(':');
  display.println(now.second(), DEC);
  display.display();
  //delays are to make button/screen easier to use
  delay(2000);
  while(digitalRead(BUTTON_B)){
    delay(200);
  }
  displayStartScreen();
  startAllBurst();
  delay(500);
}

void syncTime(){

  stopAllBurst();
  
  display.clearDisplay();
  display.display();

  display.setCursor(0,0);
  display.println("Force Glove Logger");

  display.setCursor(0,10);
  display.println("Micro-USB required.");
  
  display.setCursor(0,20);
  display.println("Requesting time...");
  Serial.println("Requesting time from PC...");
  display.display();
  
  while(!Serial.available()){ delay(1);}
  inyear = Serial.parseInt();
  while(!Serial.available()){ delay(1);}
  inmonth = Serial.parseInt();
  while(!Serial.available()){ delay(1);}
  inday = Serial.parseInt();
  while(!Serial.available()){ delay(1);}
  inhour = Serial.parseInt();
  while(!Serial.available()){ delay(1);}
  inminute = Serial.parseInt();
  while(!Serial.available()){ delay(1);}
  insecond = Serial.parseInt();
  
  rtc.adjust(DateTime(inyear, inmonth, inday, inhour, inminute, insecond));

  display.setCursor(0,40);
  now = rtc.now();
  display.print("RTC Time: ");
  display.print(now.hour(), DEC);
  display.print(':');
  display.print(now.minute(), DEC);
  display.print(':');
  display.println(now.second(), DEC);
  
  display.setCursor(0,55);
  display.println("Press C to return");
  display.display();

  delay(2000);
  while(digitalRead(BUTTON_C)){
    
    delay(200);
  }
  displayStartScreen();
  startAllBurst();
  delay(500);
}
