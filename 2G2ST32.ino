// The FW rev is defined at the top of 2G2ST32.h
// Change log at end of this file.
#include "2G2ST32.h"                // #define FW_REV, Pindefs and compile #defs.
#include <i2c_t3.h>                 // PJRC for timing functions and I2C, replaces <Wire.h>
#include "lidar_IO.h"               // move GarminModes.h here
#include "pitches.h"

// ISR's on t3 timer
IntervalTimer Timer_Send_Com1; //Timer to be attached to an interrupt to keep 200Hz data flow out of SmLIDAR
IntervalTimer Timer_Watchdog_NonRespond_Peripherals;
IntervalTimer LaserFlash;
extern uint8_t Lidar_Read(int Select, uint8_t Register);

volatile long BusyCapture[2] = {0,0};
volatile int LaserFlashSpeed = 0;
volatile int RestartRequired = FALSE;
long AimLaserFlashRates[10] = {1000000, 800000, 600000, 400000, 200000, 100000, 80000, 60000, 40000, 20000}; 
int Sampling = TRUE;  //Can kill SD sampling in SampleBackground routine if stuck.
int Errors_Till_CPU_RESTART = 0;
float Mean[2], StdDev[2];    // Added 190505 to use baseline quality as part of 'black car' reconstructions
float FarThreshold[2];       // 190505 5 (for now) std deviations above baseline mean.
int StdDevSampleCutoff = 5;

unsigned long millis_start, millis_now;  // For TB_Monitor command receipt timing.
uint8_t LL3_Status_0x01[2];              // Busy status of Garmin.  Replace reading if busy.

// State variables set when TB enables S-200 to free-run.   Made TRUE as each LIDAR is issued the $GO S-200 command.
int Run_Port1 = FALSE, Run_Port2 = FALSE;  

// Various signal proc or user feedback 'features'. 
int AudioFeedback = FALSE;
int RaindropMode = FALSE;
int StreamUSB = FALSE;
int ReplLongErrantReadings = TRUE;   // To invoke replacement of invalid long distance returns with last goo known reading.
int PopWindow = TRUE;          // TRUE enables code to calc running average & "fix" readings thought to be errant.
//int AvgBkg[2] = {100, 100}, 
int AvgArrayLen = 16;
int NewRawDist[2] = {0, 0},  LastRawDist[2] = {0, 0};
int CurrentAvgDist[2] = {100, 100};

// not using ClipperMode at 190519 revision.
int ClipperMode = TRUE, Far_DistClip = 2500, Near_DistClip = 10;   //Temporary fix DistClip to 25m

volatile char OutDataSent = TRUE, Set_LED_Timer = FALSE;
char OutString[3][32];  // RS-232 ports.  Typical S-200 "$DR,dist,time,CRC" return is 24 chars max.
char CommandReply_OutString[3][32];  // RS-232 ports.  Typical S-200 "$DR,dist,time,CRC" return is 24 chars max.

int PreviousReading[2];
int busybit[2] = {0,0};
volatile int LIDAR_Dist_Buf[2][258];           // Ring buffer for distance data from LIDAR.
volatile int LIDAR_Ints_Buf[2][258];           // Ring buffer for intensity data from LIDAR.
volatile int PROC_Buffer[2][258];              // Ring buffer for processed data.
volatile int Head_In = 0, Tail_In = 0;         // pointers for LIDAR_Dist_Buf[]
volatile int Proc_Head, Proc_Tail;             // pointers for PROC_Buffer[]
//volatile int Dist_Hist[2][1025];             // NOTE: Garmin LIDAR range is 40m, i.e. 4000 cm.  for now distance range will be
//volatile int Intensity_Hist[2][1025];        // constrained to a 1024 bins by dividing distance by 4, then multiplying when extracted.
// Intensity maximums thus far 1810124 have been seen no higher than 616.

int AvgLevel = 3;                      //Exponent to set averaging array size, i.e. the x parameter passed in $AV,x.
int Valid_S200_Command = FALSE;               // Set to TRUE if real S-200 command is validated, this will trigger command decode and Arduino reply.
volatile int DistNow[2], SigInt[2], PeakCorr, NoisePeak, LED13_mode, ToneOut;
volatile float f_millis_S200 = 0, f_last_millis_S200 = 1, millis_ISR_period = 5;

unsigned short CrC;
int AdditionalParameters = FALSE;

volatile int TF_lidar_dist = 0;  //For Benewake TFmini LIDAR

int MM = 0, DM = 0, DT = 0, MU = 0, PO = 0,  DI = 0; // S-200 variables.  See GarminModes.h for documentation.
char DM_mode = 3, DT_mode = 2;  // Internal variables to hold S-200 mode "substitutes".

// Temporary variables for confirming performance during development....
int watchdogLoops=0;
int WhichLidarOn = 0;
float Hertz;
unsigned long LoopsLow = 0, LoopsHigh = 0;
int BitBucket;
volatile unsigned long uSound_duration;
unsigned int LoopResetter = 0;
int DEBUG_SN_HighByte[2], DEBUG_SN_LowByte[2], DEBUG_reading_0x66_Status[2];
int GetLaserStat_0x66(int WhichLAser);
int DEBUG_ErrCnt_StateMachine[2] = {0, 0};
long LoopLoops = 0;
int FakeDataLoops = 0;
// end of temp variables for checking timing (remove them at some point)
//void TestTone(int);


void setup() {
  pinMode(PIN_Teensy_Led, OUTPUT);        digitalWrite(PIN_Teensy_Led, HIGH);
  pinMode(PIN_LIDAR_1_PWR_ENAB, OUTPUT);  digitalWrite(PIN_LIDAR_1_PWR_ENAB, LOW);
  pinMode(PIN_LIDAR_2_PWR_ENAB, OUTPUT);  digitalWrite(PIN_LIDAR_2_PWR_ENAB, LOW);
  pinMode(PIN_LIDAR_1_MODE_CTL, OUTPUT);  digitalWrite(PIN_LIDAR_1_MODE_CTL, HIGH);
  pinMode(PIN_LIDAR_2_MODE_CTL, OUTPUT);  digitalWrite(PIN_LIDAR_2_MODE_CTL, HIGH);

#ifdef DEBUG_PINS
  pinMode(PIN_TestPoint_1, OUTPUT);  digitalWrite(PIN_TestPoint_1, LOW);
  pinMode(PIN_TestPoint_2, OUTPUT);  digitalWrite(PIN_TestPoint_2, LOW);
#endif

/*
#ifdef BENCH_PROTOTYPE    // Pinout for proto-board prototype
  pinMode(PIN_TestPoint_1, OUTPUT);  digitalWrite(PIN_TestPoint_1, LOW);
  pinMode(PIN_TestPoint_2, OUTPUT);  digitalWrite(PIN_TestPoint_2, LOW);
  pinMode(PIN_trig, OUTPUT);         // ping output for ultrasound device on prototype
  pinMode(PIN_echo, INPUT);          // Goes high on ping return.
  pinMode(PIN_Speaker, OUTPUT); digitalWrite(PIN_Speaker, LOW);
  pinMode(PIN_Speaker, OUTPUT);
  pinMode(PIN_RedLaser, OUTPUT);      digitalWrite(PIN_RedLaser, RED_LASER_ON);
#endif
*/

#ifdef LIFACE_2    // Pinout for PCB deployed in production units
  pinMode(PIN_Speaker, INPUT);
  //pinMode(PIN_TestPoint_1, OUTPUT);   digitalWrite(PIN_TestPoint_1, LOW);
  //pinMode(PIN_TestPoint_2, OUTPUT);   digitalWrite(PIN_TestPoint_2, LOW);
  pinMode(PIN_Red_LED, OUTPUT);       digitalWrite(PIN_Red_LED, LOW);
  pinMode(PIN_Grn_LED, OUTPUT);       digitalWrite(PIN_Grn_LED, LOW);
  pinMode(PIN_RedLaser, OUTPUT);      digitalWrite(PIN_RedLaser, RED_LASER_ON);
#endif

  Serial.begin(115200);  // USB serial port, temporary for dev/debug.
  Serial1.begin(115200); // Pins RX1 and TX1 on the T32 https://www.pjrc.com/teensy/card5a_rev7.png
  Serial2.begin(115200); // Pins RX2 and TX2 on the T32 second I/O to IoT host.
  TestTone(1);  //delay(10);
  Serial.print("FW_VER:");  Serial.print(FW_VER);
  Serial.print(".");  Serial.print(FW_SUBVER); 
  
  float fDataRate = (float)1/((float)IRQ_WRITE_PERIOD_uS/1000000);
  Serial.print(fDataRate); Serial.println(" Hz");       
  
  ReSetup();   // Broken out of setup() so it can be called in the event of a hang-up.
  // LIDARS should be ready now, enable interrupts and GO!
  millis_ISR_period = IRQ_WRITE_PERIOD_uS / 1000;
  Timer_Send_Com1.begin(INT_SendSerial_1, IRQ_WRITE_PERIOD_uS); //5000 mS gives 200Hz out at Serial1 to controlling device.
  Timer_Watchdog_NonRespond_Peripherals.begin(INT_Watchdog, 1000000); 
  //LaserFlash.begin(INT_LaserFlash, 1000);
  
  SampleBackground();
  TestTone(4);
}

/*  ***********   SETUP and ReInitializations   ***************  */
void ReSetup(void) {
  RestartRequired = FALSE;
    //PlayTheTune(0);  // BENCHTOP_PROTOTYPE  To detect if system has been self-restarted
  //#ifdef DEBUG_USB
    Serial.println("2G2ST32->ReSetup()");
    Serial.println("Init I2C at 400KHz.....");
  //#endif
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400, I2C_OP_MODE_DMA );
  Wire.resetBus();
  TestTone(2);
  MoveLidar_Address();  // Need to ensure this was sucessful.  
  TestTone(3);
  f_millis_S200 = 0;        // jwb 190219 In the event this routine was called to reset the "time freeze" error
  f_last_millis_S200 = 1;
  //SampleBackground();                                            // Fills first half of LIDAR_Dist_Buf
  
}

void TestTone(int n){
  //int Freq = 1500+n*250;
  if(n>5) n=5;
  if(n<0) n=0;
  int StdTones[6] = {2500, 3000, 3500, 4000, 4500, 5000};
  pinMode(PIN_Speaker, OUTPUT);    // Enable speaker.
  //Serial.println(StdTones[n]);
  tone(PIN_Speaker, StdTones[n]); delay(100); noTone(PIN_Speaker);
  pinMode(PIN_Speaker, INPUT);     // Disable speaker (prevent any heating due to DC bias at Teensy OUTPUT
return;
}

/*    *******  ISRs  ISRs  ISRs  ISRs  ISRs  ISRs  ISRs  *********     */
void INT_SendSerial_1() {        // Interrupt to send data every 5mS (200 Hz)
  #ifdef DEBUG_PINS
  digitalWrite(PIN_TestPoint_1, HIGH);
  #endif

  // Run_PortX is made TRUE by sucessful S-200/laserpp init seqence.
  if (Run_Port1 == TRUE) {  Serial1.println(OutString[0]);    }
  if (Run_Port2 == TRUE) {  Serial2.println(OutString[1]);    }
  if (StreamUSB == TRUE){   Serial.println(OutString[2]); }  
  
  OutDataSent = TRUE;                           // Detected in main loop for initiating new LIDAR readings
  Set_LED_Timer = TRUE;                         // Indicates data is being output.
  f_millis_S200 = f_millis_S200 + millis_ISR_period;   // Timestamp on faux S-200 readings
  //  S-200 rolls over at 10 seconds.  Reset one ISR period before hitting 10000 mS (10 S).
  if (f_millis_S200 > (10000 - millis_ISR_period)) f_millis_S200 = 0;
  
  #ifdef DEBUG_PINS
  digitalWrite(PIN_TestPoint_1, LOW);
  #endif
}

void INT_Watchdog() {   //jwb 5Dec18 Used for a status indication at present
  static long runningMinutes = 0;
  if (digitalRead(PIN_Teensy_Led) != HIGH){  digitalWrite(PIN_Teensy_Led, HIGH); 
  }else{    digitalWrite(PIN_Teensy_Led, LOW);    }
  
  #ifdef WD_MSG
    if ((watchdogLoops >= 60) && (runningMinutes < 120)){
      runningMinutes += 1;
      watchdogLoops = 0;    // reset to get print every minute.
      Serial.print("min,");       Serial.print(runningMinutes);
      Serial.print(",L0D,");      Serial.print(NewRawDist[0]);
      Serial.print(",Busy0,");    Serial.print(BusyCapture[0]);
      Serial.print(",L1D,");      Serial.print(NewRawDist[1]);
      Serial.print(",Busy1,");    Serial.println(BusyCapture[1]);
      BusyCapture[0] = 0; BusyCapture[1] = 0;
    }
  #endif
  watchdogLoops += 1;
}


void INT_LaserFlash(){
  static int LaserStatus;
  if(LaserFlashSpeed == 0){
    if (LaserStatus != 0){
     LaserStatus = 0;
     digitalWrite(PIN_RedLaser, RED_LASER_OFF);
   }else{
     LaserStatus = 1;
     digitalWrite(PIN_RedLaser, RED_LASER_ON);
   }
  }
}

void loop() {
  int i, Look_Ahead = 3;     // for now, simple spike filter is the only use of Look_Ahead.  Comp current to neighbors.   long ReadingAccumulator = 0;   // for the running average maintained by the process buffer (PROC_Buffer).
  static unsigned long start_micros, micros_now, elapsed_micros;
  long ReadingAccumulator = 0;   // for the running average maintained by the process buffer (PROC_Buffer).
  static int Got_LL3_2 = FALSE;  //Got_LL3_1 = FALSE, 
  int SpeakerFreq = 1000; //, LastSpeakerFreq = 0;
  float ScaleSpeakerFreq = 1;
  
  if (RestartRequired == TRUE) {   CPU_RESTART;   } //ReSetup();   // $SE - send here through any serial port.
  if ( OutDataSent == TRUE ) {                      // Every 5mS (200Hz) per INT_SendSerial_1() increment the ring buffer.
    start_micros = micros();                        
    
    // Read first LL3, and if not busy, get a new reading, otherwise, substitute.
    LL3_Status_0x01[0] = Lidar_Read (0, REG_ADDR_SystemStatus);
    SigInt[0] = LL3_Status_0x01[0] * 100;    //  Temporarily to allow busy to be seen by vlogs.
    LL3_Status_0x01[0] = LL3_Status_0x01[0] & 0x01;     
    if(LL3_Status_0x01[0] != 0){     // LL3 is busy.
      NewRawDist[0] = PreviousReading[0]; 
      BusyCapture[0] += 1;
      busybit[0] = 1;
    }else{
      busybit[0] = 0;
      NewRawDist[0] =  readDistance(0); // Get distance now and store.  readDistance() now fills in SigInt[2], i.e. intensity.
      if ( NewRawDist[0] < (Mean[0] + StdDevSampleCutoff * StdDev[0])){  // 190816 jwb Critical for Garmins.  This allows 
        PreviousReading[0] = NewRawDist[0];             // errant readings (beyond target) to be 'skipped'.
      }  
    }
    elapsed_micros = 1;     // For rollover case     micros() timer rolls over every 70 minutes.
    while (Got_LL3_2 == FALSE){
      while (elapsed_micros <= IRQ_WRITE_PERIOD_uS/2){     //    Wasting time here
        micros_now = micros();                                              //    while we get
        if( micros_now >= start_micros){                                    //    to the second read
          elapsed_micros = micros_now - start_micros;                       //    in the 10mS  100Hz
        }else{  //Rollover happened                                         //    period.
          elapsed_micros = 4294967295 - start_micros + micros_now;          //    Have to do this 
        }                                                                   //    by interrupt to recover 
      }                                                                     //    time for other stuff.
      if (elapsed_micros >= IRQ_WRITE_PERIOD_uS/2){ 
        LL3_Status_0x01[1] = Lidar_Read (1, REG_ADDR_SystemStatus); // 0x00 from 0x01 means not busy.
        SigInt[1] = LL3_Status_0x01[1] * 100;     // Temporarily to allow vlogs to see busy status.
        LL3_Status_0x01[1] = LL3_Status_0x01[1] & 0x01;    
        if(LL3_Status_0x01[1] != 0){          // LL3 is busy.   
          NewRawDist[1] = PreviousReading[1]; 
          BusyCapture[1] += 1;
          busybit[1] = 1;
        }else{   //LL3 was not busy, so reading is valid and current.  
          busybit[1] = 0;
          NewRawDist[1] =  readDistance(1); // Get distance now and store.  readDistance() now fills in SigInt[2], i.e. intensity.        
          if ( NewRawDist[1] < (Mean[1] + StdDevSampleCutoff * StdDev[1]) ){   // Only update Previous readings with valid & current readings.
            PreviousReading[1] = NewRawDist[1];
          }
        }
        Got_LL3_2 = TRUE; // > 2.5mS, so read, and break while(Got_LL3_2 == FALSE)
      }
    } 
    Got_LL3_2 = FALSE;   // Broke out of Got_LL3_2 == FALSE loop, need to make it false for next time.
    // At this point, NewRawDist[n] is the latest distance read, UNLESS the LL3 was busy, in which case, 
    // NewRawDist[n] will contain the last reading.  
    // NOTE: DistNow[n] will be the variable used to generate the formatted output string.  

    // FIRST CORRECTION of data:  Distances longer than sampled mean are erroneous.  
    // Criteria: reading is 'longer' than 'baseline' (distance to background target. (2 standard deviations))
    // if criteria is met, substitute the last valid reading.   NOTE: 'valid' might already be the previous reading
    // if the LL3 too too long to respond.  
  
    if (ReplLongErrantReadings == TRUE){       // default is TRUE.
     for(i=0; i<=1; i++){                     
       DistNow[i] = NewRawDist[i];      // Ultimately DistNow[i] is used by Generate_S200_OutputString(i);
       if(DistNow[i] > (Mean[i] + 2 * StdDev[i])){   // Testing to detect aberant 'far' readings, such as 
            DistNow[i] = PreviousReading[i];          // are produced by unusual windshield angles, 'excessive specularity',
       }                                             // and black vehicles
     }
    }else{
      for(i=0; i<=1; i++){
        DistNow[i] = NewRawDist[i];
      }
   }


   // **** Calculate 'running average', i.e. an averave (of variable length) which slides along the raw data.
   // LIDAR_Dist_Buf[] was used previously for 'raw' data, it now stores 'last good' IF LL3 was busy.
   // PROC_Buffer[] is a 256 byte ring buffer to hold the sliding average.
   // Both data arrays use the same head and tail pointers. i.e. they stay synced that way.  
   if (++Head_In > 255){ Head_In = 0; }
     Tail_In = (Head_In - Look_Ahead) & 0x00ff;      // Tail pointer follows Head by Look_Ahead. 
      // Then calculate the new average.
     if (AvgArrayLen >= 128) AvgArrayLen = 128;  /* For now, need to keep the max averaging array 
                                                       under 256 so it doesn't run into the tail. */
     // PROC (or Proc) is generic for PROCess, presently the only process is a running average.
     Proc_Head = Tail_In;
     Proc_Tail = (Proc_Head - AvgArrayLen) & 0x00ff;  // Grab incoming data after spike removal (if any)
  
     for (i = 0; i <= 1; i++) {       
       LIDAR_Dist_Buf[i][Head_In] = DistNow[i];    // Put latest reading at head of ring buffer.
       PROC_Buffer[i][Proc_Head] = LIDAR_Dist_Buf[i][Tail_In];     
       ReadingAccumulator = 0;
       for (int q = 0; q <= AvgArrayLen-1; q++){     // accumulates from tail to head.
         ReadingAccumulator += PROC_Buffer[i][((Proc_Tail + q) & 0x00ff)];
       }
       DistNow[i] = (int)(ReadingAccumulator / AvgArrayLen);  // Writing calculated avg to latest PROC value
     }
     /****************************************/

    //  SECOND correction, 'Pop' filter (raindrops, and the random errant distances sometimes seen)
    if(PopWindow == TRUE){  // This will set readings that are bounded by high differentials within 
                            // 'a few' mS of each other to Mean[n].
          ;
    }

    for(i=0; i<=1; i++){               // Last step after all signal corrections....format the string 
      Generate_S200_OutputString(i);   // to be sent up to the linux board BEFORE the next IRQ.  
    }
    if (StreamUSB == TRUE){   // 190815 jwb
      Generate_USB_OutputString(); 
    }
  /*************************************************************************************************************/  
  } 
  OutDataSent =  FALSE;   //The previous if(OutDataSent == TRUE) only runs when ISR sets OutDataSent.

   // Speaker range is 1000 Hz at Mean[0] distance, 8000 Hz at 0 cm.  
   // For ref, on SN 126642180104, cutoff freq of spkr is ~800Hz.  
   /*   Removed temporarily 190911 until time effect is confirmed 
   if ((DistNow[0] < FarThreshold[0]) || (DistNow[1] < FarThreshold[1])){
      digitalWrite(PIN_RedLaser, RED_LASER_ON);    
      if (AudioFeedback == TRUE){  // Put out a tone every so many entries so this dowsn't sound too bad.
         ScaleSpeakerFreq = (1-((float)DistNow[0] / (float)Mean[0]))*7000;  
         SpeakerFreq = (int)(ScaleSpeakerFreq) + 1000 ; 
      }
   }else{
     noTone (PIN_Speaker);
     digitalWrite(PIN_RedLaser, RED_LASER_OFF);
   }  */
  int ret = 0;
  if (Serial.available() > 0 ) { ret=Build_Check_S200_Command(0);  // USB port
  } else {
    if (Serial1.available() > 0) {  ret=Build_Check_S200_Command(1);  // First 'S-200' port
    } else {
      if (Serial2.available() > 0){ ret=Build_Check_S200_Command(2);   // Second 'S-200' port
      }
    }
  }
  if (ret != 0)Serial.println("Somebody sent bad S-200 command");
}

void Generate_USB_OutputString(void) {
  char bufTemp[8];
  char TempOutBuf[32] = {"**"};
  float fDistNow;
  noInterrupts();   
  strcpy(OutString[2], "USB");
  strcpy(TempOutBuf, ":");
  
  fDistNow = (float)DistNow[0] / 100;  // 190815 Get laser 0
  sprintf(bufTemp, ",%3.2f", fDistNow); //
  strcat(TempOutBuf, bufTemp);
  
  fDistNow = (float)DistNow[1] / 100;  // 190815 Get laser 1
  sprintf(bufTemp, ",%3.2f", fDistNow); //
  strcat(TempOutBuf, bufTemp);
    
  sprintf(bufTemp, ",%4.3f", f_millis_S200 / 1000); // Get time
  strcat(TempOutBuf, bufTemp);  

  sprintf(bufTemp, ",%d,%d", busybit[0], busybit[1]);
  strcat(TempOutBuf, bufTemp);  
  
  strcat(OutString[2], TempOutBuf);
  interrupts();
  return;
}

void Generate_S200_OutputString(int WhichLaser) {
  char bufTemp[8];
  char TempOutBuf[32];
  float fDistNow;
  noInterrupts();   
  fDistNow = (float)DistNow[WhichLaser] / 100;  // 190326 Temp removed to look at raw distance
  // Need to detect targeting mode here and start the return strings with the right prefix.
  strcpy(TempOutBuf, "DR,");           // S-200 string starts with "$", but CRC is calculated without $
  sprintf(bufTemp, "%3.2f", fDistNow); //
  strcat(TempOutBuf, bufTemp);
  
  if(WhichLaser == 1){   // Need to add a half ISR increment to 'staggered' laser. //f_millis_S200 rollover handled in ISR.
    f_last_millis_S200 = f_millis_S200;  //Hold LL3[0] time
    if (f_millis_S200 == 0){      // 10 sec S-200 rollover - 1 increment
      f_millis_S200 = 10000 - IRQ_WRITE_PERIOD_uS / 2000;
    }else{
      f_millis_S200 += IRQ_WRITE_PERIOD_uS / 2000;
    }
  }
  sprintf(bufTemp, ",%4.3f", f_millis_S200 / 1000);
  if(WhichLaser == 1) f_millis_S200 = f_last_millis_S200;   // End of staggered timestamp for LL3[1].
  strcat(TempOutBuf, bufTemp);
  sprintf(bufTemp, ",%3d", SigInt[WhichLaser] / 10);   //190128 jwb added temp Garmin intensity for vlogs
  strcat(TempOutBuf, bufTemp);  // ",12");  was just set at 12...random value
  strcpy(CommandReply_OutString[WhichLaser], TempOutBuf); // 190306 FOR DEBUGGING CRC *************************************
  CrC = crc16(0x00, (unsigned char*)TempOutBuf, strlen(TempOutBuf));
  sprintf(bufTemp, "*%04X", CrC);
  strcat(TempOutBuf, bufTemp);
  strcpy(OutString[WhichLaser], "$");
  strcat(OutString[WhichLaser], TempOutBuf);
  interrupts();
  return;
}


int Build_Check_S200_Command(int SerialChannel) {
  static char IncomingString[16];         // OK to interrupt filling this.
  char *InStrPtr = &IncomingString[0];   // loop() is checking and processing incoming streams sequentially, i.e.
  int StrPtr;                             // it checks serial, then serial1, then serial2.
  StrPtr = 0;                             // i.e. completely reply to any incoming serial command before going on to the next.
  switch (SerialChannel) {  // @115200, ~9uS per bit so roughly 112 uS for all bits. 
    case 0: while (Serial.available())  { IncomingString[StrPtr++] = Serial.read();  delayMicroseconds(120); }   break;
    case 1: while (Serial1.available()) { IncomingString[StrPtr++] = Serial1.read(); delayMicroseconds(120); }   break;
    case 2: while (Serial2.available()) { IncomingString[StrPtr++] = Serial2.read(); delayMicroseconds(120); }   break;
    default: break;
  }
  if (((IncomingString[0] == '$') || (IncomingString[0] == '?'))) {
    Valid_S200_Command = TRUE;
    #ifdef DEBUG_USB
      Serial.print("tty:"); Serial.print(SerialChannel);  Serial.print(" rcvd:");  Serial.print(IncomingString);
    #endif
    Process_S200_Command(SerialChannel, InStrPtr);
    return 0;
  }
  return 1;
}

void SendSerialReply(int WhichPort, char *SendString) {
  switch (WhichPort) {
#ifdef DEBUG_USB
    case 0: Serial.println(SendString);  break;
#endif
    case 1: Serial1.println(SendString);  break;
    case 2: Serial2.println(SendString);  break;
    default: break;
  }
//#ifdef DEBUG_USB
  Serial.print("tty:"); Serial.print(WhichPort); Serial.print(" send: "); Serial.println(SendString);
//#endif

  return;
}

int Process_S200_Command(int SourcePort, char *CommandString) {   //  S-200 and S-200(aux) command decoder.
  int OK_Flag, LL3_FW;
  //static int CountStops = 0;
  static char OutgoingString[32];    // 190417 jwb possibly related to "static" declaration? 
  static char *OutStrPtr = &OutgoingString[0];
  
  OK_Flag = FALSE;
  // Test for 'Return OK' cases.....  If found, set OK_Flag to TRUE to jump around CRC gen part.
  if ((CommandString[1] == 'G') && (CommandString[2] == 'O')) {
    if (SourcePort == 0 ) {      Run_Port1 = TRUE;      Run_Port2 = TRUE;    }
    if (SourcePort == 1 ) {      Run_Port1 = TRUE;    }
    if (SourcePort == 2 ) {      Run_Port2 = TRUE;    }
    OK_Flag = TRUE;
    strcpy(OutgoingString, "$OK*0774");
    SendSerialReply( SourcePort, OutStrPtr );
  }
  if ((CommandString[1] == 'S') && (CommandString[2] == 'T')) {
    if (SourcePort == 0 ) {      Run_Port1 = FALSE;      Run_Port2 = FALSE;    }
    if (SourcePort == 1 ) {      Run_Port1 = FALSE;    }
    if (SourcePort == 2 ) {      Run_Port2 = FALSE;    }
    OK_Flag = TRUE;
    strcpy(OutgoingString, "$OK*0774");
    SendSerialReply( SourcePort, OutStrPtr );
  }
  if ((CommandString[1] == 'P') && (CommandString[2] == 'O')) {
    OK_Flag = TRUE;
    strcpy(OutgoingString, "$OK*0774");
    SendSerialReply( SourcePort, OutStrPtr );
  }
  if ((CommandString[1] == 'I') && (CommandString[2] == 'D')) {
    switch (SourcePort) {
      case 0: LL3_FW = GetFWsn(0);  
              Serial.print ("Lidar_"); Serial.print (LidarAddress[0]); Serial.print (" FW Ver:");  Serial.println(LL3_FW); 
              LL3_FW = GetFWsn(1); 
              Serial.print ("Lidar_"); Serial.print (LidarAddress[1]); Serial.print (" FW Ver:");  Serial.println (LL3_FW);
              Serial.print ("Teensy FW:"); Serial.print(FW_VER); Serial.print("-");  Serial.println(FW_SUBVER);       //sprintf(OutgoingString, "%d", FW_VER);       
              break;
      case 1: strcpy(OutgoingString, "SER-1"); break;
      case 2: strcpy(OutgoingString, "SER-2"); break;
      default: break;
    }
    OK_Flag = TRUE;
    SendSerialReply( SourcePort, OutStrPtr );
    //Serial1.println("$ID,DS-100,TruSense S200-1.14-76,MAY 20 2014,ED0B0894*508A"); OK_Flag = TRUE;
    //Serial.println("!!!-$ID,DS-100,TruSense S200-1.14-76,MAY 20 2014,ED0B0894*508A");
  }

  if (OK_Flag != TRUE) { //OK_Flag, just means the command wasn't a simple case with no arguments, processed above.
    switch (CommandString[0]) {
      case '$':
        if ((CommandString[1] == 'T') && (CommandString[2] == 'T')) TestTone(CommandString[4]-48);
        switch (CommandString[1]) {
          // ******  Start of cases for 'S-200+' (Garmin or SEEED) functionality  *************
          case 'E':
            switch (CommandString[2]) {
              case 'U':                                                               
                if(StreamUSB != FALSE){ StreamUSB = FALSE; }else{ StreamUSB = TRUE; }  break;   // $EU
              default:   break;
            }                                                                          break;   // $Ex    
          case 'S':                                                       
            switch (CommandString[2]) {
              case 'R':     CPU_RESTART;                                                     break; // $SH Reboots CPU.   'hard' reset
              case 'B':     SampleBackground();                                              break;  //$SB (Sample Background)
              case 'D':     StdDevSampleCutoff = CommandString[4]-48;    SampleBackground(); break;  //$SB (Sample Background)
              default:                                                                       break;
            }                                                                 break; //$Sx
          case 'L':
            switch (CommandString[2]) {
              case 'P':   // Operate pointing laser on prototype
                if (CommandString[4] == '1') {     
                  digitalWrite(PIN_RedLaser, RED_LASER_ON);
                } else {                           
                  digitalWrite(PIN_RedLaser, RED_LASER_OFF);
                }                                                           break; // $LP, x
              default:                                                      break; 
            }                                                               break; //$Lx
          case 'A':
            switch (CommandString[2]) {
              case 'A': if (AudioFeedback != TRUE){ AudioFeedback = TRUE; TestTone(1);
              }else AudioFeedback = FALSE;   break;
              case 'P': if (AdditionalParameters != TRUE) AdditionalParameters = TRUE;
                else  AdditionalParameters = FALSE;                                                    break;
              case 'V':   // Set averaging to 2 ^^ value in CommandString[4]
                if (isDigit(CommandString[4])) {
                  AvgLevel = (int)(CommandString[4] - 48);   // 0 = 48 in ASCII.
                  AvgArrayLen = CalcAvgArrayLen(AvgLevel);  //= (1 << AvgLevel);
                  #ifdef DEBUG_USB
                    Serial.print("2^"); Serial.print(AvgLevel); Serial.print("=");  Serial.println(AvgArrayLen);
                  #endif
                } else {
                  AvgArrayLen = 0;
                }                                                       break;  // $AV,x end of test for legit averaging mode
              case '?':  
                #ifdef DEBUG_USB
                  Serial.println(AvgArrayLen);                   
                #endif 
                                                                        break;  // $A?  (Get sizeof averaging buffer)
              default: AvgArrayLen = 0;                                 break;  // case was $Ax, not a recognized $A command
            }                                                           break;  // End of case $Ax  (Test for $Averaging command)
          case 'R':
            switch (CommandString[2]) {
              case 'E':  // Replace Error mode
                switch (CommandString[4]) {
                  case '0': ReplLongErrantReadings = FALSE; Serial.println("Replace Err OFF");  break;
                  case '1': ReplLongErrantReadings = TRUE;  Serial.println("Replace Err ON");   break;
                  default:                                                                  break;
                }     break; 
              
              case '0':
                RaindropMode = FALSE;    Serial.println("RD mode OFF");     break;  // $R0
              case '1':
                RaindropMode = TRUE;    Serial.println("RD mode ON");       break;  // $R1
              case '!':
                RestartRequired = TRUE;    Serial.println("RESET");         break;  // resetup()
              case '?':
                if (RaindropMode != 0) {  Serial.println("RD mode ON");
                } else {                  Serial.println("RD mode OFF");
                }
                break;  // $R?       
                
              default: RaindropMode = 1;  //Serial.println("RD mode ON");
                SampleBackground();                                         break;  // $Rx default (raw data) 
            }                                                               break;   // $R
          case 'C':
            switch (CommandString[2]) {
              case '0':
                ClipperMode = 0;    Serial.println("Clipper OFF");          break;  // $C0
              case '?':
                if (ClipperMode != 0) {   Serial.println("Clipper ON"); 
                } else {                  Serial.println("Clipper OFF"); 
                }                                                           break;   // $C?
              default: ClipperMode = 1;   Serial.println("Clipper ON"); 
                SampleBackground();                                         break;  // $Cx  default (raw data)
            }                                                               break;   // $C
          // ******  END of cases for 'S-200+' (Garmin or SEEED) NEW functionality  *************
          // ******  START of cases for S-200 supported commands                    *************
          case 'D':       //DM, DT, & DI commandsl
            switch (CommandString[2]) {
              case 'M':
                switch (CommandString[4]) {
                  case '4':   strcpy(OutgoingString, "$DM,4*F099");
                    SendSerialReply( SourcePort, OutStrPtr );
                    DM_mode = 4;           break;  // $DM,4
                  case '3':   strcpy(OutgoingString, "$DM,3*32D8");
                    SendSerialReply( SourcePort, OutStrPtr );
                    DM_mode = 3;           break;  // $DM,3
                  case '2':   strcpy(OutgoingString, "$DM,2*F219");
                    SendSerialReply( SourcePort, OutStrPtr );
                    DM_mode = 2;           break;  // $DM,2
                  default:    strcpy(OutgoingString, "$DM,2*F219");
                    SendSerialReply( SourcePort, OutStrPtr );
                    DM_mode = 2;           break;  // $DM,x
                }                                                                            break;  // $DM
              case 'T':        //DT modes, time in output, page 14 of @-300 manual
                switch (CommandString[4]) {
                  case '0':   strcpy(OutgoingString, "$DT,0*F449");
                    SendSerialReply( SourcePort, OutStrPtr );
                    DT_mode = 0;           break;  // $DT,0
                  default:    strcpy(OutgoingString, "$DT,2*35C8");
                    SendSerialReply( SourcePort, OutStrPtr );
                    DT_mode = 2;          break;  // $DT,x
                }                                                                            break;  // $DT
              case 'I':
                switch (CommandString[4]) {
                  case '0':   strcpy(OutgoingString, "$DI,0*F2D9");
                    SendSerialReply( SourcePort, OutStrPtr );
                    OK_Flag = TRUE;
                    break;  // $DI,0
                  default:    strcpy(OutgoingString, "$DI,256*93EC");
                    SendSerialReply( SourcePort, OutStrPtr );
                    OK_Flag = TRUE;       break;  // $DI,x
                }                                                                  break;  // $DI
              default:        Serial.println("Not a $Dx      ");                       break;  // $Dx
            }                                                                             break;  // $D
          case 'M':
            switch (CommandString[2]) {
              case 'M':
                switch (CommandString[4]) {
                  case '4':  strcpy(OutgoingString, "$MM,4*6C9A");
                    SendSerialReply( SourcePort, OutStrPtr );             break;  // $MM,4
                  case '3':  strcpy(OutgoingString, "$ER,35,3*2FCB");
                    SendSerialReply( SourcePort, OutStrPtr );             break;  // $MM,3
                  case 'M':  strcpy(OutgoingString, "$MM,0*AF9B");
                    SendSerialReply( SourcePort, OutStrPtr );             break;  // $MM,M
                  default:                                                break;  // $MM,x
                }                                                                       break;  // $MM
              case 'U':
                switch (CommandString[4]) {
                  case 'M':  strcpy(OutgoingString, "$MU,M,22,K,11*14D2");
                    SendSerialReply( SourcePort, OutStrPtr );                           break;  // $MU,M
                  default:                                                              break;  // $MU,x
                }                                                                       break;  // $MU
              default:                                                                  break;  // $Mx
            }                                                                           break;  // $M
          default:                                                                      break;  // $D,$M
        }                                                                                                     break;  // $x
      case '?':  ListCommands();                                                        break;
      default:                                                                          break;  // Not a $x or a ?
    }
  } 
  return 0;  //Success
}


int CalcAvgArrayLen(int AvgLevel){  
    return (1 << AvgLevel);
}

void ListCommands(void) {   // These are the supported commands, echoed back when user sends a '?' to either port
  Serial.print("FW ver "); Serial.println(FW_VER);
  Serial.println("-- Garmin Specific Commands --");
  Serial.println("$SR,  Hard CPU_RESET");
  Serial.println("$SB   Sample Baseline.");   // samples until worse std-dev < a cutoff value
  Serial.println("$SD,x   Sample Baseline."); // Sets std-dev cutoff & invokes the SampleBaseline() routine
  Serial.println("$EU,x Echo serial to USB. x=1 to echo otherwise, off.");
  Serial.println("$RE,x Replace error readings with last good value.  0= noreplace, 1= replace");
  Serial.println("$LP,x Pointer Laser x=1:On x=0:Off");
  Serial.println("$AA,x AudioFeedback.");
  Serial.println("$AV,x Averaging mode.  # averages is 2^x.");
  Serial.println("$A?   Display # of averages setting.");
  Serial.println("$AP   Additional Garmin Parameters reported.");
  Serial.println("$C0   Clip beyond Sampled Background off.");
  Serial.println("$R?   Display clipper.");
  Serial.println("$Cx   Clipper on. x = anything but '?'.");
  Serial.println("$Rx   x=0:Raindrop filter off. x=1: Raindrop filter on.");
  Serial.println("$R?   Display raindrop filter status.");
  Serial.println("$Rx   Raindrop filter on. x = anything but '0' or '?'.");
  Serial.println(" ");
  Serial.println("------ S-200 Commands active ------");
  Serial.println("$ST   Stop acquiring.  echos $OK*0774");
  Serial.println("$GO   Start acquiring. echos $OK*0774");
  Serial.println(" ");
  Serial.println("------ S-200 Commands, reply 'echoed back' ------");
  Serial.println("------ These are only to fake out laserpp. ------");
  Serial.println("$PO   echos $OK*0774");
  Serial.println("$ID   echos our version of S-200 ID mssg");
  Serial.println("$DM,x   $DT,x   $DI,x  $MM,4,3,M   $MU,M");    
  return;
}

/* Need to build the LTI S-200 CRC if backwards compatibility is a goal. */
// from M. Gerber ..  https://github.com/AllTrafficSolutions/laserpp/blob/master/include/utils.hpp#L258
static unsigned short const crc16_table[256] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
//============================================================================
static inline unsigned short crc16(unsigned short crc, unsigned char const *buffer, unsigned int len)
{
  while (len--)
    crc = (unsigned short)((crc >> 8) ^ (crc16_table[(crc ^ (*buffer++)) & 0xff]));
  return crc;
}
extern uint16_t gen_crc16(const uint8_t *data, uint16_t size);


void SampleBackground(void) {  // This routine samples BKG_STDEV_SAMPLE readings and calcs the mean and standard deviation
  unsigned int n, Distances[2][BKG_STDEV_SAMPLE];
  int Freq, LrgerStDevRnd=0;
  int Sampling = TRUE, loops=0;
  float accumulator[2] = {0,0};
  //noInterrupts(); 
  AvgArrayLen = CalcAvgArrayLen(1); //AvgLevel);
  //digitalWrite(PIN_RedLaser, RED_LASER_ON);
  LaserFlash.begin(INT_LaserFlash, 250000);
  pinMode(PIN_Speaker, OUTPUT);    // Enable speaker.
  //OutDataSent = TRUE;   //  Why does OutDatSent get set TRUE here?  Seems like a mistake.  ?
  for (n=0; n<1024; n++){  // And why waste all these readings? 
    readDistance(0);  delayMicroseconds(10); 
    readDistance(1);  delayMicroseconds(10);
  }
  loops=0;
  Serial.print("Sampling till StdDevs < ");  Serial.println(StdDevSampleCutoff);
  while (Sampling == TRUE){ loops += 1;
    accumulator[0] = 0; accumulator[1] = 0;
    n=0;
    while (n<BKG_STDEV_SAMPLE){
      LL3_Status_0x01[0] = Lidar_Read (0, REG_ADDR_SystemStatus);
      LL3_Status_0x01[0] = LL3_Status_0x01[0] & 0x01;
      LL3_Status_0x01[1] = Lidar_Read (1, REG_ADDR_SystemStatus);
      LL3_Status_0x01[1] = LL3_Status_0x01[1] & 0x01;
      if ( ( LL3_Status_0x01[0] | LL3_Status_0x01[1] ) == 0){
        Distances[0][n] = readDistance(0);  accumulator[0] += Distances[0][n];  
        Distances[1][n] = readDistance(1);  accumulator[1] += Distances[1][n];  
        n++;
      }
    }
    Mean[0] = accumulator[0] / BKG_STDEV_SAMPLE;
    Mean[1] = accumulator[1] / BKG_STDEV_SAMPLE;
    accumulator[0] = 0;
    accumulator[1] = 0;
    for (n=0; n<BKG_STDEV_SAMPLE; n++){
      Distances[0][n] = sq(Distances[0][n] - (Mean[0]));  
      Distances[1][n] = sq(Distances[1][n] - (Mean[1]));  
      accumulator[0] += Distances[0][n];
      accumulator[1] += Distances[1][n];
    }
    for (n=0; n <= 1; n++){
      StdDev[n] = accumulator[n] / (BKG_STDEV_SAMPLE-1);
      accumulator[n] = StdDev[n];  // hold in accumulator to take sqrt. 
      StdDev[n] = sqrt(accumulator[n]);
      FarThreshold[n] = Mean[n] - 6 * StdDev[n];
    }
    #ifdef SAMP_MSG
      if (loops == 1){
        Serial.println("  StdDevs     Means     Thresholds");  
      }
    if(loops > 100 ){ Sampling = FALSE; noTone(PIN_Speaker); 
      Serial.print("Good background not found after "); Serial.println(loops);
    }else{
      Serial.print(StdDev[0]);        Serial.print(",");
      Serial.print(StdDev[1]);        Serial.print(",   ");
      Serial.print(Mean[0]);          Serial.print(",");
      Serial.print(Mean[1]);          Serial.print(",   ");
      Serial.print(FarThreshold[0]);  Serial.print(",");
      Serial.println(FarThreshold[1]);
    }
    #endif
    
    if ( (StdDev[0] < StdDevSampleCutoff ) && (StdDev[1] < StdDevSampleCutoff) ){   // Samples until this condition is met. i.e. samples  
          Sampling = FALSE;  
          PlayTheTune(1);
          noTone(PIN_Speaker);                                // until lasers are targeted on a 'decent' background.
    }else{  // Find larger of the StdDevs to generate an 'aiming tone'.   Low freq = better StdDev.
      if( StdDev[0] >= StdDev[1]){
        LrgerStDevRnd = round(StdDev[0]);
      }else{  
        LrgerStDevRnd = round(StdDev[1]); 
      }
       if (LrgerStDevRnd > 9){ 
          LrgerStDevRnd=9; Freq = 5000;
       }else  Freq = LrgerStDevRnd*300+1000;
       //Serial.print(" SDrnd = "); Serial.print(LrgerStDevRnd);
       //Serial.print("  freq: "); Serial.println(Freq);
       tone(PIN_Speaker, Freq);
       LaserFlash.update(AimLaserFlashRates[LrgerStDevRnd]);
    }
  }
  digitalWrite(PIN_RedLaser, RED_LASER_OFF);
  pinMode(PIN_Speaker, INPUT);    // Disable speaker.
  AvgArrayLen = CalcAvgArrayLen(4);
  //interrupts(); 
  LaserFlash.end();
  digitalWrite(PIN_RedLaser, RED_LASER_OFF);
  return;
}

//#ifdef BENCH_PROTOTYPE
void PlayTheTune(int scale) {  // A tune as a notification of status.
  if (scale >= 2) scale = 2;
  for (int thisNote = 0; thisNote < 8; thisNote++) {        // to calculate the note uSound_duration, take one second divided by the note type. e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 250 / noteDurations[thisNote];
    if (melody[scale * 8 + thisNote] > 1) { // a 0 in the array of frequencies (tones, or notes) means no note to be played, a 'rest' in music.
      tone(PIN_Speaker, melody[scale * 8 + thisNote], noteDuration);    // to distinguish the notes, set a minimum time between them.      // the note's uSound_duration + 30% seems to work well:
    }
    int pauseBetweenNotes = 1.5 * noteDuration;
    if (melody[scale * 8 + thisNote] > 1) {
      digitalWrite(PIN_Teensy_Led, HIGH);
    }  delay(pauseBetweenNotes / 2);
    digitalWrite(PIN_Teensy_Led, LOW);  delay(pauseBetweenNotes / 2);
    noTone(PIN_Speaker);     // stop the tone playing:
  }   //......  END of 'Shave and a Haircut'
}
//#endif



// Revision trail, such as it is.......
// jwb_181226 Re-wrote data buffers to implement ring buffers for signal aquisition with filtering.
// This version has the averaging code in readDistance() commented out.  It will be using the ring buffer in a future rev.
// V 0.5 Added TFmini lidar support.   Multithreading mode though, interferes with serial output timing.
// V 0.6 Revisited spike filter.  Also added detection sensitivity in getdistance()
// V 0.7 Added readback of signal intensity  -- started getting 'hung clock' errors at laserpp (Linux process)
/* V 0.8 Switched out un-needed "intensity" readback parameters.  Increased I2C bus speed from 100Khz to 1 Mhz incrementally.
         With BENCHTOP_PROTOTYPE, (5.1K I2C pullups) 1.2 MHZ was the upper limit.  Reduced pullups to 2.5KOhm
         Timer hangup error still present.  Added test for duplicate timestamps for diagnostics / restart.
         pushed V0.8b out to unit 126644180109 at 12:30 EST 2/19/19 for test.
*/
/* TB_Tester: Rev 0 Modified Pirate Pete placing a second RS-232 adapter on Serial2.
   Serial 1 & 2 to drive RS-232 ports of TB.
   HEAVILY modified code to double array space needed to handle additional channel. */
// Derived from TB_Tester from TB_Monitor, from GarLL3_T32_V_0.8b.ino and previous vers of 2G2ST32x.
// 2G2ST32a - approx 190301 - First version to use new PCB to service 2 Garmins, outputting to two RS-232 ports.
// 2G2ST32d was thought deployable into street test units, but it generates S-200 reset sequences from TB.

// jwb 200119 ...... Thus far, approx 700 hours runtime, no hangups of Garmin/Teensy combinaiton.
// jwb 190219 ...... 'time freeze' error on unit 126644180109 after deployed with USB hub.  (could be
//  coincidence, but benchtop prototype has not had this 'time freeze' issue.
// jwb 190306 First dual Garmin unit noted experiencing 'time freeze' issue.
// jwb 190404 Implemented 'flatline' detection as a once-removed means of I2C bus lockup condition.  Issue hard reset when this happens.
              // A non-ideal (big hammer) solution until the root cause of the I2C hangup is known. 
// jwb 190505 - Have previously implemented calc of standard deviation within baseline as a means to detect
              // poor target / range too far baseline noise.  Also added support for a 'dongle' with a small 
              // speaker to give audio feedback if distance is outside some number of stddevs of background mean.
/* jwb 190516 Version 2G2ST32_190514_V3.1 incorporates supression (by substitution of last valid reading) in the case of
 *                                        1. 'laser busy' (due typically to LL3 inability to resolve poor target. 
 *                                        2. 'unreasonable reading beyond background', i.e. laser for some reason has returned 
 *                                           a reading beyond 5 standard deviations of the sampled background.
 */
/* jwb 190522 Lets USB connected master toggle data stream to USB on and off, added initial audio feedback for aiming, 
 * Added feedback beep to test command echo through ssh/USB.   Took out the code from DEV-DONGLE.  */
// jwb 190528 - cleaned project, removed redundant and extraneous files GarminModes.h and PutAside.cpp
/* jwb 190529 - adding back HW test points for timing verification & to debug 70mS errant reads.     
                HW timing does not seem to be the issue.  Timeing is verified 100Hz ISR freq, I2C I/O 800uS-1mS depending
                on LL3 busy status (800uS = busy, 1mS if ready to read (complete read takes additional time) */  
// jwb 190531 - Added code to allow all ISR & S-200 timestamp variables to be set from IRQ_WRITE_PERIOD_uS.  
//             Interesting observation:  60-70mS logging freq of vlogs foes away, at least in todays sampling.

// jwb 190815 added audible feedback to acquire good target.  Good target is a background with adequately
// high repeatability (low std dev) so errant 'long' readings can be removed.
// jwb 190911 re-introduced 'pop' filter, new code, not the code removed on 190528. 
 
