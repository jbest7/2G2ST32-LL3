// 190306 - lidar_IO.cpp
// Broke out Garmin I/O routines.  All I2C to Garmins through here.
// void Init_LIDAR(int SelectLidar)    Called by ReSetup(), which is called by Setup()
#include "2G2ST32.h"
#include "lidar_IO.h"
//#include "GarminModes.h"
#include <string.h>
#include <i2c_t3.h>

extern unsigned long LoopsLow, LoopsHigh;
extern int Errors_Till_CPU_RESTART;
static int intraReadDelay = 2, interReadDelay = 100;

char ErrBuf[32];
int laserSNnow[2]= {17,18};  // init to 0001 0001 & 0001 0010 just for ref.
unsigned char LidarAddress[2] = {LIDARLite_ADDRESS, LIDARLite_ADDRESS_2};  //index 0 is 0x65, the 'new' address.

uint8_t Lidar_Write(int Select, uint8_t Register, uint8_t dataOut){
  int nackack = 99;
  noInterrupts();
  while (nackack != 0) {
    //digitalWrite(PIN_TestPoint_1, HIGH); delayMicroseconds(1); digitalWrite(PIN_TestPoint_1, LOW);
    Wire.beginTransmission(LidarAddress[Select]); 
    delayMicroseconds(intraReadDelay);  
    Wire.write(Register);      //#define REG_ADDR_Soft_Reset 0x5c  //190327 From Dennis Corey of Garmin.  Soft "partial reset' complete functionality and states unknown at this time.
    delayMicroseconds(intraReadDelay);  
    Wire.write(dataOut);               //#define Soft_Reset 0xff   // Data to invoke 'full' soft reset.
    nackack = Wire.endTransmission(I2C_STOP, 1000);
    Error_I2C_LIDAR (nackack, Select);            // i2c error processing
  } 
  interrupts();
  delayMicroseconds(interReadDelay);
  return 0;
}
uint8_t Lidar_Read (int Select, uint8_t Register){
    int nackack = 100;
    noInterrupts();
    while (nackack != 0) {   
      //digitalWrite(PIN_TestPoint_1, HIGH); 
      Wire.beginTransmission(LidarAddress[Select]);  
      delayMicroseconds(intraReadDelay);  
      Wire.write(Register);
      delayMicroseconds(intraReadDelay);  
      nackack = Wire.endTransmission(I2C_STOP, 1000);
      Error_I2C_LIDAR (nackack, Select);                      //For now, to echo errors returned from LIDAR to serial monitor.
      //digitalWrite(PIN_TestPoint_1, LOW);
    }
    Wire.requestFrom(LidarAddress[Select], 1, I2C_STOP, 1000 ); //Request 1 bytes from slave (the LIDAR)
    uint8_t return_uint8 = Wire.readByte();
    interrupts();                        
    delayMicroseconds(interReadDelay);  
    return return_uint8;
}


int readDistance(int SelectLidar) {   // Request and receive LIDAR distance return by I2C.  First write to initiate a read , then request and get a read.
  // readDistance takes about 800 uS with 72MHz clock & 400KHz I2C.
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses
  noInterrupts();
  // Setup bias correction
  Lidar_Write(SelectLidar, REG_ADDR_Acq_Rst, Acq_Rst_AcqWithBiasCorr);   // Alternately, Acq_Rst_AcqWithNObiasCorr
  // Acquisition mode, high speed, high noise, or low speed-low noise.  
  // Lidar_Write(SelectLidar, REG_ADDR_DetSens, DetSens_LoSens_LoError);             //DetSens_LoSens_LoError = 0x60 Could also be DetSens_HiSens_HiError 0x20.
  // This transaction returns additional data about the reading. REG_ADDR_Signal_Strength=0x0e    REG_ADDR_Peak)=0x0c or     REG_ADDR_Noise_Peak=0x0d  
  // SigInt[SelectLidar] = Lidar_Read(SelectLidar, REG_ADDR_Noise_Peak);
  // Get distance in two bytes
  nackack = 100;              
  while (nackack != 0) {  
    Wire.beginTransmission(LidarAddress[SelectLidar]);
    Wire.write(RegisterHighLowB);                   // Set up Garmin to return data as two sequential I2C reads.
    nackack = Wire.endTransmission(I2C_STOP, 1000);
    Error_I2C_LIDAR (nackack, SelectLidar);
  }
  //delayMicroseconds(intraReadDelay);
  Wire.requestFrom(LidarAddress[SelectLidar], 2, I2C_STOP, 1000 ); //Request 2 bytes
  //delayMicroseconds(intraReadDelay);  
  int reading = Wire.readByte() ;
  reading = reading << 8;
  reading |= Wire.readByte();
  //delayMicroseconds(interReadDelay);
  interrupts();
  return reading;
}

int Reset_Lidar_Soft(int WhichLidar){
  Lidar_Write(WhichLidar, REG_ADDR_Soft_Reset, Soft_Reset_ALL); // send 0xff to 0x5c
  DEBUG_reading_0x66_Status[WhichLidar] = Lidar_Read(WhichLidar, REG_ADDR_0x66_Status);
  return DEBUG_reading_0x66_Status[WhichLidar];  
}

int GetLaserStat_0x66(int WhichLaser){
  DEBUG_SN_HighByte[WhichLaser] = Lidar_Read(WhichLaser, UNIT_ID_HIGH);
  DEBUG_SN_LowByte[WhichLaser] = Lidar_Read(WhichLaser, UNIT_ID_LOW);
  DEBUG_reading_0x66_Status[WhichLaser] = Lidar_Read(WhichLaser, REG_ADDR_0x66_Status);
  return DEBUG_reading_0x66_Status[WhichLaser];
}

int Error_I2C_LIDAR (int ErrCode, int whichLaser) {   // Code to branch calling routine toward recovery from error.
  static int ErrBufPtr = 0, ReInitMode= FALSE;
  
  //digitalWrite(PIN_TestPoint_1, HIGH);
    if ( ReInitMode == TRUE ){ //reinitialize buffer, it's a new transaction
      ReInitMode = FALSE;  strcpy (ErrBuf, "-");  ErrBufPtr = 0;   Errors_Till_CPU_RESTART = 0;
    }
    if (ErrCode == 0){  //Successful I2C, so reset routine to begin acquiring a new ErrBuf.
        ReInitMode = TRUE;           ErrBufPtr = 0;  Errors_Till_CPU_RESTART = 0;
    }else{ 
      ErrBuf[ErrBufPtr++] = ErrCode; 
      if( ErrBufPtr >= 30 ) { ErrBufPtr = 30; } // To prevent buffer overrun in case of a large number of errors.
      Errors_Till_CPU_RESTART += 1;
    }      
  #ifdef DEBUG_USB_VERBOSE 
      //for(int q = 0; q< ErrBufPtr; q++) Serial.print(ErrBuf[q]); 
      //Serial.print(","); 
     // if(ErrCode != 0) {Serial.print("L:");  Serial.print(LidarAddress[whichLaser]);  Serial.print("  I2C Er: "); }
    switch (ErrCode) {
      case 0:  break;  //success, continue I2C transaction by reading the data next.
      case 1:  Serial.print(ErrCode); Serial.println(" - Data too long");       break;
      case 2:  Serial.println (ErrCode);  //Serial.println(" - Rcvd ADDR NACK");      
                                                                                break;
      case 3:  Serial.print(ErrCode); Serial.println(" - Rcvd DATA NACK");      break;
      case 4:  Serial.print(ErrCode); Serial.println(" - Other Error ");        break;
      default: Serial.print(ErrCode); Serial.print(" - WAY Other Error");       break;
    }
  #endif  
      //digitalWrite(PIN_TestPoint_1, LOW);
  return Errors_Till_CPU_RESTART;
}



int MoveLidar_Address(void) {
  uint8_t SN_HighByte, SN_LowByte;  //to hold the address values that must be moved.
  //delay(150);  // just to give I2C initialization time?  I2C reports ADDR NACK right after initialization 190403
  digitalWrite(PIN_LIDAR_1_PWR_ENAB, LOW);    // P12, J4 // delay(150);   // LOW = DISABLE, This one will use the default address 0x62
  digitalWrite(PIN_LIDAR_2_PWR_ENAB, HIGH);   // Teensy DIO 15, dest -> J3, Pin 2, Orn (enable) of LL3  
  delay(150);   
           
  Init_LIDAR(0);  // Initializes L3 at default addr 0x62, since initialization would init the address to 0x62 and then fail.
  Serial.println("Moving default to alternate");
  SN_HighByte = Lidar_Read (0, UNIT_ID_HIGH);      // First read back serial number, high byte
  //Serial.print("SN: ");  Serial.print(SN_HighByte); 
  SN_LowByte = Lidar_Read (0, UNIT_ID_LOW);        // Read back second byte of SN
  //Serial.print(" : ");  Serial.println(SN_LowByte);
  Lidar_Write(0, REG_ADDR_I2C_Addr_Unlock_Enab_HIGH,SN_HighByte);  //Write high and low bytes to unlock enab registers.
  Lidar_Write(0, REG_ADDR_I2C_Addr_Unlock_Enab_LOW, SN_LowByte); 
  Lidar_Write(0, REG_ADDR_I2C_New_Addr, LIDARLite_ADDRESS_2); 
  Lidar_Write(0, REG_ADDR_I2C_Unlock, I2C_Unlock);   // 0x08 to 0x1e to start using new address

  // Now the other LIDAR can be enabled and there should be one at I2C 0x62, and 0x65
  //delay(3000);  // Wait 3 seconds to read what's been sent to the serial monitor.

  delay(25);
  digitalWrite(PIN_LIDAR_1_PWR_ENAB, HIGH);  delay(50);  // Enable second L3, which is at default I2C address and will stay there.
  Init_LIDAR(0);   // This is the LIDAR farthest from the Teensy.  It's address is still at the default 0x62.
  delay(50);
  #ifdef DEBUG_USB
    SN_HighByte = Lidar_Read(0, REG_ADDR_I2C_New_Addr);
    Serial.print ("SB 0x62 ( 98 dec): ");  Serial.println (SN_HighByte);
    delay(50);
    SN_HighByte = Lidar_Read(1, REG_ADDR_I2C_New_Addr);
    Serial.print ("SB 0x66 (102 dec): ");  Serial.println (SN_HighByte);
    //delay(500);
  #endif 
  return 0;  //Make into a success
}

int GetFWsn (int WhichLidar){
    uint8_t reading_High_FW_ADDR = Lidar_Read(WhichLidar, RED_ADDR_FWRev_High); 
    uint8_t reading_Low_FW_ADDR = Lidar_Read(WhichLidar, RED_ADDR_FWRev_Low);
    //Serial.print("LL3:"); Serial.print(WhichLidar); 
    //Serial.print(" FW:"); Serial.print(reading_Low_FW_ADDR);  Serial.print("."); Serial.println(reading_High_FW_ADDR);
    return (reading_High_FW_ADDR << 8) + reading_Low_FW_ADDR;
}

void Init_LIDAR(int SelectLidar) {    // Called by ReSetup(), which is called by Setup()

  #ifdef DEBUG_USB 
    GetFWsn(0);
  #endif
  
  Lidar_Write(0, REG_ADDR_Acq_Rst, Acq_Rst_RESET_ALL);                // 0x00 to 0x00

  Lidar_Write(0, REG_ADDR_Acq_Rst, Acq_Rst_AcqWithBiasCorr);          // 0x04 to 0x00 
  uint8_t reading_REG_0x00 = Lidar_Read(0, REG_ADDR_Acq_Rst);         // see if 0x04 is in 0x00
  
  Lidar_Write(0, REG_ADDR_SigCountVal, 0x04);                          // Max counts 128   //190416 jwb WAS 0x80
  uint8_t reading_MaxAcqCnt = Lidar_Read(0, REG_ADDR_SigCountVal);
  
  Lidar_Write(0, REG_ADDR_Acquisition_mode_control, Acquisition_mode_control_TerminateEarly);   // 0x00 to 0x04
  uint8_t reading_AcqModeCtl = Lidar_Read(0, REG_ADDR_Acquisition_mode_control);
  
  Lidar_Write(0, REG_ADDR_BurstMode, BurstMode_NoRepeats);          //0x00 to 0x11 Single read, single return, no repeats
  uint8_t reading_BurstMode = Lidar_Read(0, REG_ADDR_BurstMode);
  
  Lidar_Write(0, REG_ADDR_RefAcqCnt, RefAcqCnt_Default);            // 0x05 to 0x12 
  uint8_t reading_RefAcqCnt = Lidar_Read(0, REG_ADDR_RefAcqCnt);
  
  uint8_t reading_SNbyteHigh = Lidar_Read(0, UNIT_ID_HIGH);
  uint8_t reading_SNbyteLow = Lidar_Read(0, UNIT_ID_LOW);
  
  Lidar_Write(0, REG_ADDR_DetSens, DetSens_LoSens_LoError);  // 0x60 to 0x1c   
  uint8_t reading_DetSens = Lidar_Read(0, REG_ADDR_DetSens);
  
  #ifdef DEBUG_USB 
    Serial.print("Acq_Rst        0x00 = "); Serial.println(reading_REG_0x00);
    Serial.print("Max Acq Counts 0x02 = "); Serial.println(reading_MaxAcqCnt);
    Serial.print("Acq Mode from  0x04 = "); Serial.println(reading_AcqModeCtl);
    Serial.print("BurstMode from 0x11 = "); Serial.println(reading_BurstMode);
    Serial.print("Acq count from 0x12 = "); Serial.println(reading_RefAcqCnt);
    Serial.print("High SN byte   0x16 = "); Serial.println(reading_SNbyteHigh);
    Serial.print("Low SN byte    0x17 = "); Serial.println(reading_SNbyteLow);
    Serial.print("Detect Sens    0x1c = "); Serial.println(reading_DetSens);
    //Setup Detection Sensitivity  (register 0x1c, also THRESHOLD_BYPASS)
    Serial.println("Exit Init_LIDAR()");
    Serial.println(" ");
  #endif
  
}

// For LIDAR register diagnostics.  Not intended to be used in prod fw.
void DumpHeader(void){
  //char ReturnBuf[132];
  /* 
  Serial.print("L ");       Serial.print(","); 
  for (int n=0; n<127; n++){
    sprintf(ReturnBuf, "%2X", n);
    Serial.print(ReturnBuf); 
    Serial.print(",");   
  }
  sprintf(ReturnBuf, "%2X", 127);
  Serial.println(ReturnBuf);   */
return;
  }

void DumpLidarRegisters(int SelectLidar){
  char registerContent, ReturnBuf[8];
  uint8_t  n=0;
  for (n=0; n<=127; n++){
    registerContent = Lidar_Read(SelectLidar, n);
      sprintf(ReturnBuf, "%2X", registerContent);
      // Serial.print(ReturnBuf); 
  }
  return; 
}


/* Registers to change default I2C addresses
    Available addresses are 7 bit values with a '0' at the LSB. i.e. even hex numbers
    1. Read high and low address bytes from 0x16 and 0x17
    2. Write high byte to 0x18
    3. Write low byte to 0x19
    4. Write new I2C address to 0x1a
    5. Write 0x08 to 0x1e to disable old I2C address
    The default I2C addresses will be restored after a power cycle.

  #define UNIT_ID_HIGH 0x16     // Read serial number high byte
  #define UNIT_ID_LOW  0x17     // Read serial number low byte
  #define REG_ADDR_I2C_Addr_Unlock_Enab_HIGH  0x18     // Write high byte for SN unlock
  #define REG_ADDR_I2C_Addr_Unlock_Enab_LOW   0x19     // Write low  byte for SN unlock
  #define REG_ADDR_I2C_New_Addr 0x1a     // Write new I2C address after unlock
  #define REG_ADDR_I2C_Unlock   0x1e     // Write 0x08 to 0x1e to disable default (0x62) address
  ******************************/
