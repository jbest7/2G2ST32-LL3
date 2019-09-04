//extern int Lidar_Write(int, uint8_t, uint8_t);
//extern int Lidar_Read (int Select, uint8_t Register);
//extern int GetReadingORBusy(int);
extern void DumpLidarRegisters(int SelectLidar);
extern int Error_I2C_LIDAR (int, int);
extern int MoveLidar_Address(void);
extern void Init_LIDAR(int SelectLidar);
extern int readDistance(int SelectLidar);
extern int Reset_Lidar_Soft(int WhichLidar);
extern void DumpHeader(void);
extern int GetFWsn (int WhichLidar);

extern int DEBUG_SN_HighByte[2], DEBUG_SN_LowByte[2], DEBUG_reading_0x66_Status[2];
extern unsigned char LidarAddress[2];
extern volatile int SigInt[2];
extern int PreviousReading[2];

/* Garmin LIDARLite V1 I2C Register I/O & notes */
// 181220 - Rev H version adds multiple LIDARS on the I2C bus.
#define TRUE (1)
#define FALSE (0)

#define LIDARLite_ADDRESS 0x62    // 01100010 Default I2C Address of LIDAR-Lite.
#define LIDARLite_ADDRESS_2 0x66  // 01100110 Secondary address

// Main acquisition initiate register.  
#define REG_ADDR_Acq_Rst 0x00            // Register to write to initiate ranging.
#define Acq_Rst_RESET_ALL 0x00           // Perhaps due to initiate ranging.
#define Acq_Rst_AcqWithNObiasCorr 0x03   // Take distance measurement without receiver bias correction
#define Acq_Rst_AcqWithBiasCorr 0x04     // Take distance measurement with receiver bias correction

#define REG_ADDR_Acquisition_mode_control 0x04 // Quick termination on/off  R/W bit.
#define Acquisition_mode_control_TerminateEarly 0x00
// Bit 3 is the active bit, the initial value is 0x08, which is quick termination = off.
// Setting to 0x00 will allow device to terminate measurement early 
// "if it anticipated the signal peak in the correlation record will reach mazximum value."
// Clearing the bit results in "faster and slightly less accurate operation at strong signal strengths.

#define REG_ADDR_Signal_Strength 0x0e // Signal strength 
#define REG_ADDR_Peak 0x0c  // Peak Value in Correlation record
#define REG_ADDR_Noise_Peak 0x0d      // Noise in the Correlation record
#define REG_ADDR_SystemStatus 0x01    // System status register

#define RegisterHighLowB 0x8f         // Register to get both High and Low bytes in 1 call.

// "Maximum Acquisition Count" per datasheet.  
#define REG_ADDR_SigCountVal 0x02 

// Mesurement Quick Termination Detection
// "Detection Sensitivity" parameter per datasheet.
#define REG_ADDR_DetSens 0x1c        // trade-off between noise and sensitivity.  0x00 is default
// These are "recommended values", criteria of recommendation unknown.
#define DetSens_Default 0x00
#define DetSens_HiSens_HiError 0x20        // low non zero values increase sensitivity, with increased errors
#define DetSens_LoSens_LoError 0x60        // higher values reduce sensitivity and reduce errors.   

#define REG_ADDR_Soft_Reset 0x5c  //190327 From Dennis Corey of Garmin.  Soft "partial reset' complete functionality and states unknown at this time.
#define Soft_Reset_ALL 0xff           // Data to invoke 'full' soft reset.
#define REG_ADDR_0x66_Status 0x66      //190327 From Dennis Corey of Garmin.  A status register, complete functionality and states unknown at this time.

#define REG_ADDR_BurstMode 0x11  // Also, 'outer loop count'
#define BurstMode_NoRepeats  0x00  // single measurement single return.
#define BurstMode_NumRepeats 0x80  // 128 repeats. 

#define REG_ADDR_RefAcqCnt 0x12                    // Reference  inti val s/b 0x05
#define RefAcqCnt_Default 0x05
#define RegisterHighLowB 0x8f      // Register to get both High and Low bytes in 1 call.
#define REG_ADDR_SigCountVal 0x02  // "Maximum Acquisition Count" per datasheet.  

/* Registers to change default I2C addresses
 *  Available addresses are 7 bit values with a '0' at the LSB. i.e. even hex numbers
 *  1. Read high and low address bytes from 0x16 and 0x17
 *  2. Write high byte to 0x18
 *  3. Write low byte to 0x19
 *  4. Write new I2C address to 0x1a
 *  5. Write 0x08 to 0x1e to disable old I2C address
 *  The default I2C addresses will be restored after a power cycle.
 */


#define UNIT_ID_HIGH 0x16                          // Read serial number high byte
#define UNIT_ID_LOW  0x17                 
#define REG_ADDR_I2C_Addr_Unlock_Enab_HIGH  0x18   // Write hi byte for SN unlock
#define REG_ADDR_I2C_Addr_Unlock_Enab_LOW   0x19 
#define REG_ADDR_I2C_New_Addr 0x1a                 // Write new I2C address after enabling unlock
#define REG_ADDR_I2C_Unlock   0x1e                          // Enable new I2C address by writing 0x08 to 0x1e
#define I2C_Unlock 0x08

// System status register
#define REG_ADDR_SystemStatus 0x01       //Register from which BUSY status is read 
/* Bit - Function    See Garmin spec sheet, 190-02088-00_0a, Page 8.
 *  0     Busy.                   0 = not busy.   if 1, device is taking measurement.      
 *  1     Refrence overflow       0 = no overflow
 *  2     Signal overflow         0 = no overflow
 *  3     Invalid Signal Flag     0 = peak detected, OK.  1 = no peak, measurement invlaid
 *  4     Secondary return flag.  0 = no secondary return detected
 *  5     Health flag             0 = ERROR.  1 = Reference and receiver bias are operational
 *  6     Process error flag      0 = no error.  1 = system error detected during measurement. 
 */

#define RED_ADDR_FWRev_High 0x72   // High FW rev byte
#define RED_ADDR_FWRev_Low 0x73   // Low FW rev byte
 
