#define FW_VER 190916
#define FW_SUBVER 800     //This subver with default averaging 32 reads enabled 
/*************************************************************************************************************/
// #define BENCH_PROTOTYPE     // FOR TESTING ON BENCHTOP ONLY, COMMENT THIS OUT BEFORE PUSHING TO FIELD TEST UNIT 
#define LIFACE_2               // Lidar Interface 2.0, Jan 2019 PCB, WJP
#define WD_MSG                 // msg every minute.
#define SAMP_MSG               // messages during background sampling.
#define DEBUG_USB              // Debug info through USB port -- Uncomment for no output to Serial (USB)
//#define DEBUG_USB_VERBOSE   // For I2C transactional data, Also sends to Serial (USB)
//#define DEBUG_PINS 
// #define CHECK_0x66          // check & print laser 0x66 status register
// #define SINGLE_STREAM_USB   // Prototype single laser stream to Serial (USB port)
 /*************************************************************************************************************/

/*******************************************/
#define IRQ_WRITE_PERIOD_uS 10000  // 10000 uS yields 100Hz ISR
#define BKG_STDEV_SAMPLE 128       // Sample time = IRQ_WRITE_PERIOD_uS * BKG_STDEV_SAMPLE
/*******************************************/
// For soft restart .........
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)
// call CPU_RESTART; from code.

#ifdef DEBUG_PINS
  //#define PIN_TestPoint_1 (17)
  #define PIN_TestPoint_2 (20)
#endif

#ifdef BENCH_PROTOTYPE    
  #include "pitches.h" // junk for indicator tones on the prototype
  #define PIN_LIDAR_1_PWR_ENAB (17) // Used once during seup to move one LIDAR to the secondary address.
  #define PIN_LIDAR_2_PWR_ENAB (16) // used once in setup.
  // UltraSound Xducer and speaker
  #define PIN_Speaker (17)
  // #define PIN_Speaker_Out_2 (22)
  #define PIN_TestPoint_1 (2)
  #define PIN_TestPoint_2 (3)
  #define PIN_trig  (5)
  #define PIN_echo  (6)
  #define PIN_LIDAR_1_MODE_CTL PIN_RedLaser //  
  #define PIN_LIDAR_2_MODE_CTL PIN_RedLaser //  
  int LaserStatus = RED_LASER_ON;
#endif
#ifdef  LIFACE_2   // Jan 2019 PCA.
  #define PIN_LIDAR_1_PWR_ENAB (12) // LL3 on J4. Used once during seup to move one LIDAR to the secondary address.
  #define PIN_LIDAR_1_MODE_CTL (11) // Goes on left facing unit, associated with BLUE LED
  #define PIN_LIDAR_2_PWR_ENAB (15) // LL3 on J3 of PCB -  
  #define PIN_LIDAR_2_MODE_CTL (14) // Goes on right facing unit, associated with RED LED
  //#define PIN_TestPoint_1 (17)
  #define PIN_TestPoint_2 (20)
  #define PIN_Red_LED (2)
  #define PIN_Grn_LED (3)
  #define PIN_Speaker (17)
#endif

//Common pins to Bills Interface III, (with on-board regulator) and breadboard prototype.
#define PIN_RedLaser (23)     
#define RED_LASER_ON (0)  //LOW      //laser red sourced from +5, pull blk wire low to turn on.
#define RED_LASER_OFF (1) //HIGH
#define PIN_Teensy_Led  (13)    // Can be seen from outside PkTk-E, but not D.
