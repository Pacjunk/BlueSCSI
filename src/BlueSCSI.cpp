/*  
 *  BlueSCSI
 *  Copyright (c) 2021  Eric Helgeson, Androda
 *  
 *  This file is free software: you may copy, redistribute and/or modify it  
 *  under the terms of the GNU General Public License as published by the  
 *  Free Software Foundation, either version 2 of the License, or (at your  
 *  option) any later version.  
 *  
 *  This file is distributed in the hope that it will be useful, but  
 *  WITHOUT ANY WARRANTY; without even the implied warranty of  
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
 *  General Public License for more details.  
 *  
 *  You should have received a copy of the GNU General Public License  
 *  along with this program.  If not, see https://github.com/erichelgeson/bluescsi.  
 *  
 * This file incorporates work covered by the following copyright and  
 * permission notice:  
 *  
 *     Copyright (c) 2019 komatsu   
 *  
 *     Permission to use, copy, modify, and/or distribute this software  
 *     for any purpose with or without fee is hereby granted, provided  
 *     that the above copyright notice and this permission notice appear  
 *     in all copies.  
 *  
 *     THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL  
 *     WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  
 *     WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE  
 *     AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR  
 *     CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS  
 *     OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,  
 *     NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN  
 *     CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.  
 */

#include <Arduino.h> // For Platform.IO
#include <SdFat.h>

#ifdef USE_STM32_DMA
#warning "warning USE_STM32_DMA"
#endif

#define VERSION "2.0-PACJUNK(220206)"
#define DEBUG            0      // 0:No debug information output
                                // 1: Debug information output to USB Serial
                                // 2: Debug information output to LOG.txt (slow)

#define USE_DB2ID_TABLE      1 // Use table to get ID from SEL-DB

// SCSI config
#define NUM_SCSIID  7          // Maximum number of supported SCSI-IDs (The minimum is 0)
#define NUM_SCSILUN 2          // Maximum number of LUNs supported     (The minimum is 0)
#define READ_PARITY_CHECK 0    // Perform read parity check (unverified)

// HDD format
#define MAX_BLOCKSIZE 8192     // Maximum BLOCK size

// SDFAT
#define SD1_CONFIG SdSpiConfig(PA4, DEDICATED_SPI, SPI_FULL_SPEED, &SPI)
SdFs SD;

#if DEBUG == 1
#define LOG(XX)     Serial.print(XX)
#define LOGHEX(XX)  Serial.print(XX, HEX)
#define LOGN(XX)    Serial.println(XX)
#define LOGHEXN(XX) Serial.println(XX, HEX)
#elif DEBUG == 2
#define LOG(XX)     LOG_FILE.print(XX); LOG_FILE.sync();
#define LOGHEX(XX)  LOG_FILE.print(XX, HEX); LOG_FILE.sync();
#define LOGN(XX)    LOG_FILE.println(XX); LOG_FILE.sync();
#define LOGHEXN(XX) LOG_FILE.println(XX, HEX); LOG_FILE.sync();
#else
#define LOG(XX)     //Serial.print(XX)
#define LOGHEX(XX)  //Serial.print(XX, HEX)
#define LOGN(XX)    //Serial.println(XX)
#define LOGHEXN(XX) //Serial.println(XX, HEX)
#endif

#define active   1
#define inactive 0
#define high 0
#define low 1

#define isHigh(XX) ((XX) == high)
#define isLow(XX) ((XX) != high)

#define gpio_mode(pin,val) gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, val);
#define gpio_write(pin,val) gpio_write_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, val)
#define gpio_read(pin) gpio_read_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit)

//#define DB0       PB8     // SCSI:DB0
//#define DB1       PB9     // SCSI:DB1
//#define DB2       PB10    // SCSI:DB2
//#define DB3       PB11    // SCSI:DB3
//#define DB4       PB12    // SCSI:DB4
//#define DB5       PB13    // SCSI:DB5
//#define DB6       PB14    // SCSI:DB6
//#define DB7       PB15    // SCSI:DB7
//#define DBP       PB0     // SCSI:DBP
#define ATN       PA8      // SCSI:ATN
#define BSY       PA9      // SCSI:BSY
#define ACK       PA10     // SCSI:ACK
#define RST       PA15     // SCSI:RST
#define MSG       PB3      // SCSI:MSG
#define SEL       PB4      // SCSI:SEL
#define CD        PB5      // SCSI:C/D
#define REQ       PB6      // SCSI:REQ
#define IO        PB7      // SCSI:I/O
#define LED2      PA0      // External LED

#define SD_CS     PA4      // SDCARD:CS
#define LED       PC13     // LED

// GPIO register port
#define PAREG GPIOA->regs
#define PBREG GPIOB->regs

// LED control
#define LED_ON()       gpio_write(LED, high); gpio_write(LED2, low);
#define LED_OFF()      gpio_write(LED, low); gpio_write(LED2, high);

// Virtual pin (Arduio compatibility is slow, so make it MCU-dependent)
#define PA(BIT)       (BIT)
#define PB(BIT)       (BIT+16)
// Virtual pin decoding
#define GPIOREG(VPIN)    ((VPIN)>=16?PBREG:PAREG)
#define BITMASK(VPIN) (1<<((VPIN)&15))

#define vATN       PA(8)      // SCSI:ATN
#define vBSY       PA(9)      // SCSI:BSY
#define vACK       PA(10)     // SCSI:ACK
#define vRST       PA(15)     // SCSI:RST
#define vMSG       PB(3)      // SCSI:MSG
#define vSEL       PB(4)      // SCSI:SEL
#define vCD        PB(5)      // SCSI:C/D
#define vREQ       PB(6)      // SCSI:REQ
#define vIO        PB(7)      // SCSI:I/O
#define vSD_CS     PA(4)      // SDCARD:CS

// SCSI output pin control: opendrain active LOW (direct pin drive)
#define SCSI_OUT(VPIN,ACTIVE) { GPIOREG(VPIN)->BSRR = BITMASK(VPIN)<<((ACTIVE)?16:0); }

// SCSI input pin check (inactive=0,avtive=1)
#define SCSI_IN(VPIN) ((~GPIOREG(VPIN)->IDR>>(VPIN&15))&1)

// GPIO mode
// IN , FLOAT      : 4
// IN , PU/PD      : 8
// OUT, PUSH/PULL  : 3
// OUT, OD         : 1
//#define DB_MODE_OUT 3
#define DB_MODE_OUT 1
#define DB_MODE_IN  8

// Put DB and DP in output mode
#define SCSI_DB_OUTPUT() { PBREG->CRL=(PBREG->CRL &0xfffffff0)|DB_MODE_OUT; PBREG->CRH = 0x11111111*DB_MODE_OUT; }
// Put DB and DP in input mode
#define SCSI_DB_INPUT()  { PBREG->CRL=(PBREG->CRL &0xfffffff0)|DB_MODE_IN ; PBREG->CRH = 0x11111111*DB_MODE_IN;  }

// Turn on the output only for BSY
#define SCSI_BSY_ACTIVE()      { gpio_mode(BSY, GPIO_OUTPUT_OD); SCSI_OUT(vBSY,  active) }
// BSY,REQ,MSG,CD,IO Turn on the output (no change required for OD)
#define SCSI_TARGET_ACTIVE()   { }
// BSY,REQ,MSG,CD,IO Turn off output, BSY is the last input
#define SCSI_TARGET_INACTIVE() { SCSI_OUT(vREQ,inactive); SCSI_OUT(vMSG,inactive); SCSI_OUT(vCD,inactive);SCSI_OUT(vIO,inactive); SCSI_OUT(vBSY,inactive); gpio_mode(BSY, GPIO_INPUT_PU); }

#define NOP(x) for(unsigned _nopcount = x; _nopcount; _nopcount--) { asm("NOP"); }
#define SCSI_BUS_SETTLE() NOP(10);                            // spec 400ns ours ~420us
#define SCSI_PHASE_DATA_OUT()   PBREG->BSRR = 0b00000000000000000000000010101000; SCSI_BUS_SETTLE();
#define SCSI_PHASE_DATA_IN()    PBREG->BSRR = 0b00000000100000000000000000101000; SCSI_BUS_SETTLE();
#define SCSI_PHASE_COMMAND()    PBREG->BSRR = 0b00000000001000000000000010001000; SCSI_BUS_SETTLE();
#define SCSI_PHASE_STATUS()     PBREG->BSRR = 0b00000000101000000000000000001000; SCSI_BUS_SETTLE();
#define SCSI_PHASE_MSG_OUT()    PBREG->BSRR = 0b00000000001010000000000010000000; SCSI_BUS_SETTLE();
#define SCSI_PHASE_MSG_IN()     PBREG->BSRR = 0b00000000101010000000000000000000; SCSI_BUS_SETTLE();
#define SCSI_DESKEW() asm("NOP"); asm("NOP"); asm("NOP");     // spec 45ns ours ~42ns
#define SCSI_CABLE_SKEW() asm("NOP");                         // spec 10ns ours ~14ns
#define SCSI_RESET_HOLD() asm("NOP"); asm("NOP");             // spec 25ns ours ~28ns
#define SCSI_DISCONNECTION_DELAY() NOP(15);                   // spec 200ns ours ~210ns

#define SCSI_TYPE_HDD     0
#define SCSI_TYPE_CDROM   5

// HDDimage file
#define HDIMG_ID_POS  2                 // Position to embed ID number
#define HDIMG_LUN_POS 3                 // Position to embed LUN numbers
#define HDIMG_BLK_POS 5                 // Position to embed block size numbers
#define MAX_FILE_PATH 32                // Maximum file name length

#define DEFAULT_VENDOR  "QUANTUM "
#define DEFAULT_PRODUCT "FIREBALL1       "
#define DEFAULT_VERSION "1.0 "
#define LEN_VENDOR 8
#define LEN_PRODUCT 16
#define LEN_VERSION 4

struct SCSI_INQUIRY_DATA
{
  union
  {
  struct {
    byte device_type;
    byte rmb;
    byte ansi_version;
    byte response_format;
    byte additional_length;
    byte reserved_byte5;
    byte reserved_byte6;
    byte sync;
    char vendor[LEN_VENDOR];
    char product[LEN_PRODUCT];
    char revision[LEN_VERSION];
  };
  // raw bytes
  byte raw[36];
  };
};

// HDD image
typedef struct hddimg_struct
{
	FsFile            m_file;                 // File object
	uint64_t          m_fileSize;             // File size
	size_t            m_blocksize;            // SCSI BLOCK size
  byte              m_devtype;              // SCSI device type
  SCSI_INQUIRY_DATA m_inquiry;              // Inquiry data

}HDDIMG;

HDDIMG  img[NUM_SCSIID][NUM_SCSILUN]; // Maximum number

uint8_t     m_senseKey = 0;           // Sense key
unsigned    m_addition_sense = 0;     // Additional sense information
volatile bool m_isBusReset = false;   // Bus reset

byte          scsi_id_mask;           // Mask list of responding SCSI IDs
byte          m_id;                   // Currently responding SCSI-ID
byte          m_lun;                  // Logical unit number currently responding
byte          m_sts;                  // Status byte
byte          m_msg;                  // Message bytes
HDDIMG       *m_img;                  // HDD image for current SCSI-ID, LUN
byte          m_buf[MAX_BLOCKSIZE+1]; // General purpose buffer + overrun fetch
int           m_msc;
byte          m_msb[256];             // Command storage bytes

/*
 *  Data byte to BSRR register setting value and parity table
*/

// Parity bit generation
#define PTY(V)   (1^((V)^((V)>>1)^((V)>>2)^((V)>>3)^((V)>>4)^((V)>>5)^((V)>>6)^((V)>>7))&1)

// Data byte to BSRR register setting value conversion table
// BSRR[31:24] =  DB[7:0]
// BSRR[   16] =  PTY(DB)
// BSRR[15: 8] = ~DB[7:0]
// BSRR[    0] = ~PTY(DB)

// Set DBP, set REQ = inactive
//#define DBP(D)    ((((((uint32_t)(D)<<8)|PTY(D))*0x00010001)^0x0000ff01)|BITMASK(vREQ))
#define DBP(D)    ((((((uint32_t)(D)<<8)|PTY(D))*0x00010001)^0x0000ff01))
#define DBP8(D)   DBP(D),DBP(D+1),DBP(D+2),DBP(D+3),DBP(D+4),DBP(D+5),DBP(D+6),DBP(D+7)
#define DBP32(D)  DBP8(D),DBP8(D+8),DBP8(D+16),DBP8(D+24)

// BSRR register control value that simultaneously performs DB set, DP set
static const uint32_t db_bsrr[256]={
  DBP32(0x00),DBP32(0x20),DBP32(0x40),DBP32(0x60),
  DBP32(0x80),DBP32(0xA0),DBP32(0xC0),DBP32(0xE0)
};
// Parity bit acquisition
#define PARITY(DB) (db_bsrr[DB]&1)

// Macro cleaning
#undef DBP32
#undef DBP8
//#undef DBP
//#undef PTY

#if USE_DB2ID_TABLE
/* DB to SCSI-ID translation table */
static const byte db2scsiid[256]={
  0xff,
  0,
  1,1,
  2,2,2,2,
  3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
};
#endif

// Log File
#define LOG_FILENAME "LOG.txt"
FsFile LOG_FILE;

void onFalseInit(void);
void noSDCardFound(void);
void onBusReset(void);
void initFileLog(void);
void finalizeFileLog(void);

/*
 * IO read.
 */
inline byte readIO(void)
{
  // Port input data register
  uint32_t ret = GPIOB->regs->IDR;
  byte bret = (byte)((~ret)>>8);
#if READ_PARITY_CHECK
  if((db_bsrr[bret]^ret)&1)
    m_sts |= 0x01; // parity error
#endif

  return bret;
}

// Cleanup the lines read from the config file and format them correctly
void get_trimmed(char *instr, char *outstr, byte newlength)
{
  byte i;
  char *nl = strchr(instr,'\n');  // Find the LF
  if (nl != NULL) *nl = '\0';     // If LF found, truncate string there
  byte len = strlen(instr);       // Length of fixed input string
  for (i=0; i<(newlength-len); i++)  // Right pad any short strings
    strcat(instr," ");
  strncpy(outstr,instr,newlength);  // Return the correct length string
}

// If config file exists, read the first three lines and copy the contents.
void readSCSIDeviceConfig(HDDIMG *cimg, const char *image_name)
{
  char line[80] = {0};
  char vendor[LEN_VENDOR+1] = {0};
  char product[LEN_PRODUCT+1] = {0};
  char version[LEN_VERSION+1] = {0};
  char cfgname[MAX_FILE_PATH] = "#";

  strcat(cfgname,image_name);
  FsFile config_file = SD.open(cfgname, O_RDONLY);
  if (config_file.isOpen()) {
     config_file.fgets(line, sizeof(line));
     get_trimmed(line,vendor,LEN_VENDOR);
     config_file.fgets(line, sizeof(line));
     get_trimmed(line,product,LEN_PRODUCT);
     config_file.fgets(line, sizeof(line));
     get_trimmed(line,version,LEN_VERSION);
     config_file.close();
     LOG_FILE.print("Configuration for ");
  }
  else
    {
      strcpy(vendor,DEFAULT_VENDOR);
      strcpy(product,DEFAULT_PRODUCT);
      strcpy(version,DEFAULT_VERSION);
      LOG_FILE.print("Default configuration for ");
    }

  LOG_FILE.println(image_name);
  LOG_FILE.print("  SCSI VENDOR : "); LOG_FILE.println(vendor);
  LOG_FILE.print("  SCSI PRODUCT: "); LOG_FILE.println(product);
  LOG_FILE.print("  SCSI VERSION: "); LOG_FILE.println(version);
  LOG_FILE.print("  DEVICE TYPE : "); LOG_FILE.println(cimg->m_devtype);
  // Fill in the inquiry structure
  memset(&cimg->m_inquiry.raw, 0, sizeof(cimg->m_inquiry.raw));
  cimg->m_inquiry.device_type = cimg->m_devtype;
  if (cimg->m_devtype != SCSI_TYPE_HDD) cimg->m_inquiry.rmb = 0x80;  // Removable
  else cimg->m_inquiry.rmb = 0;         // Fixed
  cimg->m_inquiry.ansi_version = 1;
  cimg->m_inquiry.response_format = 1;
  cimg->m_inquiry.additional_length = 31;
  memcpy(&(cimg->m_inquiry.vendor), vendor, LEN_VENDOR);
  memcpy(&(cimg->m_inquiry.product), product, LEN_PRODUCT);
  memcpy(&(cimg->m_inquiry.revision), version, LEN_VERSION);
}

// read SD information and print to logfile
void readSDCardInfo()
{
  cid_t sd_cid;

  if(SD.card()->readCID(&sd_cid))
  {
    LOG_FILE.print("Sd MID:");
    LOG_FILE.print(sd_cid.mid, 16);
    LOG_FILE.print(" OID:");
    LOG_FILE.print(sd_cid.oid[0]);
    LOG_FILE.println(sd_cid.oid[1]);

    LOG_FILE.print("Sd Name:");
    LOG_FILE.print(sd_cid.pnm[0]);
    LOG_FILE.print(sd_cid.pnm[1]);
    LOG_FILE.print(sd_cid.pnm[2]);
    LOG_FILE.print(sd_cid.pnm[3]);
    LOG_FILE.println(sd_cid.pnm[4]);

    LOG_FILE.print("Sd Date:");
    LOG_FILE.print(sd_cid.mdt_month);
    LOG_FILE.print("/20"); // CID year is 2000 + high/low
    LOG_FILE.print(sd_cid.mdt_year_high);
    LOG_FILE.println(sd_cid.mdt_year_low);

    LOG_FILE.print("Sd Serial:");
    LOG_FILE.println(sd_cid.psn);
    LOG_FILE.sync();
  }
}

/*
 * Open HDD image file
 */

bool hddimageOpen(HDDIMG *h,const char *image_name,int id,int lun,int blocksize)
{
  h->m_fileSize = 0;
  h->m_blocksize = blocksize;
  h->m_file = SD.open(image_name, O_RDWR);
  if(h->m_file.isOpen())
  {
    h->m_fileSize = h->m_file.size();
    LOG_FILE.println("");
    LOG_FILE.print("Imagefile: ");
    LOG_FILE.print(image_name);
    if(h->m_fileSize>0)
    {
      // check blocksize dummy file
      LOG_FILE.print(" / ");
      LOG_FILE.print(h->m_fileSize);
      LOG_FILE.print("bytes / ");
      LOG_FILE.print(h->m_fileSize / 1024);
      LOG_FILE.print("KiB / ");
      LOG_FILE.print(h->m_fileSize / 1024 / 1024);
      LOG_FILE.println("MiB");
      switch(toupper(image_name[0])) {
        case 'H': h->m_devtype = SCSI_TYPE_HDD; break;
        case 'C': h->m_devtype = SCSI_TYPE_CDROM; break;
        default : h->m_devtype = SCSI_TYPE_HDD; break;
      }
      return true; // File opened
    }
    else
    {
      h->m_file.close();
      h->m_fileSize = h->m_blocksize = 0; // no file
      LOG_FILE.println("FileSizeError");
    }
  }
  return false;
}

/*
 * Initialization.
 *  Initialize the bus and set the PIN orientation
 */
void setup()
{
  // PA15 / PB3 / PB4 Cannot be used
  // JTAG Because it is used for debugging.
  disableDebugPorts();

  // Serial initialization
#if DEBUG > 0
  Serial.begin(9600);
  // If using a USB->TTL monitor instead of USB serial monitor - you can uncomment this.
  //while (!Serial);
#endif

  // PIN initialization
  gpio_mode(LED2, GPIO_OUTPUT_PP);
  gpio_mode(LED, GPIO_OUTPUT_OD);
  LED_OFF();

  //GPIO(SCSI BUS)Initialization
  //Port setting register (lower)
//  GPIOB->regs->CRL |= 0x000000008; // SET INPUT W/ PUPD on PAB-PB0
  //Port setting register (upper)
  //GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
//  GPIOB->regs->ODR = 0x0000FF00; // SET PULL-UPs on PB15-PB8
  // DB and DP are input modes
  SCSI_DB_INPUT()

  // Input port
  gpio_mode(ATN, GPIO_INPUT_PU);
  gpio_mode(BSY, GPIO_INPUT_PU);
  gpio_mode(ACK, GPIO_INPUT_PU);
  gpio_mode(RST, GPIO_INPUT_PU);
  gpio_mode(SEL, GPIO_INPUT_PU);
  // Output port
  gpio_mode(MSG, GPIO_OUTPUT_OD);
  gpio_mode(CD,  GPIO_OUTPUT_OD);
  gpio_mode(REQ, GPIO_OUTPUT_OD);
  gpio_mode(IO,  GPIO_OUTPUT_OD);
  // Turn off the output port
  SCSI_TARGET_INACTIVE()

  LED_ON();

  // clock = 36MHz , about 4Mbytes/sec
  if(!SD.begin(SD1_CONFIG)) {
#if DEBUG > 0
    Serial.println("SD initialization failed!");
#endif
    noSDCardFound();
  }
  initFileLog();
  readSDCardInfo();

  //Sector data overrun byte setting
  m_buf[MAX_BLOCKSIZE] = 0xff; // DB0 all off,DBP off
  //HD image file open
  scsi_id_mask = 0x00;

  // Iterate over the root path in the SD card looking for candidate image files.
  SdFile root;
  root.open("/");
  SdFile file;
  bool imageReady;
  int usedDefaultId = 0;
  while (1) {
    if (!file.openNext(&root, O_READ)) break;
    char name[MAX_FILE_PATH+1];
    if(!file.isDir()) {
      file.getName(name, MAX_FILE_PATH+1);
      file.close();
      String file_name = String(name);
      file_name.toLowerCase();
      if(file_name.startsWith("hd") || file_name.startsWith("cd")) {
        // Defaults for Hard Disks
        int id  = 1; // 0 and 3 are common in Macs for physical HD and CD, so avoid them.
        int lun = 0;
        int blk = 512;

        // Positionally read in and coerase the chars to integers.
        // We only require the minimum and read in the next if provided.
        int file_name_length = file_name.length();
        if(file_name_length > 2) { // HD[N]
          int tmp_id = name[HDIMG_ID_POS] - '0';

          if(tmp_id > -1 && tmp_id < 8) id = tmp_id; // If valid id, set it, else use default
          else usedDefaultId++;
          
        } else usedDefaultId++;

        if(file_name_length > 3) { // HD0[N]
          int tmp_lun = name[HDIMG_LUN_POS] - '0';

          if(tmp_lun > -1 && tmp_lun < 2) {
            lun = tmp_lun; // If valid id, set it, else use default
          }
        }
        int blk1, blk2, blk3, blk4 = 0;
        if(file_name_length > 8) { // HD00_[111]
          blk1 = name[HDIMG_BLK_POS] - '0';
          blk2 = name[HDIMG_BLK_POS+1] - '0';
          blk3 = name[HDIMG_BLK_POS+2] - '0';
          if(file_name_length > 9) // HD00_NNN[1]
            blk4 = name[HDIMG_BLK_POS+3] - '0';
        }
        if(blk1 == 2 && blk2 == 5 && blk3 == 6) {
          blk = 256;
        } else if(blk1 == 1 && blk2 == 0 && blk3 == 2 && blk4 == 4) {
          blk = 1024;
        } else if(blk1 == 2 && blk2 == 0 && blk3 == 4 && blk4 == 8) {
          blk  = 2048;
        }

        if(id < NUM_SCSIID && lun < NUM_SCSILUN) {
          HDDIMG *h = &img[id][lun];
          imageReady = hddimageOpen(h,name,id,lun,blk);

          if(imageReady) { // Marked as a responsive ID
            scsi_id_mask |= 1<<id;
            readSCSIDeviceConfig(h,name);
          }
        } else {
          LOG_FILE.println("");
          LOG_FILE.print("Bad LUN or SCSI id for image: ");
          LOG_FILE.println(name);
          LOG_FILE.sync();
        }
      } else {
          LOG_FILE.println("");
          LOG_FILE.print("Not an image: ");
          LOG_FILE.println(name);
          LOG_FILE.sync();
      }
    }
  }
  if(usedDefaultId > 1) {
    LOG_FILE.println("!! More than one image did not specify a SCSI ID. Last file will be used at ID 1. !!");
    LOG_FILE.sync();
  }
  root.close();

  // Error if there are 0 image files
  if(scsi_id_mask==0) {
    LOG_FILE.println("ERROR: No valid images found!");
    onFalseInit();
  }

  finalizeFileLog();
  LED_OFF();
  //Occurs when the RST pin state changes from HIGH to LOW
  attachInterrupt(PIN_MAP[RST].gpio_bit, onBusReset, FALLING);
}

/*
 * Setup initialization logfile
 */
void initFileLog() {
  LOG_FILE = SD.open(LOG_FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
  LOG_FILE.println("BlueSCSI <-> SD - https://github.com/erichelgeson/BlueSCSI");
  LOG_FILE.print("VERSION: ");
  LOG_FILE.println(VERSION);
  LOG_FILE.print("DEBUG:");
  LOG_FILE.print(DEBUG);
  LOG_FILE.print(" SDFAT_FILE_TYPE:");
  LOG_FILE.println(SDFAT_FILE_TYPE);
  LOG_FILE.print("SdFat version: ");
  LOG_FILE.println(SD_FAT_VERSION_STR);
  LOG_FILE.print("SdFat Max FileName Length: ");
  LOG_FILE.println(MAX_FILE_PATH);
  LOG_FILE.println("Initialized SD Card - lets go!");
  LOG_FILE.sync();
}

/*
 * Finalize initialization logfile
 */
void finalizeFileLog() {
  // View support drive map
  LOG_FILE.print("ID");
  for(int lun=0;lun<NUM_SCSILUN;lun++)
  {
    LOG_FILE.print(":LUN");
    LOG_FILE.print(lun);
  }
  LOG_FILE.println(":");
  //
  for(int id=0;id<NUM_SCSIID;id++)
  {
    LOG_FILE.print(" ");
    LOG_FILE.print(id);
    for(int lun=0;lun<NUM_SCSILUN;lun++)
    {
      HDDIMG *h = &img[id][lun];
      if( (lun<NUM_SCSILUN) && (h->m_file))
      {
        LOG_FILE.print((h->m_blocksize<1000) ? ": " : ":");
        LOG_FILE.print(h->m_blocksize);
      }
      else      
        LOG_FILE.print(":----");
    }
    LOG_FILE.println(":");
  }
  LOG_FILE.println("Finished initialization of SCSI Devices - Entering main loop.");
  LOG_FILE.sync();
  LOG_FILE.close();
}

/*
 * Initialization failed, blink 3x fast
 */
void onFalseInit(void)
{
  LOG_FILE.sync();
  while(true) {
    for(int i = 0; i < 3; i++) {
      LED_ON();
      delay(250);
      LED_OFF();
      delay(250);
    }
    delay(3000);
  }
}

/*
 * No SC Card found, blink 5x fast
 */
void noSDCardFound(void)
{
  while(true) {
    for(int i = 0; i < 5; i++) {
      LED_ON();
      delay(250);
      LED_OFF();
      delay(250);
    }
    delay(3000);
  }
}

/*
 * Bus reset interrupt.
 */
void onBusReset(void)
{
  if(isHigh(gpio_read(RST))) {
    SCSI_RESET_HOLD();
    if(isHigh(gpio_read(RST))) {
      // BUS FREE is done in the main process
      SCSI_DB_INPUT()
      LOGN("BusReset!");
      m_isBusReset = true;
    }
  }
}

/*
 * Read by handshake.
 */
inline byte readHandshake(void)
{
  SCSI_OUT(vREQ,active)
  //SCSI_DB_INPUT()
  while( ! SCSI_IN(vACK)) { if(m_isBusReset) return 0; }
  byte r = readIO();
  SCSI_OUT(vREQ,inactive)
  while( SCSI_IN(vACK)) { if(m_isBusReset) return 0; }
  return r;  
}

/*
 * Write with a handshake.
 */
inline void writeHandshake(byte d)
{
  SCSI_DB_OUTPUT()
  GPIOB->regs->BSRR = db_bsrr[d];
  SCSI_DESKEW(); SCSI_CABLE_SKEW();
  while( SCSI_IN(vACK)) { if(m_isBusReset) return; }
  SCSI_OUT(vREQ,active)
  while(!m_isBusReset && !SCSI_IN(vACK));
  SCSI_OUT(vREQ, inactive);
  SCSI_DB_INPUT()
}

/*
 * Data in phase.
 *  Send len bytes of data array p.
 */
void writeDataPhase(int len, const byte* p)
{
  LOGN("DATAIN PHASE");
  SCSI_PHASE_DATA_IN();
  for (int i = 0; i < len; i++) {
    if(m_isBusReset) {
      return;
    }
    writeHandshake(p[i]);
  }
}

/* 
 * Data in phase.
 *  Send len block while reading from SD card.
 */
void writeDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAIN PHASE(SD)");
  const uint32_t BUFBLOCKS = MAX_BLOCKSIZE / m_img->m_blocksize;  // Number of blocks buffer can hold
  register const uint32_t *bsrr_tbl = db_bsrr;  // Table to convert to BSRR
  m_img->m_file.seek(adds * m_img->m_blocksize);

  SCSI_PHASE_DATA_IN();
  SCSI_DB_OUTPUT();

  while (len > 0) {
    uint32_t btt = (len < BUFBLOCKS) ? len : BUFBLOCKS;  // Blocks to transfer

    m_img->m_file.read(m_buf, m_img->m_blocksize*btt);

    // Dump the block out to a file for analysis
    //FsFile dump_file = SD.open(String(millis())+"_"+String(adds)+"_"+String(len)+".DMP", O_WRONLY | O_CREAT);
    //dump_file.write(m_buf,m_img->m_blocksize*btt);
    //dump_file.close();

    register byte *srcptr= m_buf;         // Source buffer
    register byte *endptr= srcptr + m_img->m_blocksize*btt; // End pointer

    #define DATA_TRANSFER() \
      GPIOB->regs->BSRR = bsrr_tbl[*srcptr++]; \
      while(SCSI_IN(vACK)) { if(m_isBusReset) return; } \
      SCSI_OUT(vREQ,active) \
      while(!m_isBusReset && !SCSI_IN(vACK)); \
      SCSI_OUT(vREQ, inactive);

    do
    {
      // 16 bytes per loop
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();
      DATA_TRANSFER();

    } while(srcptr < endptr);
      len -= btt;
  }
  while(SCSI_IN(vACK)) { if(m_isBusReset) return; }
  SCSI_DB_INPUT();
}

/* 
 * Data in phase.
 *  Send len bytes while reading from SD card.
 */
void writelongDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAIN PHASE(SD)");
  register const uint32_t *bsrr_tbl = db_bsrr;  // Table to convert to BSRR
  m_img->m_file.seek(adds * m_img->m_blocksize);

  SCSI_PHASE_DATA_IN();
  SCSI_DB_OUTPUT();

  while (len > 0) {
    uint32_t btt = (len < MAX_BLOCKSIZE) ? len : MAX_BLOCKSIZE;  // Bytes to transfer

    m_img->m_file.read(m_buf, btt);

    register byte *srcptr= m_buf;         // Source buffer
    register byte *endptr= srcptr + btt;  // End pointer

    do
    {
      GPIOB->regs->BSRR = bsrr_tbl[*srcptr++]; \
      while(SCSI_IN(vACK)) { if(m_isBusReset) return; } \
      SCSI_OUT(vREQ,active) \
      while(!m_isBusReset && !SCSI_IN(vACK)); \
      SCSI_OUT(vREQ, inactive);
    } while(srcptr < endptr);
      len -= btt;
  }
  while(SCSI_IN(vACK)) { if(m_isBusReset) return; }
  SCSI_DB_INPUT();
}

/*
 * Data out phase.
 *  len block read
 */
void readDataPhase(int len, byte* p)
{
  LOGN("DATAOUT PHASE");
  SCSI_PHASE_DATA_OUT();
  for(uint32_t i = 0; i < len; i++)
    p[i] = readHandshake();
}

/*
 * Data out phase.
 *  Write to SD card while reading len block.
 */
void readDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAOUT PHASE(SD)");
  uint32_t pos = adds * m_img->m_blocksize;
  m_img->m_file.seek(pos);
  SCSI_PHASE_DATA_OUT();

  for(uint32_t i = 0; i < len; i++) {
  register byte *dstptr= m_buf;
	register byte *endptr= m_buf + m_img->m_blocksize;

    for(dstptr=m_buf;dstptr<endptr;dstptr+=8) {
      dstptr[0] = readHandshake();
      dstptr[1] = readHandshake();
      dstptr[2] = readHandshake();
      dstptr[3] = readHandshake();
      dstptr[4] = readHandshake();
      dstptr[5] = readHandshake();
      dstptr[6] = readHandshake();
      dstptr[7] = readHandshake();
      if(m_isBusReset) {
        return;
      }
    }
    m_img->m_file.write(m_buf, m_img->m_blocksize);
  }
  m_img->m_file.flush();
}

/*
 * Data out phase.
 *  Write to SD card while reading len bytes.
 */
void readlongDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAOUT PHASE(SD)");
  uint32_t pos = adds * m_img->m_blocksize;
  uint32_t idx;

  m_img->m_file.seek(pos);
  SCSI_PHASE_DATA_OUT();

  while (len > 0) {
    uint32_t btt = (len < MAX_BLOCKSIZE) ? len : MAX_BLOCKSIZE;  // Bytes to transfer

    for(idx=0;idx<btt;idx++) {
      m_buf[idx] = readHandshake();
      if(m_isBusReset) {
        return;
      }
    m_img->m_file.write(m_buf, btt);
    }
    len -= btt;
  }
  m_img->m_file.flush();
}

/*
 * INQUIRY command processing.
 */
byte onInquiryCommand(byte len)
{
  writeDataPhase(len < 36 ? len : 36, m_img->m_inquiry.raw);
  return 0x00;
}

/*
 * REQUEST SENSE command processing.
 */
void onRequestSenseCommand(byte len)
{
  byte buf[18] = {
    0x70,   //CheckCondition
    0,      //Segment number
    m_senseKey,   //Sense key
    0, 0, 0, 0,  //information
    10,   //Additional data length
    0, 0, 0, 0, // command specific information bytes
    (byte)(m_addition_sense >> 8),
    (byte)m_addition_sense,
    0, 0, 0, 0,
  };
  m_senseKey = 0;
  m_addition_sense = 0;
  writeDataPhase(len < 18 ? len : 18, buf);  
}

/*
 * READ CAPACITY command processing.
 */
byte onReadCapacityCommand(byte pmi)
{
  if(!m_img) return 0x02; // Image file absent
  
  uint32_t bl = m_img->m_blocksize;
  uint32_t bc = m_img->m_fileSize / bl - 1; // Points to last LBA
  uint8_t buf[8] = {
    bc >> 24, bc >> 16, bc >> 8, bc,
    bl >> 24, bl >> 16, bl >> 8, bl    
  };
  writeDataPhase(8, buf);
  return 0x00;
}

/*
 * READ6 / 10 Command processing.
 */
byte onReadCommand(uint32_t adds, uint32_t len)
{
  LOGN("-R");
  LOGHEXN(adds);
  LOGHEXN(len);

  if(!m_img) return 0x02; // Image file absent
  
  LED_ON();
  writeDataPhaseSD(adds, len);
  LED_OFF();
  return 0x00; //sts
}

/*
 * WRITE6 / 10 Command processing.
 */
byte onWriteCommand(uint32_t adds, uint32_t len)
{
  LOGN("-W");
  LOGHEXN(adds);
  LOGHEXN(len);
  
  if(!m_img) return 0x02; // Image file absent
  
  LED_ON();
  readDataPhaseSD(adds, len);
  LED_OFF();
  return 0; //sts
}

/*
 * READLONG10 Command processing.
 */
byte onReadLongCommand(uint32_t adds, uint32_t len)
{
  LOGN("-RL");
  LOGHEXN(adds);
  LOGHEXN(len);

  if(!m_img) return 0x02; // Image file absent
  if (len == 0) return 0x00;  // Zero length is not an error
  
  LED_ON();
  writelongDataPhaseSD(adds, len);
  LED_OFF();
  return 0x00; //sts
}

/*
 * WRITELONG10 Command processing.
 */
byte onWriteLongCommand(uint32_t adds, uint32_t len)
{
  LOGN("-WL");
  LOGHEXN(adds);
  LOGHEXN(len);
  
  if(!m_img) return 0x02; // Image file absent
  if (len == 0) return 0x00;  // Zero length is not an error

  LED_ON();
  readlongDataPhaseSD(adds, len);
  LED_OFF();
  return 0; //sts
}

/*
 * MODE SENSE command processing.
 */
byte onModeSenseCommand(byte scsi_cmd, byte dbd, int cmd2, uint32_t len)
{
  if(!m_img) return 0x02; // No image file

  uint32_t bl =  m_img->m_blocksize;
  uint32_t bc = m_img->m_fileSize / bl;
  
  memset(m_buf, 0, sizeof(m_buf));
  int page_code = cmd2 & 0x3F;
  int a = (scsi_cmd == 0x5A) ? 8 : 4;
  
  if(dbd == 0) {
    byte c[8] = {
      0,//Density code
      bc >> 16, bc >> 8, bc,
      0, //Reserve
      bl >> 16, bl >> 8, bl    
    };
    memcpy(&m_buf[a], c, 8);
    a += 8;
  }
  
  switch(page_code) {
      case 0x3F:
      case 0x01: // Read/Write Error Recovery
        m_buf[a + 0] = 0x01;
        m_buf[a + 1] = 0x0A;
        a += 0x0C;
        if(page_code != 0x3F) break;

      case 0x02: // Disconnect-Reconnect page
        m_buf[a + 0] = 0x02;
        m_buf[a + 1] = 0x0A;
        a += 0x0C;
        if(page_code != 0x3f) break;

      case 0x03:  //Drive parameters
        m_buf[a + 0] = 0x03; //Page code
        m_buf[a + 1] = 0x16; // Page length
        m_buf[a + 11] = 0x3F;//Number of sectors / track
        m_buf[a + 12] = (byte)(m_img->m_blocksize >> 8);
        m_buf[a + 13] = (byte)m_img->m_blocksize;
        m_buf[a + 15] = 0x1; // Interleave
        a += 0x18;
        if(page_code != 0x3F) break;
        
      /*
	        cylinder = LBA / (heads_per_cylinder * sectors_per_track)
	        temp = LBA % (heads_per_cylinder * sectors_per_track)
	        head = temp / sectors_per_track
	        sector = temp % sectors_per_track + 1
      */
      case 0x04:  //Drive parameters
      {
        unsigned cylinders = bc / (16 * 63);
        m_buf[a + 0] = 0x04; //Page code
        m_buf[a + 1] = 0x16; // Page length
        m_buf[a + 2] = (byte)(cylinders >> 16); // Cylinders
        m_buf[a + 3] = (byte)(cylinders >> 8);
        m_buf[a + 4] = (byte)cylinders;
        m_buf[a + 5] = 16;   //Number of heads
        a += 0x18;
        if(page_code != 0x3F) break;
      }
      break; // Don't want 0x3F falling through to error condition

  default:
    m_senseKey = 5; // Illegal request
    m_addition_sense = 0x2400; // Invalid field in CDB
    return 0x02;
    break;
  }
  if(scsi_cmd == 0x5A) // MODE SENSE 10
  {
    m_buf[1] = a - 2;
    m_buf[7] = 0x08;
  }
  else
  {
    m_buf[0] = a - 1;
    m_buf[3] = 0x08;
  }
  writeDataPhase(len < a ? len : a, m_buf);
  return 0x00;
}

/*
 * MsgIn2.
 */
void MsgIn2(int msg)
{
  LOGN("MsgIn2");
  SCSI_PHASE_MSG_IN();
  writeHandshake(msg);
}

/*
 * MsgOut2.
 */
void MsgOut2()
{
  LOG("MsgOut2-");
  SCSI_PHASE_MSG_OUT();
  m_msb[m_msc] = readHandshake();
  LOGHEXN(m_msb[m_msc]);
  m_msc++;
  m_msc %= 256;
}

/*
 * Main loop.
 */
void loop() 
{
  //int msg = 0;
  m_msg = 0;

  // Wait until RST = H, BSY = H, SEL = L
  do {} while( SCSI_IN(vBSY) || !SCSI_IN(vSEL) || SCSI_IN(vRST));

  // BSY+ SEL-
  // If the ID to respond is not driven, wait for the next
  //byte db = readIO();
  //byte scsiid = db & scsi_id_mask;
  byte scsiid = readIO() & scsi_id_mask;
  if((scsiid) == 0) {
    SCSI_DISCONNECTION_DELAY();
    return;
  }
  LOGN("Selection");
  m_isBusReset = false;
  // Set BSY to-when selected
  SCSI_BSY_ACTIVE();     // Turn only BSY output ON, ACTIVE

  // Ask for a TARGET-ID to respond
#if USE_DB2ID_TABLE
  m_id = db2scsiid[scsiid];
  //if(m_id==0xff) return;
#else
  for(m_id=7;m_id>=0;m_id--)
    if(scsiid & (1<<m_id)) break;
  //if(m_id<0) return;
#endif

  // Wait until SEL becomes inactive
  while(isHigh(gpio_read(SEL)) && isLow(gpio_read(BSY))) {
    if(m_isBusReset) {
      goto BusFree;
    }
  }
  SCSI_TARGET_ACTIVE()  // (BSY), REQ, MSG, CD, IO output turned on
  //  
  if(isHigh(gpio_read(ATN))) {
    bool syncenable = false;
    int syncperiod = 50;
    int syncoffset = 0;
    int loopWait = 0;
    m_msc = 0;
    memset(m_msb, 0x00, sizeof(m_msb));
    while(isHigh(gpio_read(ATN)) && loopWait < 255) {
      MsgOut2();
      loopWait++;
    }
    for(int i = 0; i < m_msc; i++) {
      // ABORT
      if (m_msb[i] == 0x06) {
        goto BusFree;
      }
      // BUS DEVICE RESET
      if (m_msb[i] == 0x0C) {
        syncoffset = 0;
        goto BusFree;
      }
      // IDENTIFY
      if (m_msb[i] >= 0x80) {
      }
      // Extended message
      if (m_msb[i] == 0x01) {
        // Check only when synchronous transfer is possible
        if (!syncenable || m_msb[i + 2] != 0x01) {
          MsgIn2(0x07);
          break;
        }
        // Transfer period factor(50 x 4 = Limited to 200ns)
        syncperiod = m_msb[i + 3];
        if (syncperiod > 50) {
          syncperiod = 50;
        }
        // REQ/ACK offset(Limited to 16)
        syncoffset = m_msb[i + 4];
        if (syncoffset > 16) {
          syncoffset = 16;
        }
        // STDR response message generation
        MsgIn2(0x01);
        MsgIn2(0x03);
        MsgIn2(0x01);
        MsgIn2(syncperiod);
        MsgIn2(syncoffset);
        break;
      }
    }
  }
  LOG("("); LOG(millis()); LOG(")");
  LOG("Command:");
  SCSI_PHASE_COMMAND();
  
  int len;
  byte cmd[12];
  cmd[0] = readHandshake(); if(m_isBusReset) goto BusFree;
  LOGHEX(cmd[0]);
  // Command length selection, reception
  static const int cmd_class_len[8]={6,10,10,6,6,12,6,6};
  len = cmd_class_len[cmd[0] >> 5];
  cmd[1] = readHandshake(); LOG(":");LOGHEX(cmd[1]); if(m_isBusReset) goto BusFree;
  cmd[2] = readHandshake(); LOG(":");LOGHEX(cmd[2]); if(m_isBusReset) goto BusFree;
  cmd[3] = readHandshake(); LOG(":");LOGHEX(cmd[3]); if(m_isBusReset) goto BusFree;
  cmd[4] = readHandshake(); LOG(":");LOGHEX(cmd[4]); if(m_isBusReset) goto BusFree;
  cmd[5] = readHandshake(); LOG(":");LOGHEX(cmd[5]); if(m_isBusReset) goto BusFree;
  // Receive the remaining commands
  for(int i = 6; i < len; i++ ) {
    cmd[i] = readHandshake();
    LOG(":");
    LOGHEX(cmd[i]);
    if(m_isBusReset) goto BusFree;
  }
  // LUN confirmation
  m_sts = cmd[1]&0xe0;      // Preset LUN in status byte
  m_lun = m_sts>>5;
  // HDD Image selection
  m_img = (HDDIMG *)0; // None
  if( (m_lun <= NUM_SCSILUN) )
  {
    m_img = &(img[m_id][m_lun]); // There is an image
    if(!(m_img->m_file.isOpen()))
    {
      m_img = (HDDIMG *)0;       // Image absent
    }
  }
  
  LOG(":ID ");
  LOG(m_id);
  LOG(":LUN ");
  LOG(m_lun);

  LOGN("");
  switch(cmd[0]) {
  case 0x00:
    LOGN("[Test Unit]");
    break;
  case 0x01:
    LOGN("[Rezero Unit]");
    break;
  case 0x03:
    LOGN("[RequestSense]");
    onRequestSenseCommand(cmd[4]);
    break;
  case 0x04:
    LOGN("[FormatUnit]");
    break;
  case 0x06:
    LOGN("[FormatUnit]");
    break;
  case 0x07:
    LOGN("[ReassignBlocks]");
    break;
  case 0x08:
    LOGN("[Read6]");
    m_sts |= onReadCommand((((uint32_t)cmd[1] & 0x1F) << 16) | ((uint32_t)cmd[2] << 8) | cmd[3], (cmd[4] == 0) ? 0x100 : cmd[4]);
    break;
  case 0x0A:
    LOGN("[Write6]");
    m_sts |= onWriteCommand((((uint32_t)cmd[1] & 0x1F) << 16) | ((uint32_t)cmd[2] << 8) | cmd[3], (cmd[4] == 0) ? 0x100 : cmd[4]);
    break;
  case 0x0B:
    LOGN("[Seek6]");
    break;
  case 0x12:
    LOGN("[Inquiry]");
    m_sts |= onInquiryCommand(cmd[4]);
    break;
  case 0x1A:
    LOGN("[ModeSense6]");
    m_sts |= onModeSenseCommand(cmd[0], cmd[1]&0x80, cmd[2], cmd[4]);
    break;
  case 0x1B:
    LOGN("[StartStopUnit]");
    break;
  case 0x1E:
    LOGN("[PreAllowMed.Removal]");
    break;
  case 0x25:
    LOGN("[ReadCapacity]");
    m_sts |= onReadCapacityCommand(cmd[8]);
    break;
  case 0x28:
    LOGN("[Read10]");
    m_sts |= onReadCommand(((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case 0x2A:
    LOGN("[Write10]");
    m_sts |= onWriteCommand(((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case 0x2B:
    LOGN("[Seek10]");
    break;
  case 0x3E:
    LOGN("[ReadLong10]");
    m_sts |= onReadLongCommand(((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case 0x3F:
    LOGN("[WriteLong10]");
    m_sts |= onWriteLongCommand(((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case 0x5A:
    LOGN("[ModeSense10]");
    m_sts |= onModeSenseCommand(cmd[0], cmd[1] & 0x80, cmd[2], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  default:
    LOGN("[*Unknown]");
    m_sts |= 0x02;
    m_senseKey = 5;  // Illegal request
    m_addition_sense = 0x2000; // Invalid Command Operation Code
    break;
  }
  if(m_isBusReset) {
     goto BusFree;
  }

  LOGN("Sts");
  SCSI_PHASE_STATUS();
  writeHandshake(m_sts);
  if(m_isBusReset) {
     goto BusFree;
  }

  LOGN("MsgIn");
  SCSI_PHASE_MSG_IN();
  writeHandshake(m_msg);

BusFree:
  LOGN("BusFree");
  LED_OFF();
  m_isBusReset = false;
  SCSI_TARGET_INACTIVE() // Turn off BSY, REQ, MSG, CD, IO output
}
