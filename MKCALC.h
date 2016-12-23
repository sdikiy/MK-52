/***************************************************
 * MKCALC.h   - Core of MKCalk library for linking *
 *              MK-52/61 calculators and Arduino.  *
 *              Created by _______________________ *
 *              2016-_____________________________ *
 ***************************************************/

#ifndef MKCALC_H
#define MKCALC_H

#include "Arduino.h"
#include "HardwareSerial.h"

#define NOP5 __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t")

typedef enum {
  WAIT_A_START, //Quick search mark
  WAIT_A_MARK,  //Precise positioning of mark
  READ_BITS     //InSync
} readStateType;

typedef enum {
  WAIT_WRITE_COMMAND,
  INIT_WRITE,
  WRITE_IN_PROGRES
} write2mkStateType;

struct commandQueueStruct {
  uint8_t in;
  uint8_t out;
  uint8_t command[16];
};

class MKCALC {
  public:
    MKCALC();
    void setSerial(HardwareSerial* serial);
    void MemoryPagesPrint(volatile uint8_t* memDump);
    void WriteRamToRom(uint8_t* memDump);
    void WriteRamToMK(uint8_t* memDump);

    //static volatile byte cmd_state;
    static volatile readStateType readState;
    static volatile uint16_t posRAM;
    static volatile uint8_t skipClockCycle;
    static volatile uint8_t tempOut;
    static volatile write2mkStateType dumpWriteToMkStatus;
    static volatile uint8_t* dumpToMk;
    static volatile uint16_t byteNumToMk;
    static void interruptSPI();
    static volatile uint8_t RAMdata[315];
    static volatile commandQueueStruct commandQ;

  protected:
    HardwareSerial* serial_;

  private:
    static byte readTetrad(uint16_t ind, volatile uint8_t* memDump);
    static volatile uint8_t fixCounter;
    volatile byte Timsk0_temp;
};

#endif  // MKCALC_H