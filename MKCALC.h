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

#define WAIT_A_START  0
#define WAIT_A_MARK   1
#define READ_BITS     2

#define NOP5 __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t")

class MKCALC {
  public:
    MKCALC();
    void setSerial(HardwareSerial* serial);
    void MemoryPagesPrint();
    void WriteRamToRom();
    void WriteRamToMK();

    static volatile byte cmd_state;
    static volatile uint16_t posRAM;
    static volatile uint8_t skipClockCycle;
    static volatile uint8_t tempOut;
    static volatile uint8_t byteWriteToMkStatus;
    static volatile uint8_t byteToMk;
    static volatile uint16_t byteNumToMk;
    static void interruptSPI();

  protected:
    HardwareSerial* serial_;

  private:
    static byte readTetrad(uint16_t ind, volatile uint8_t* memDump);
    static volatile uint8_t RAMdata[315];
    static volatile uint8_t fixCounter;
    volatile byte Timsk0_temp;
};

#endif  // MKCALC_H