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

#define WAIT_A_START  0  //ожидание синхроимпульса для старта
#define WAIT_A_MARK   1  //ожидание сигнала "метка"
#define READ_BITS     2  //чтение битов из ОЗУ калькулятора

#define NOP5 __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t")

class MKCALC {
  public:
    MKCALC();
    void setSerial(HardwareSerial* serial);
    void MemoryPagesPrint();  //вывод всех страниц памяти в виде тетрад в HEX виде

    static volatile byte cmd_state;  //начальное состояние программы
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
    static byte readTetrad(int index);    //функция для чтения тетрады из смассива ОП   //index - номер терады
    static volatile uint8_t RAMdata[315]; // 2520/8 = 315 массив для хранения всей ОП 
    volatile byte Timsk0_temp;
};

#endif  // MKCALC_H