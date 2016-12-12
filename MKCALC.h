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

//входные сигналы от калькулятора
#define SYNC 2          //вход синхроимпулься (СИ)                  //вход D2, пин PD2
#define F4 3            //вход тактовых импульсов генератора Ф4     //вход D3,
#define REG_inp 11      //вход системной магистрали (Рг вх)         //вход D11, пин PB3
#define PD2_F4    2
#define PD2_F4_MASKA    0x04
#define PD3_SY    3
#define PD3_SY_MASKA    0x08
#define PD4_PrIN  4
#define PD4_PrIN_MASKA  0x10
#define PD5_PrOU  5

//определяем состояния программы
#define WAIT_A_START  0  //ожидание синхроимпульса для старта
#define WAIT_A_MARK  1   //ожидание сигнала "метка"
#define READ_BITS 2      //чтение битов из ОЗУ калькулятора

#define NOP5 __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t")

class MKCALC {
	public:
		MKCALC();
    void setSerial(HardwareSerial* serial);
    void setInterrupt();
    void clrInterrupt();
    byte getState();
    void setState(byte newState);
    void ringToMem();
    void ringToMemNoInterrupt();
    void MemoryPagesPrint();  //вывод всех страниц памяти в виде тетрад в HEX виде

    static volatile uint16_t posRAM;
    static volatile uint8_t commandT;
    static volatile uint8_t tempOut;
    static volatile byte cmd_state;  //начальное состояние программы
    static void F4_int();
    static void interruptSPI();

	protected:
		HardwareSerial* serial_;

	private:
    static void F4_interrupt();             //подпрограмма обработки прерываний

    static void writeBit(int index, bool value);  //запись бита в массив RAMdata
    static bool readBit(int index);         //чтение бита из массива RAMdata
    static byte readTetrad(int index);    //функция для чтения тетрады из смассива ОП   //index - номер терады

    static volatile uint8_t RAMdata[315]; // 2520/8 = 315 массив для хранения всей ОП 
    static uint16_t BitCounter;               //счетчик битов в кольцевой ОП
    static byte port_B;
    static byte port_D;
    volatile byte Timsk0_temp;

    static long startTimeCount;
    static long waitTimeCount;
    static long readTimeCount;
    //static byte M1_tetrads[210];       //блок памяти М1
    //static byte M2_tetrads[210];       //блок памяти М2
    //static byte M3_tetrads[210];       //блок памяти М3
};

#endif  // MKCALC_H