/***************************************************
 * MKCALC.cpp - Core of MKCalk library for linking *
 *              MK-52/61 calculators and Arduino.  *
 *              Created by _______________________ *
 *              2016-_____________________________ *
 ***************************************************/

#include "Arduino.h"
#include "MKCALC.h"

volatile readStateType MKCALC::readState = WAIT_A_START;
volatile uint8_t MKCALC::RAMdata[315];
volatile commandQueueStruct MKCALC::commandQ;
volatile uint16_t MKCALC::posRAM = 0;
volatile uint8_t MKCALC::skipClockCycle = 0;
volatile uint8_t MKCALC::tempOut = 0;
volatile write2mkStateType MKCALC::dumpWriteToMkStatus = WAIT_WRITE_COMMAND;
volatile uint8_t* MKCALC::dumpToMk = NULL;
volatile uint16_t MKCALC::byteNumToMk = 13;
volatile uint8_t MKCALC::fixCounter = 0x00;

MKCALC::MKCALC() {
  //serial_ = NULL;
  readState = WAIT_A_START;
  commandQ.in = 0;
  commandQ.out = 0;
}

void MKCALC::setSerial(HardwareSerial* serial) {
  serial_ = serial;
}

byte MKCALC::readTetrad(uint16_t ind, volatile uint8_t* memDump) {
  //See: Arduino/hardware/tools/avr/avr/include/avr/builtins.h
  return 0x0F & (((ind & 0x01) == 1) ? (memDump[ind >> 1]) : (__builtin_avr_swap(memDump[ind >> 1])));
}

//TODO setTetrad(0x0F, 24 * 3 + 1, MyTestMemDump)
void MKCALC::setTetrad(uint8_t val, uint16_t ind, volatile uint8_t* memDump) {
  if ((ind & 0x01) == 1) {
    //0x0F & (memDump[ind >> 1]) - from readTetrad
    memDump[ind >> 1] = ((0xF0 & (memDump[ind >> 1])) | (0x0F & val));
  } else {
    memDump[ind >> 1] = ((0x0F & (memDump[ind >> 1])) | (0xF0 & (val << 4)));
  }
}

#define TOREALIND(indRAM) ((indRAM >= 315) ? (indRAM - 315) : (indRAM))
void MKCALC::interruptSPI() {
  PORTD |= (1 << 6);
  //Set data to send by SPI
  if (readState == READ_BITS) {
    if (dumpWriteToMkStatus == WRITE_IN_PROGRES) {
      SPDR = __builtin_avr_swap(dumpToMk[TOREALIND(posRAM + 1)]);
    } else {
      SPDR = __builtin_avr_swap(RAMdata[TOREALIND(posRAM + 1)]);
    }
  }
  //Read data from SPI Data Register (double buffered)
  byte c = __builtin_avr_swap(~SPDR);

  if (posRAM == 0) PORTB |= (1 << 0); //oscilloscope trigger start
  fixCounter++;

  RAMdata[posRAM] = c;
  posRAM = posRAM + 1;

  //Quick search mark
  if ((readState == WAIT_A_START) && (posRAM > 15)) {
    uint16_t t = posRAM - 14;
    if ( ((RAMdata[t+0] & RAMdata[t+3] & RAMdata[t+6] & RAMdata[t+9] & RAMdata[t+12] & 0x0F )
        | (RAMdata[t+2] & RAMdata[t+5] & RAMdata[t+8] & RAMdata[t+11]              & 0xF0 )) == 0xFF ) {
      posRAM = 15;
      readState = WAIT_A_MARK;
    } else if (fixCounter == 0) {
      skipClockCycle = 1;
    }
  }

  //Precise positioning of mark
  if (posRAM == 315) {
    posRAM = 0;
    if ( ((RAMdata[0] & RAMdata[3] & RAMdata[6] & RAMdata[9] & RAMdata[12] & 0x0F )
        | (RAMdata[2] & RAMdata[5] & RAMdata[8] & RAMdata[11]              & 0xF0 )) == 0xFF ) {
      readState = READ_BITS;
      uint8_t curentCommand = (readTetrad(25 * 3 + 1, RAMdata) << 4) | readTetrad(24 * 3 + 1, RAMdata);
      if (!(commandQ.command[commandQ.out] == curentCommand)) {
        commandQ.out = (commandQ.out + 1) & 0x0F;
        commandQ.command[commandQ.out] = curentCommand;
      }
      if (dumpWriteToMkStatus == WRITE_IN_PROGRES) {
        dumpWriteToMkStatus = WAIT_WRITE_COMMAND;
        PORTD |= (1 << 7); //write to MK OFF
      } else
      if (dumpWriteToMkStatus == INIT_WRITE) {
        dumpWriteToMkStatus = WRITE_IN_PROGRES;
        PORTD &= ~(1 << 7); //write to MK ON
      } else
      if ((2 == readTetrad(25 * 3 + 1, RAMdata)) && (7 == readTetrad(24 * 3 + 1, RAMdata))) {
        //start write MK memory to EEPROM
      } else
      if ((2 == readTetrad(25 * 3 + 1, RAMdata)) && (9 == readTetrad(24 * 3 + 1, RAMdata))) {
        //start write EEPROM to MK memory
      }
    } else {
      if (readState == READ_BITS) readState = WAIT_A_START;
      skipClockCycle = 1;
    }
  }

  //skip one SPI clock cycle
  if (skipClockCycle == 1) {
    PORTB |= (1 << 1); 
    skipClockCycle = 0;
    NOP5; NOP5; NOP5; NOP5;
    PORTB &= ~(1 << 1);
  }

  PORTD &= ~(1 << 6);
  PORTB &= ~(1 << 0); //oscilloscope trigger end
}

void MKCALC::WriteRamToRom(uint8_t* memDump) {
  NOP5;
}

void MKCALC::WriteRamToMK(uint8_t* memDump) {
  NOP5; NOP5;
}

void MKCALC::MemoryPagesPrint(volatile uint8_t* memDump) {
  serial_->print("readState in MemoryPagesPrint - ");
  serial_->println(readState);
  serial_->println("+----+--------------- + -------------- + -------------- + -------------- + -------------- +");
  serial_->println("|    | 0123456789ABCD | 0123456789ABCD | 0123456789ABCD | 0123456789ABCD | 0123456789ABCD |");
  serial_->println("+----+--------------- + -------------- + -------------- + -------------- + -------------- +");

  for (uint8_t j = 0; j < 3; j++) {
    serial_->print("  M");
    serial_->print(j + 1);
    serial_->print(" | ");
    for (uint16_t i = 0; i < 210; i++) {
      serial_->print(readTetrad(i * 3 + j, memDump), HEX);
      if (i > 0) {
        if (0 == (i + 1) % 14) serial_->print(" | ");
        if (0 == ((i + 1) % (14 * 5))) serial_->print("\r\n     | ");
      }
    }
    serial_->println("");
  }
  
  serial_->println("+----+--------------- + -------------- + -------------- + -------------- + -------------- +");
  serial_->println("");
  uint8_t curentCommand = (readTetrad(25 * 3 + 1, RAMdata) << 4) | readTetrad(24 * 3 + 1, RAMdata);
  serial_->print("                   commandQ.in - "); serial_->println(commandQ.in, HEX);
  serial_->print(" commandQ.command[commandQ.in] - "); serial_->println(commandQ.command[commandQ.in], HEX);
  serial_->print("                  commandQ.out - "); serial_->println(commandQ.out, HEX);
  serial_->print("commandQ.command[commandQ.out] - "); serial_->println(commandQ.command[commandQ.out], HEX);
  serial_->print("                 curentCommand - "); serial_->println(curentCommand, HEX);
  serial_->println("");
}
