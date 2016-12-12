/***************************************************
 * MKCALC.cpp - Core of MKCalk library for linking *
 *              MK-52/61 calculators and Arduino.  *
 *              Created by _______________________ *
 *              2016-_____________________________ *
 ***************************************************/

#include "Arduino.h"
#include "MKCALC.h"

volatile byte MKCALC::cmd_state = WAIT_A_START;
uint16_t MKCALC::BitCounter;
byte MKCALC::port_B;
byte MKCALC::port_D;
volatile uint8_t MKCALC::RAMdata[315];
long MKCALC::startTimeCount;
long MKCALC::waitTimeCount;
long MKCALC::readTimeCount;
//byte MKCALC::M1_tetrads[210];
//byte MKCALC::M2_tetrads[210];
//byte MKCALC::M3_tetrads[210];
volatile uint16_t MKCALC::posRAM = 0;
volatile uint8_t MKCALC::commandT = 0;
volatile uint8_t MKCALC::tempOut = 0;


MKCALC::MKCALC() {
  BitCounter=0;               //счетчик битов в кольцевой ОП
  pinMode(PD3_SY, INPUT);
  pinMode(PD2_F4, INPUT);
  pinMode(PD4_PrIN, INPUT);
  Timsk0_temp = TIMSK0;
  TIMSK0 = 0;
  //serial_ = NULL;
  cmd_state = WAIT_A_START;
  //setInterrupt();
}

void MKCALC::setSerial(HardwareSerial* serial) {
  serial_ = serial;
}

void MKCALC::setInterrupt() {
  //привязываем прерывание
  // привязываем 0-е прерывание к функции SYNC_int().
  //attachInterrupt(digitalPinToInterrupt(SYNC), SYNC_int, FALLING);
  // привязываем 1-е прерывание к функции F4_int() по изменению с 0->1.  
  //attachInterrupt(digitalPinToInterrupt(PD2_F4), F4_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(PD2_F4), F4_int, FALLING);
}

void MKCALC::clrInterrupt() {
  detachInterrupt(digitalPinToInterrupt(PD2_F4));
}

byte MKCALC::getState() {
  return cmd_state;
}

void MKCALC::setState(byte newState) {
  cmd_state = newState;
}

void MKCALC::ringToMem() {
  startTimeCount = 0;
  waitTimeCount = 0;
  readTimeCount = 0;
  clrInterrupt();
  Timsk0_temp = TIMSK0;
  TIMSK0 = 0;
  cmd_state = WAIT_A_START;
  serial_->print("cmd_state in ringToMem - ");
  serial_->println(getState());
  setInterrupt();
  /**/
  for (long cc=0; cc < 17000; cc++) {
    //PORTB &= ~(1 << 4);
    if (getState() == WAIT_A_START) startTimeCount++;
    if (getState() == WAIT_A_MARK) waitTimeCount++;
    if (getState() == READ_BITS) readTimeCount++;
  }
  /**/
  clrInterrupt();
  TIMSK0 = Timsk0_temp;
  serial_->print("startTimeCount = ");
  serial_->println(startTimeCount);
  serial_->print("waitTimeCount = ");
  serial_->println(waitTimeCount);
  serial_->print("readTimeCount = ");
  serial_->println(readTimeCount);
}

void MKCALC::ringToMemNoInterrupt() {
  bool syncF = 0;
  //uint8_t pinPortD;
  cmd_state = WAIT_A_START;
  serial_->print("cmd_state in ringToMemNoInterrupt - ");
  serial_->println(getState());
  /**/
  __builtin_avr_cli();
  for (uint16_t cc=0; cc < 64000; cc++) {
    //pinPortD = PIND;
    if (syncF) {
      if (PD2_F4_MASKA & PIND) {
        F4_int();
        //PORTB &= ~(1 << 4);
        //PORTB |= (1 << 4);
        syncF = 0;
      }
    } else {
      if (!(PD2_F4_MASKA & PIND)) {
        syncF = 1;
      }
    }
  }
  __builtin_avr_sei();
  /**/
}

//запись бита в массив RAMdata
void MKCALC::writeBit(int index, bool value) {
  int byte_index = index>>3;
  byte bits = RAMdata[byte_index];
  byte bit_index = index & 0x7; // same as (index - byte_index<<3) or (index%8)
  byte mask = 1<<bit_index;
  byte new_bits = (bits & ~mask) | value<<bit_index;
  RAMdata[byte_index] = new_bits;
}

//чтение бита из массива RAMdata
bool MKCALC::readBit(int index) {
  int byte_index = index>>3;
  byte bits = RAMdata[byte_index];
  int bit_index = index & 0x7; // same as (index - byte_index<<3)
  byte mask = 1<<bit_index;
  return (0!=(bits & mask));
}

//функция для чтения тетрады из смассива ОП
//index - номер терады
byte MKCALC::readTetrad(int index) {
  byte tetrad = 0;
  ///int byte_index = index>>1;        //сдвигаем номер тетрады на один бит вправо - получаем номер байта
  ///byte bits = RAMdata[byte_index];  //считываем нужный байт
  ///int tetrad_index = index & 0x1;   // младшая или старшая тетрада, оставляем только нулевой бит 
  //if (tetrad_index==0) 
  if ((index & 0x01) == 1) {
    tetrad = RAMdata[index >> 1] & 0x0F;
  } else {
    //tetrad = (RAMdata[index >> 1] >> 4) & 0x0F;
    //tetrad = ((RAMdata[index >> 1] << 4) | (RAMdata[index >> 1] >> 4)) & 0x0F; // swap & 0x0F ?
    // See - Arduino/hardware/tools/avr/avr/include/avr/builtins.h
    tetrad = __builtin_avr_swap(RAMdata[index >> 1] & 0xF0);
  }
  tetrad = ((tetrad & 0x01) << 3) | ((tetrad & 0x02) << 1) | ((tetrad & 0x04) >> 1) | ((tetrad & 0x08) >> 3);
  return (tetrad);
}

//подпрограмма обработки прерываний
void MKCALC::F4_int() {
  ///PORTB |= (1 << 1);
  //port_D=PIND;
  RAMdata[BitCounter>>3] = ((RAMdata[BitCounter>>3] << 1) | (!(PD4_PrIN_MASKA & PIND)));
  BitCounter++;
  switch (cmd_state) { 
    case WAIT_A_START:
      if ((PD3_SY_MASKA & PIND)==LOW) {
        cmd_state = WAIT_A_MARK;
        BitCounter=0;
      }
      break;
    case WAIT_A_MARK:
      if (BitCounter == 168) { // 14*4*3
        if ( ((RAMdata[0] & RAMdata[3] & RAMdata[6] & RAMdata[9] & RAMdata[12] & 0x0F )
            | (RAMdata[2] & RAMdata[5] & RAMdata[8] & RAMdata[11]              & 0xF0 )) == 0xFF ) {
          cmd_state = READ_BITS;
        } else {
          BitCounter=0;
        }
      }
      break;
    case READ_BITS:
      if (BitCounter == 2520) BitCounter = 0;
      break;
  }
  ///PORTB &= ~(1 << 1);
}

void MKCALC::interruptSPI() {
  byte c = ~SPDR;  // grab byte from SPI Data Register
      //PORTB |= (1 << 1);
      //PORTB &= ~(1 << 1);
  //if (posRAM == 314) {SPDR = ~RAMdata[0];} else {SPDR = ~RAMdata[posRAM + 1];}
  if (posRAM == 314) {SPDR = RAMdata[0];} else {SPDR = RAMdata[posRAM + 1];}
  //SPDR = ~RAMdata[posRAM+1];
  ///if (posRAM == 0) {SPDR = ~RAMdata[314];} else {SPDR = ~RAMdata[posRAM - 1];}
  ///SPDR = 0xFF;
  if (posRAM == 0) PORTB |= (1 << 0);
  if (commandT == 1) {
    PORTB |= (1 << 1);
    commandT = 0;
    NOP5;
    NOP5;
    NOP5;
    NOP5;
    PORTB &= ~(1 << 1);
  }
  // add to buffer if room
  RAMdata[posRAM] = c;
  posRAM = posRAM + 1;
  ///tempOut = RAMdata[posRAM];
  if (posRAM == 315) {
    posRAM = 0;
    if ( ((RAMdata[0] & RAMdata[3] & RAMdata[6] & RAMdata[9] & RAMdata[12] & 0x0F )
        | (RAMdata[2] & RAMdata[5] & RAMdata[8] & RAMdata[11]              & 0xF0 )) == 0xFF ) {
      cmd_state = READ_BITS;
    } else {
      PORTB |= (1 << 1);
      NOP5;
      NOP5;
      NOP5;
      NOP5;
      PORTB &= ~(1 << 1);
    }
  }
  PORTB &= ~(1 << 0);
}

//TODO MARK without CN
void MKCALC::F4_interrupt() {
  //PORTB |= (1 << 4);
  port_D=PIND;
  switch (cmd_state) {
    case WAIT_A_START:
      BitCounter = 0;
      RAMdata[0] = 0;
      RAMdata[1] = 0;
      RAMdata[2] = 0;
      cmd_state = WAIT_A_MARK;
      break;
    case WAIT_A_MARK:
      uint8_t nibble;
      nibble = (BitCounter>>3)%3;
      if (!(PD4_PrIN_MASKA & port_D)) {
        if (RAMdata[nibble] == 36) {
          BitCounter = 108 + nibble;
          cmd_state = READ_BITS;
        } else {
          RAMdata[nibble] = 0;
        }
      } else {
        RAMdata[nibble] =  RAMdata[nibble] + 1;
      }
      BitCounter++;
      if (BitCounter >= 5041) {
      //cmd_state = READ_BITS;
        BitCounter = 0;
        RAMdata[0] = 0;
        RAMdata[1] = 0;
        RAMdata[2] = 0;
      }
      break;
    case READ_BITS:
      RAMdata[BitCounter>>3] = ((RAMdata[BitCounter>>3] << 1) | (!(PD4_PrIN_MASKA & port_D)));
      BitCounter++;                                      //увеличиваем на 1 счетчик битов
      if (BitCounter == 2520) BitCounter = 0;            //как только записали все 2519 бит сбрасываем счетчик битов и записываем сначала
      break;
  }
  //PORTB &= ~(1 << 4);
}

//вывод всех страниц памяти в виде тетрад в HEX виде
void MKCALC::MemoryPagesPrint() {
  serial_->print("cmd_state in MemoryPagesPrint - ");
  serial_->println(getState());
  byte k=0;
  byte l=0;
  switch (cmd_state) { 
    case WAIT_A_START:
      serial_->println("WAIT_A_START...");
      //break;

    case WAIT_A_MARK:
      serial_->println("WAIT_A_MARK...");
      //break;

    case READ_BITS:
      serial_->println("READ_BITS...");
      serial_->print("+----+-");
      for(int i=0; i<=34; i++) {
        serial_->print("--"); k++;
        if (k==7) {serial_->print(" + "); k=0; }
      }
      serial_->print("\r\n");

      k=0;
      serial_->print("|    | ");
      for(int i=0; i<=69; i++) { //счетчик тетрад
        serial_->print(k, HEX);
        //serial_->print(" ");
        k++;
        if (k==14) {
          serial_->print(" | ");
          k=0;
        }
      }
      serial_->println("");

      k=0;
      serial_->print("+----+-");
      for(int i=0; i<=34; i++) {
        serial_->print("--"); k++;
        if (k==7) {serial_->print(" + "); k=0; }
      }
      serial_->print("\r\n");

      serial_->print("| M1 | ");
      k=0;
      l=0;
      for(int i=0; i<=209; i++) { //счетчик тетрад
        //M1_tetrads[i]= readTetrad(i*3);       //209*3=627
        //M2_tetrads[i]= readTetrad(i*3+1);     //209*3+1=628
        //M3_tetrads[i]= readTetrad(i*3+2);     //209*3+2=629
        serial_->print(readTetrad(i*3), HEX);
        ///serial_->print(" ");
        k++;
        if (k==14) {
          serial_->print(" | ");
          k=0;
          l++;
          if (l==5) {serial_->print("\r\n     | "); l=0;}
        }
      }
      serial_->print("\r\n");

      k=0;
      l=0;
      serial_->print("| M2 | ");
      for(int i=0; i<=209; i++) { //счетчик тетрад
        serial_->print(readTetrad(i*3+1), HEX);
        ///serial_->print(" ");
        k++;
        if (k==14) {
          serial_->print(" | ");
          k=0;
          l++;
          if (l==5) {serial_->print("\r\n     | "); l=0;}
        }
      }
      serial_->println("");

      k=0;
      l=0;
      serial_->print("| M3 | ");
      for(int i=0; i<=209; i++) { //счетчик тетрад
        serial_->print(readTetrad(i*3+2), HEX);
        ///serial_->print(" ");
        k++;
        if (k==14) {
          serial_->print(" | ");
          k=0;
          l++;
          if (l==5) {serial_->print("\r\n     | "); l=0;}
        }
      }
      serial_->print("\r\n");

      k=0;
      serial_->print("+----+-");
      for(int i=0; i<=34; i++) {
        serial_->print("--"); k++;
        if (k==7) {serial_->print(" + "); k=0; }
      }
      serial_->print("\r\n");
      serial_->println("");
      break;
    }
}
