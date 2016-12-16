/***************************************************
 * MKCALC.cpp - Core of MKCalk library for linking *
 *              MK-52/61 calculators and Arduino.  *
 *              Created by _______________________ *
 *              2016-_____________________________ *
 ***************************************************/

#include "Arduino.h"
#include "MKCALC.h"

volatile byte MKCALC::cmd_state = WAIT_A_START;
volatile uint8_t MKCALC::RAMdata[315];
volatile uint16_t MKCALC::posRAM = 0;
volatile uint8_t MKCALC::skipClockCycle = 0;
volatile uint8_t MKCALC::tempOut = 0;
volatile uint8_t MKCALC::byteWriteToMkStatus = 0;
volatile uint8_t MKCALC::byteToMk = 0xAA;
volatile uint16_t MKCALC::byteNumToMk = 55;


MKCALC::MKCALC() {
  //serial_ = NULL;
  cmd_state = WAIT_A_START;
}

void MKCALC::setSerial(HardwareSerial* serial) {
  serial_ = serial;
}

byte MKCALC::readTetrad(int index) {
  byte tetrad = 0;
  //See: Arduino/hardware/tools/avr/avr/include/avr/builtins.h
  tetrad = ((index & 0x01) == 1) ? (RAMdata[index >> 1]) : (__builtin_avr_swap(RAMdata[index >> 1]));
  tetrad = tetrad & 0x0F;
  //tetrad = ((tetrad & 0x01) << 3) | ((tetrad & 0x02) << 1) | ((tetrad & 0x04) >> 1) | ((tetrad & 0x08) >> 3);
  return (tetrad);
}

void MKCALC::interruptSPI() {
  SPDR = __builtin_avr_swap( (posRAM == 314) ? (RAMdata[0]) : (RAMdata[posRAM + 1])); //Set data to send by SPI
  byte c = __builtin_avr_swap(~SPDR);  //read from SPI Data Register (double buffered)
  if (posRAM == 0) PORTB |= (1 << 0); //oscilloscope trigger start

  
  PORTD |= (1 << 7); //write to MK OFF
  if ((posRAM == (byteNumToMk - 1)) && (cmd_state == READ_BITS) && (byteWriteToMkStatus == 2)) {
    PORTD &= ~(1 << 7); //write to MK ON
    byteWriteToMkStatus = 0;
  }
  if ((posRAM == byteNumToMk) && (cmd_state == READ_BITS) && (byteWriteToMkStatus == 1)) {
    //RAMdata[57] = 0x55;
    c = byteToMk;
    byteWriteToMkStatus = 2;
  }
  

  RAMdata[posRAM] = c;
  posRAM = posRAM + 1;

  if (posRAM == 315) {
    posRAM = 0;
    if ( ((RAMdata[0] & RAMdata[3] & RAMdata[6] & RAMdata[9] & RAMdata[12] & 0x0F )
        | (RAMdata[2] & RAMdata[5] & RAMdata[8] & RAMdata[11]              & 0xF0 )) == 0xFF ) {
      cmd_state = READ_BITS;
    } else {
      cmd_state = WAIT_A_START;
      skipClockCycle = 1;
    }
  }

  if (skipClockCycle == 1) { //skip one clock cycle
    PORTB |= (1 << 1); 
    skipClockCycle = 0;
    NOP5; NOP5; NOP5; NOP5;
    PORTB &= ~(1 << 1);
  }

  PORTB &= ~(1 << 0); //oscilloscope trigger end
}

//вывод всех страниц памяти в виде тетрад в HEX виде
void MKCALC::MemoryPagesPrint() {
  serial_->print("cmd_state in MemoryPagesPrint - ");
  serial_->println(cmd_state);
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
