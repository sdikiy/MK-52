#include "MKCALC.h"
#include <avr/interrupt.h>
#include <SPI.h>
#include <Arduino.h>

MKCALC mk52 = MKCALC();

// SPI interrupt routine
ISR(SPI_STC_vect) {
  mk52.interruptSPI();
}

void setup() {
  pinMode( 6, OUTPUT);      // pin PD6 - in SPI interrupt, used for debug
  PORTD &= ~(1 << 6);
  PORTD |= (1 << 7);        // write to MK OFF
  pinMode( 7, OUTPUT);      // pin PD7 - allow write to MK ON/OFF
  pinMode( 8, OUTPUT);      // pin PB0 - oscilloscope trigger, used for debug
  pinMode( 9, OUTPUT);      // pin PB1 - wired to pin /ss of SPI

  Serial.begin(115200, SERIAL_8N1);
  mk52.setSerial(&Serial);
  TIMSK0 = 0;

  //SPI CONFIG
  //Bit 6 – SPE0: SPI0 Enable. When the SPE bit is written to one, the SPI is enabled.
  //SPCR |= (1<<SPE);
  SPCR |= _BV(SPE);
  //Bit 5 – DORD0: Data0 Order: one - the LSB, zero - the MSB
  //SPCR &= ~(1<<DORD);
  SPCR |= (1<<DORD);
  //Bit 4 – MSTR0: Master/Slave0 Select
  /* This bit selects Master SPI mode when written to one, and Slave SPI mode when written logic zero. If SS
  is configured as an input and is driven low while MSTR is set, MSTR will be cleared, and SPIF in SPSR
  will become set. The user will then have to set MSTR to re-enable SPI Master mode.*/
  SPCR &= ~(1<<MSTR);
  pinMode(MISO, OUTPUT);
  //Bit 3 – CPOL0: 0 - Rising/Falling; 1 - Falling/Rising
  //SPCR |= _BV(CPOL);
  //SPCR &= ~(1<<CPOL);
  SPCR |= (1<<CPOL);
  //Bit 2 – CPHA0: 0 - Sample/Setup;   1 - Setup/Sample
  //SPCR |= _BV(CPHA);
  //SPCR &= ~(1<<CPHA);
  SPCR |= (1<<CPHA);
  //Bit 7 – SPIE0: SPI0 Interrupt Enable
  SPCR |= _BV(SPIE);

}

void loop() {
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case '1':
        Serial.println("ArduinoNano");
        Serial.print("cmd_state ");
        Serial.println(mk52.cmd_state);
        break;
      case '2':
        Serial.println("set SPI /SS = 1");
        mk52.skipClockCycle = 1;
        break;
      case '3':
        mk52.byteToMk = ((mk52.byteToMk == 0x55) ? (0xAA) : (0x55));
        mk52.byteWriteToMkStatus = 1;
        break;
      case 'r':
        mk52.cmd_state = WAIT_A_START;
        break;
      case 's':
        mk52.MemoryPagesPrint();
        break;
    }
  }   
}
