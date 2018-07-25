/*
 * PAN-writer-attiny
 * written by Ingo Randolf 2018
 * 
 * GNU GPL 3
 */
 
/*
 * Pinout ATtiny25/45/85
 *
 *                                        --------
 *          (PCINT5/!RESET/ADC0/dW) PB5 -|        |- VCC
 *   (PCINT3/XTAL1/CLKI/!OC1B/ADC3) PB3 -|        |- PB2 (SCK/USCK/SCL/ADC1/T0/INT0/PCINT2)
 *    (PCINT4/XTAL2/CLKO/OC1B/ADC2) PB4 -|        |- PB1 (MISO/DO/AIN1/OC0B/OC1A/PCINT1)
 *                                  GND -|        |- PB0 (MOSI/DI/SDA/AIN0/OC0A/!OC1A/AREF/PCINT0)
 *                                        --------
 *
 */

#include <RingBuffer.h>
/*
 * using BufferUtils by Christopher Baker:
 * https://github.com/bakercp/BufferUtils
 */

#define PARITY_ODD
//#define PREAMBLE_ONLY

/*
 * why ODD:
 * only zero-value should result in a non-valid byte
 * this is only possible with odd parity
 */


/*
 * speed calculations:  
 * 
 * 8-bit preamble: 10101010
 * 
 * 8-bit data
 * 1 parity-bit
 * 1 stop-bit
 * 
 * 10 bit / byte
 *
 * 200us / bit (defined...)
 * = 5000 bit / sec (5000 baud)
 * ~500 byte / sec (- preamble)
 *
 *
 * reaching 9600 baud:
 * 1 / 9600 = 104,16666666 us / bit
 */


long last_bit_time = 0;
long last_data_time = 0;

byte preamble = B10101010;
int write_count = 7;
byte outgoing_byte = 0;

byte parity_count = 0;
byte parity_bit = 0;

const size_t bufferSize = 128;
uint8_t buffer[bufferSize];
RingBuffer ring(buffer, bufferSize);

enum _writestate {
  NONE,
  PREAMBLE,
  DATA,
  PARITY,
  STOP
};

volatile enum _writestate writestate = NONE;

void setup() {

  pinMode(1, OUTPUT); // PB1: resonator out
  
  // setup timer 1 for PWM
  // attiny running on 8MHz
  TCCR1 = (1 << PWM1A) | (1 << COM1A1);
  OCR1C = 24; //24; // == ~3 us ~= 333 333,333 Hz
  OCR1A = 12; //12; // 50% duty

#ifdef PREAMBLE_ONLY
  toPreamble();
#endif

  last_data_time = last_bit_time = micros();
}

void loop() {
  
  long _now = micros();
  
  if ((_now - last_bit_time) >= 200) { //200
    // every 200 us
    // TODO: do this in a timer
    bitOut();
    last_bit_time = _now;
  }

#ifndef PREAMBLE_ONLY
    if ((_now - last_data_time) >= 200000) {  
      putData();      
      last_data_time = _now;
    }
#endif
}

void putData() {

  // use a start-prefix
  sendData('s');
  sendData('t');
  sendData('a');

  // hello, world!
  // problems with String on attiny - write out ascii values
  sendData(104);
  sendData(101);
  sendData(108);
  sendData(108);
  sendData(111);
  sendData(44);
  sendData(32);
  sendData(119);
  sendData(111);
  sendData(114);
  sendData(108);
  sendData(100);
  sendData(33);

  // use a end-prefix
  sendData('e');
  sendData('n');
  sendData('d');
}

//----------------------------------------------
//----------------------------------------------
// FUNCTIONS
//----------------------------------------------
//----------------------------------------------
void sendString(String data, int len) {
  int l = len;//data.length();
  byte* d = data.c_str();

  for (int i=0; i<l; i++) {
    sendData(d[i]);
  }
}

void sendData(byte data) {
  ring.put(data);
}

void sendInt16(int16_t data) {
  ring.put((byte)(data >> 8));
  ring.put((byte)data);
}

inline void pwmOff() {
  TCCR1 &= ~(1 << CS10);
}

inline void pwmOn() {
  TCNT1 = 0;
  TCCR1 |= (1 << CS10);
}


inline void toPreamble() {
  write_count = 7;
  writestate = PREAMBLE;
}

inline void toParity() {
  writestate = PARITY;
}

inline void writeNextByte() {
  ring.get(outgoing_byte);
  write_count = 7;
  parity_count = 0;
  writestate = DATA;
}


void bitOut() {

  switch(writestate) {

    //----------------------------------------
    // NONE
    //----------------------------------------
    case NONE:
      // turn off
      pwmOff();

#ifdef PREAMBLE_ONLY
      toPreamble();
#else
      if (!ring.empty()) {        
        toPreamble();
      }
#endif
      break;

    //----------------------------------------
    // PREAMBLE
    //----------------------------------------
    case PREAMBLE:
      if ( ( write_count >= 0 ) && ( preamble & (1 << write_count) ) ) {
        // 1-bit - value high
        pwmOn();
        
      } else {
        // 0-bit - value low
        pwmOff();
      }

      if (--write_count < 0) {
#ifdef PREAMBLE_ONLY
        toPreamble();
#else
        // switch state
        writeNextByte();
#endif
      }
      break;

    //----------------------------------------
    // DATA
    //----------------------------------------
    case DATA:
      if ((write_count >= 0) && (outgoing_byte & (1 << write_count))) {
        // 1-bit - value high
        pwmOn();
        parity_count++;
      } else {
        // 0-bit - value low
        pwmOff();
      }

      if (--write_count < 0) {

        // set parity bit
#ifdef PARITY_ODD
      if (parity_count % 2 == 0) {
        // make it odd
        parity_bit = 1;
      } else {
        // odd numer
        parity_bit = 0;
      }
#else
      if (parity_count % 2 == 0) {
        // even number
        parity_bit = 0;
      } else {
        // make it even
        parity_bit = 1;
      }      
#endif
        // next write parity bit
        toParity();
      }
      break;

    //----------------------------------------
    // PARITY
    //----------------------------------------
    case PARITY:
      if (parity_bit) {
        // 1-bit - value high
        pwmOn();
      } else {
        // 0-bit - value low
        pwmOff();
      }

      writestate = STOP;
      break;

    case STOP:
      if (!ring.empty()) {
        // more data to send, 1-bit stop-bit
        pwmOn();
        writeNextByte();
      } else {
        // end of transmission, 0-bit stop bit... ?
        pwmOff();
        writestate = NONE;
      }      
      break;
  }
  
}
