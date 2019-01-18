/*
 * PAN-writer-attiny
 * written by Ingo Randolf 2018
 * 
 * GNU GPL 3
 */

#include <Packetizer.h>
/*
 * using Packetizer
 * https://github.com/i-n-g-o/Packetizer
 */
#include <CircularBuffer.h>
/*
 * using BufferUtils by Christopher Baker:
 * https://github.com/bakercp/BufferUtils
 */


#define PARITY_ODD;
/*
 * why ODD:
 * only zero-value should result in a non-valid byte
 * this is only possible with odd parity
 */

 #define IS_FLIPPED
 /*
  * input is logically flipped.
  * this depends how we use the optocoupler and may be removed
  */


byte preamble = B10101010;
byte incoming_preamble = 0;

byte incoming_byte = 0;
bool incoming_byte_valid = false;
byte bit_counter = 0;
byte parity_count = 0;

enum _readstate {
  PREAMBLE,
  DATA,
  PARITY,
  STOP
};

volatile enum _readstate readstate = PREAMBLE;

const size_t bufferSize = 128;
uint8_t buffer[bufferSize];
CircularBuffer ring(buffer, bufferSize);

uint8_t error_buffer[bufferSize];
CircularBuffer error_ring(error_buffer, bufferSize);

boolean on = false;
volatile boolean stop_req = false;

Packetizer slicer;

//----------------------------------------------
//----------------------------------------------
// SETUP
//----------------------------------------------
//----------------------------------------------
void setup() {

  // data input
  pinMode(2, INPUT); // PD2: ext interrupt

  pinMode(7, OUTPUT); //PD7: error-led
  pinMode(8, OUTPUT); //PB0: pin-interrupt (1-bits)
  pinMode(9, OUTPUT); //PB1: timer interrupt
  pinMode(10, OUTPUT); //PB2: bit-output
  pinMode(11, OUTPUT); //PB3: bit-set toggle
  pinMode(12, OUTPUT); //PB4: preamble detected
  pinMode(13, OUTPUT); //PB5: data-match output

  PORTB = 0;

  // uno running on 16MHz
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1BH = 0;
  OCR1BL = 0;
  TCNT1 = 0;
  
#ifdef IS_FLIPPED
  // falling edge
  EICRA = (1 << ISC01);
#else
  // rising edge
  EICRA = (1 << ISC01) | (1 << ISC00);
#endif

  
  Serial.begin(115200);
  Serial.println("init");


  // setup packetizer
  byte result = slicer.init(128);
  if (result != pz_noErr)
  {
    //error handling
    Serial.println("error init");
  }

  // use same start/end conditions as in writer
  slicer.setStartCondition("sta");
  slicer.setEndCondition("end");

  // set package-callback
  slicer.onPacket(myOnPacket);


  // start listening for preamble
  resetPreamble();
}


//----------------------------------------------
//----------------------------------------------
// LOOP
//----------------------------------------------
//----------------------------------------------
void loop() {

  while(!error_ring.empty()) {
    PORTD |= (1 << PD7);
    uint8_t i;
    error_ring.get(i);
    Serial.print("error: ");
    Serial.println((char)i);
  }
  PORTD &= ~(1 << PD7);


  while(!ring.empty()) {
    uint8_t i;
    ring.get(i);    
    slicer.appendData(i);
  }
}


void myOnPacket(byte* _buffer, unsigned int _bufferSize)
{
  // buffer is caller-owned
  // copy data if you need it later

  Serial.print("buffer: ");
  Serial.write(_buffer, _bufferSize);
  Serial.println();
}


//----------------------------------------------
//----------------------------------------------
// FUNCTIONS
//----------------------------------------------
//----------------------------------------------
inline void resetPreamble() {
  // end of line...?
  // waiting for next preamble
  stopTimer();
  stop_req = true;
  
  incoming_preamble = 0;
  readstate = PREAMBLE;

  PORTB &= ~(1 << PB5);
  PORTB &= ~(1 << PB4);

  startInt0();
}

inline void readNextByte() {
  // reset, prepare for next byte
  incoming_byte = 0;
  incoming_byte_valid = false;
  bit_counter = 0;
  parity_count = 0;
  
  // read data
  readstate = DATA;
}


void setBitValue(boolean _b) {

  // got a bit
  // if we dot not yet have a preamble add it to preamble

  switch(readstate) {
    case PREAMBLE:
      // shift bit into incoming
      if (_b) {
        incoming_preamble = (incoming_preamble << 1) + 1;
      } else {
        incoming_preamble = (incoming_preamble << 1);
      }

      if (incoming_preamble == 0) {
        resetPreamble();
        return;
      }
      
      if (incoming_preamble == preamble) {        
        // got a preamble... 
        PORTB |= (1 << PB4);
        // turn off data-match led
        PORTB &= ~(1 << PB5);
        
        // getting intersting here.
        incoming_preamble = 0;
        readNextByte();
        
      } else {
        PORTB &= ~(1 << PB4);
        // turn off data-match led
        PORTB &= ~(1 << PB5);
      }
      break;


    case DATA:
      // we already received a preamble, lets add this bit to incoming byte      
      if (_b) {
        incoming_byte = (incoming_byte << 1) + 1;
        parity_count++;
      } else {
        incoming_byte = (incoming_byte << 1);
      }

      if (++bit_counter == 8) {
        readstate = PARITY;
      }
      break;
      

    case PARITY:

      // 9-th bit = parity bit
      
      // parity bit
#ifdef PARITY_ODD
      if(parity_count % 2 == 0) {
        // even number
        incoming_byte_valid = _b;
        
        if (!_b) {
          // MISSMATCH!
          //resetPreamble();
        } else {
          // we got a full byte...
          //receivedByte(incoming_byte);
        }
      } else {
        // odd number
        incoming_byte_valid = !_b;
        
        if (_b) {
          // MISSMATCH!
          //resetPreamble();
        } else {
          // we got a full byte...
          //receivedByte(incoming_byte);
        }
      }
#else
      if(parity_count % 2 == 0) {
        // even number
        incoming_byte_valid = !_b;
        if (_b) {
          // MISSMATCH!
          //ring.put('.');
          //resetPreamble();
        } else {
          // we got a full byte...
          //receivedByte(incoming_byte);
        }
      } else {
        // odd number
        incoming_byte_valid = _b;
        if (!_b) {
          // MISSMATCH!
          //ring.put('.');
          //resetPreamble();
        } else {
          // we got a full byte...
          //receivedByte(incoming_byte);
        }
      }
#endif

      readstate = STOP;      
      break;

    case STOP:
      // 10-th bit = stop bit
      if (incoming_byte_valid) {
        ring.put(incoming_byte);
      } else {
        slicer.reset();
        error_ring.put(incoming_byte);
      }
      
      if (_b) {
        // continue;
        readNextByte();
      } else {
        // ok... transmission stop        
        resetPreamble();        
      }
      
      break;
  }

  if (_b) {
    PORTB |= (1 << PB2);
  } else {
    PORTB &= ~(1 << PB2);
    startInt0();
  }

  // toggle
  if (PORTB & (1 << PB3)) {
    PORTB &= ~(1 << PB3);    
  } else {
    PORTB |= (1 << PB3);
  }
}

//----------------------------------------------
//----------------------------------------------
// FUNCTIONS
//----------------------------------------------
//----------------------------------------------

inline void startTimer(int cmp) {

  TCNT1H = 0;
  TCNT1L = 0;
  // target = 240  
  OCR1BH = (byte)(cmp >> 8);
  OCR1BL = (byte)cmp; //145; // clk/2: 20 us

  // enable timer
  TIMSK1 |= (1 << OCIE1B);
  TCCR1B = (1 << CS11);

  //GTCCR &= ~(1 << TSM);
  stop_req = false;
}

inline void stopTimer() {  
  TCCR1B = 0;
  TIMSK1 &= ~(1 << OCIE1B);
  TCNT1L = 0;
  TCNT1H = 0;  
}

inline void startInt0() {
  // turn on INT0
  EIMSK |= (1 << INT0);
  // turn interrupt flag off
  EIFR |= (1 << INTF0);
  PORTB |= (1 << PB0);
}

inline void stopInt0() {
  // turn off INT0
  EIMSK &= ~(1 << INT0);
  PORTB &= ~(1 << PB0);
}


//----------------------------------------------
//----------------------------------------------
// ISR
//----------------------------------------------
//----------------------------------------------

//----------------------------------------------
// pin-interrupt
ISR(INT0_vect) {

  // rising or falling edge
  // depends if IS_FLIPPED is defined

  stopTimer();
  stopInt0();

  // bit length: 200us
  // expect next bit in 200-300us
  // if no rising interrupt (1-bit) arrives in time, it is a 0-bit
  
  startTimer(590); // 300 us

  // 1-bit
  setBitValue(true);
}


//----------------------------------------------
// timer interrupt
ISR(TIMER1_COMPB_vect) {

  stopTimer();

  // measure...
  if (PIND & (1 << PD2)) {
    // high
#ifdef IS_FLIPPED
    setBitValue(false);
#else
    setBitValue(true);
#endif
  } else {
    // low
#ifdef IS_FLIPPED
    setBitValue(true);
#else
    setBitValue(false);
#endif
  }

  if (!stop_req) {
    // start timer - next expected bit in 200us
    startTimer(386); // 200 us
  }
  
  // toggle output for debugging purpose
  if (PORTB & (1 << PB1) ) {
    PORTB &= ~(1 << PB1);
  } else {
    PORTB |= (1 << PB1);
  }
}

