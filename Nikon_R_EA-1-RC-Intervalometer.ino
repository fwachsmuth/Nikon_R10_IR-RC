/* ATtiny85 IR Remote Control Receiver
 * To Do:
 *  [ ] Consider lightmetering in interval modes (long ones maybe?)
 *  [ ] do we need repeat?
 *  [ ] Allow IR code learning to EEPROM


This code simulates the Nikon EA-1 Remote Control Switch and adds extra functions, 
like IR-Remote control support and a nifty intervalometer with 416 different intervals
ranging from 200ms to more than 37 hours.
The power needed for the circuit is harvested from the camera (the camera connector has no actual power output).

Some documentation on the hardware below:

Pin numbering, looking onto the camera side jack (or PCB Front)

GND 3 Green - - Brown   4  
    2 Red   - - Yellow  5 
      space     space 
+9V 1 Blue  - - White   6

Inside the handpiece of the remote control:
Sliders move from 0 to 1 for lightmeter and from 1 to 2 for exposure.

  |-------|      |---------|       |------|  2 expose 
  |       |                        |      | 
  |-------|------|         |       |------|  1 lightmeter on 

  |       |      |         |       |      |  0 no connect 
Brown    Blue   Red      Green  Yellow   White 
  4       1      2         3       5      6

  \ Hold /\ Load /\ Start /         \Light/                    

The "Load" phase charges a capacitor to ease sudden startup and assure even exposure timing. 
The entire start mechanism of the Nikon R8/R10 is rather sophisticated and barely documented.
We need four relays here to follow the protocol. Not following the protocol can work, but might 
damage the camera... who wants that?

*/
#include <util/delay.h> // built-in delay() runs too fast due to timer0 usage in IR receiver ISR

/* Ella LED FB EF 00 
 
#define RC_CODE               0xEF00  // LED-Kette
#define ALU_CENTER_KEY       0x15  // Menu
#define MENU_KEY          0x09  // Center
#define DOWN_KEY            0x0D  // Down
#define UP_KEY            0x05  // Up
#define LEFT_KEY       0x08  // Left
#define RIGHT_KEY         0x0A  // Right
#define ALU_PLAY_KEY        0x16  // Run
*/

/* Apple Remote (all models) 87 EE */
// see https://en.wikipedia.org/wiki/Apple_Remote


// *** defines and variables ***************************

#define RC_CODE           0x87EE // Apple
#define UP_KEY            0x05  // Up
#define DOWN_KEY          0x06  // Down
#define LEFT_KEY          0x04  // Left
#define RIGHT_KEY         0x03  // Right
#define MENU_KEY          0x01  // Menu
#define ALU_CENTER_KEY    0x2E  // Center (Alu RC)
#define ALU_PLAY_KEY      0x2F  // Play (Alu RC)
#define WHITE_PLAY_KEY    0x02  // Play (old white Apple Remote)

#define LM_MODE_1ST_SINGLESHOT  1   // On the first single exposure shot, we start with lightmetering. 
#define LM_MODE_SUB_SINGLESHOT  2   // Subsequent shots do not meter the light again.

// These are our four relays
int lightmeterPin = 3;  // K4 ye + wh : Lightmeter
int startPin = 4;       // K3 re + gr : Start
int loadPin = 1;        // K2 bl + re : Load
int holdPin = 0;        // K1 br + bl : Hold

int ledPin = 1;         // LED is parallel to load-relay

volatile int irBits;   // This counts the incoming bits from NEC-encoded IR transmitters in the ISR
volatile unsigned long receivedData;  // This contains the received code

// These are used to maintain the different intervals 
byte oldIntervalStep = 31;  // This is the initial OCR1C value (at 1 MHz / Prescaler 16384 = 524ms)
byte newIntervalStep = 31;  // store it here after a change, since we only update OCR1C in the ISR
int postscaler = 2;         // the postscaler doubles the timer1 by POT factors up to 2^16 (39:04:20)
volatile int divider = 0;   // needed in ISR to do some modulo for intervals > 4.3 sec

byte lmMode = LM_MODE_1ST_SINGLESHOT;
bool blinkFlag = true;       // This one is used to confirm reception fo valid IR codes
bool aluRemote = false;     // Apple's Alumium Remote has one more key than the white one. Reading
                            // that key is ambigious, so we "lock in" to a safe mode once we know for sure
                            // that our RC is not the white one.


// *** Setup **********************************************
void setup() {
  pinMode(lightmeterPin, OUTPUT);     
  pinMode(holdPin, OUTPUT);     
  pinMode(loadPin, OUTPUT);     
  pinMode(startPin, OUTPUT);     

  noInterrupts();
  
  // Set up timeStampr/Counter0 (assumes 1MHz clock)
  TCCR0A = 0;                 // No compare matches
  TCCR0B = 3<<CS00;           // Prescaler 64
  
  // Set up INT0 interrupt on PB2
  MCUCR = MCUCR | 2<<ISC00;   // Interrupt on falling edge
  GIMSK = GIMSK | 1<<INT0;    // Enable INT0
  irBits = 32;                // Wait for NEC-IR AGC start pulse

  // Set up timeStampr/Counter1 (assumes 1MHz clock speed, saving some power)
  TCNT1 = 0;
  TCCR1 = 0;
  OCR1C = oldIntervalStep;    
  OCR1A = OCR1C;              // interrupt COMPA
  TCCR1 |= (1 << CTC1);       // CTC
  TCCR1 |= (1 << CS13) | (1 << CS12) | (1 << CS11) | (1 << CS10);   // Prescaler 2^14 = 16384
 
  interrupts();
}


void ReceivedCode(boolean Repeat) {
  // Check if Transmitter is not an Apple Remote
  if ((receivedData & 0xFFFF) != RC_CODE) {
    if (!Repeat) blinkLED();
    int key = receivedData>>16 & 0xFF; // extracting the command byte, full 8 bits  
    // We could actually learn some arbitrary codes here and keep them in the EEPROM. Wouldn't
    // be a very interactive experience though, so skipping that part until someone asks for it.
    if ((key != 0xFF) && !Repeat && !digitalRead(lightmeterPin))      startRunWithMetering();
    else if ((key != 0xFF) && !Repeat && digitalRead(lightmeterPin))  stopRun();

  } else { // This is comfort mode with the Apple Remote. :)
    int key = receivedData>>17 & 0x7F; // extracting the command byte, ignoring the 1-bit to match all Apple Remotes

    // If we receive codes unique to an alu RC, let's remember that to fight ambiguity
    if (((key == ALU_CENTER_KEY) && !Repeat) || ((key == ALU_PLAY_KEY) && !Repeat)) aluRemote = true;
    
    // Now let's determine what to do
    if ((key == ALU_PLAY_KEY) && !Repeat && !digitalRead(lightmeterPin))      startRunWithMetering();
    else if ((key == ALU_PLAY_KEY) && !Repeat && digitalRead(lightmeterPin))  stopRun();
    else if ((key == ALU_CENTER_KEY) && !Repeat) {
      if (lmMode == LM_MODE_1ST_SINGLESHOT) meterOnce();
      singleFrame();
    } else if ((key == MENU_KEY) && !Repeat) { //MENU_KEY
      if (TIMSK & ( 1 << OCIE1A )) TIMSK &= ~(1 << OCIE1A);  // enable timer interrupt
      else                         TIMSK |= (1 << OCIE1A);   // disable timer interrupt
    } else if ((key == DOWN_KEY) && !Repeat) {
      oldIntervalStep = newIntervalStep;
      newIntervalStep = constrain(oldIntervalStep + 10, 1, 255);
    } else if ((key == UP_KEY) && !Repeat) {
      oldIntervalStep = newIntervalStep;
      if (postscaler <= 4) {  // This could lead to intervals <200ms, which we do want to avoid
        newIntervalStep = constrain(oldIntervalStep - 10, 11, 255);
      } else {
        newIntervalStep = constrain(oldIntervalStep - 10, 1, 255);
      }
    } else if ((key == RIGHT_KEY) && !Repeat) postscaler = constrain(postscaler * 2, 1, 32768);
    else if ((key == LEFT_KEY) && !Repeat) {
      postscaler = constrain(postscaler / 2, 1, 32768);
      if ((newIntervalStep < 11) && (postscaler <= 4)) newIntervalStep = 11; }
    else if ((key == WHITE_PLAY_KEY) && !Repeat && !aluRemote && !digitalRead(lightmeterPin)) startRunWithMetering();
    else if ((key == WHITE_PLAY_KEY) && !Repeat && !aluRemote && digitalRead(lightmeterPin))    stopRun();
    else blinkFlag = false;   // if we received a partial or garbled IR code, let's not confirm reception
  }
  if (blinkFlag) blinkLED();
  blinkFlag = true;   // Let's assume the next code is a valid one
}


// ISR 1 - called on IR signal coming in (falling edge on PB2)
ISR(INT0_vect) {
  int timeStamp = TCNT0;
  int overFlow = TIFR & 1 << TOV0;
  if (irBits == 32) {
    // Let's check for AGC
    if ((timeStamp >= 194) && (timeStamp <= 228) && (overFlow == 0)) {
      receivedData = 0; irBits = 0;
    } else if ((timeStamp >= 159) && (timeStamp <= 193) && (overFlow == 0)) ReceivedCode(1);
  } else {
    // Here comes the Data in
    if ((timeStamp > 44) || (overFlow != 0)) {
      irBits = 32; // Invalid data, so try again
    }
    else {
      if (timeStamp > 26) {
        receivedData = receivedData | ((unsigned long) 1 << irBits);
      }
      if (irBits == 31) { 
        ReceivedCode(0);
      }
      irBits++;
    }
  }
  TCNT0 = 0;                    // Clear Counter0
  TIFR = TIFR | 1 << TOV0;      // Clear Overflow
  GIFR = GIFR | 1 << INTF0;     // Clear INT0 flag
}

// ISR 2 - called by timer1 (for intervalometer)
ISR(TIMER1_COMPA_vect) {
  if (newIntervalStep != oldIntervalStep) {
    OCR1C = newIntervalStep;
    OCR1A = OCR1C;
  }
  if (divider == 0) {
    singleFrame();
  }
  divider++;
  divider %= postscaler;  // modulo trick to allow intervals > 4.3 Sec
}

void loop() {
// Our loop() is empty, since everthing happens through Attiny timeStamprs/Counters and Interrupts :)
}

// Here come the various relay plays, simulating what a finger on the camera trigger would do to the built-in switches.
void blinkLED() {
  digitalWrite(ledPin, HIGH);
  _delay_ms(20);
  digitalWrite(ledPin, LOW);
}
void startRunWithMetering() {
// Regular start cycle as with the EA-1 wire
  digitalWrite(lightmeterPin, HIGH);
  digitalWrite(holdPin, HIGH);  
  digitalWrite(loadPin, HIGH);
  _delay_ms(250);      
  digitalWrite(loadPin, LOW);
  _delay_ms(70);      
  digitalWrite(startPin, HIGH);
}
void stopRun() {
  digitalWrite(startPin, LOW);
  _delay_ms(70);      
  digitalWrite(holdPin, LOW);  

  digitalWrite(lightmeterPin, LOW);
  digitalWrite(ledPin, HIGH);  
  _delay_ms(250);
  digitalWrite(ledPin, LOW);  
  lmMode = LM_MODE_1ST_SINGLESHOT;
}
void meterOnce() {
  digitalWrite(lightmeterPin, HIGH);
  digitalWrite(ledPin, HIGH);  
  _delay_ms(350);
  digitalWrite(ledPin, LOW);  
  digitalWrite(lightmeterPin, LOW);
  lmMode = LM_MODE_SUB_SINGLESHOT;
 }
void singleFrame() {
  digitalWrite(loadPin, HIGH);
  _delay_ms(10);                // 10ms is short but subsequent exposures will have a pre-loaded cap.
  digitalWrite(loadPin, LOW);
  digitalWrite(startPin, HIGH);
  _delay_ms(20);
  digitalWrite(startPin, LOW);
  digitalWrite(loadPin, HIGH);  // Pre-load the start capacitor since for the next exposure. Reduces latency. :)
  _delay_ms(50);        
  digitalWrite(loadPin, LOW);
}


