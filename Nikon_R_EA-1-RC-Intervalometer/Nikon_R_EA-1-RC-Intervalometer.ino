/* ATtiny85 IR Remote Control Receiver
 * To Do:
 *  [ ] Consider lightmetering in interval modes (long ones maybe?)
 *  [ ] do we need repeat?
 *  [ ] use the struct for IR codes only, not vars *and* a struct
 *  [ ] Optimize learning of erratic codes
 *  [ ] Shorten the single shot times
 *  [ ] cleanup variable names for IR codes (should be functions, not key names. Doh.)

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
#include <EEPROM.h>     // We use the EEPROM to store learned IR codes.

// *** defines and variables ***************************

volatile unsigned int  irRcCode           = 0x87EE;  // Apple Remote (all models) 87EE
volatile unsigned int  irPlayKey          = 0x2F;    // Play (Alu RC)
volatile unsigned int  irCenterKey        = 0x2E;    // Center (Alu RC)
volatile unsigned int  irMenuKey          = 0x01;    // Menu
volatile unsigned int  irUpKey            = 0x05;    // Up
volatile unsigned int  irDownKey          = 0x06;    // Down
volatile unsigned int  irLeftKey          = 0x04;    // Left
volatile unsigned int  irRightKey         = 0x03;    // Right

volatile unsigned int  irWhitePlayKey     = 0x02;    // Play (old white Apple Remote)

struct RC {           // This is used to store and retrieve learned IR codes to/from EEPROM
  int  RcCode;
  byte PlayKey;
  byte CenterKey;
  byte MenuKey;
  byte UpKey;
  byte DownKey;
  byte LeftKey;
  byte RightKey;
};


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
int  postscaler = 2;         // the postscaler doubles the timer1 by POT factors up to 2^16 (39:04:20)
volatile int divider = 0;   // needed in ISR to do some modulo for intervals > 4.3 sec

byte lmMode = LM_MODE_1ST_SINGLESHOT;
bool blinkFlag = true;       // This one is used to confirm reception fo valid IR codes
bool aluRemote = false;     // Apple's Alumium Remote has one more key than the white one. Reading
                            // that key is ambigious, so we "lock in" to a safe mode once we know for sure
                            // that our RC is not the white one.

unsigned long startMillis = 0;
volatile bool justBooted = true;     // Flag to support the Settings Mode available right after power-up
volatile bool learnMode  = false;    // This is true if we entered IR learning mode. IR commands cause no camera action then.

// *** Setup **********************************************
void setup() {
  pinMode(lightmeterPin, OUTPUT);     
  pinMode(holdPin, OUTPUT);     
  pinMode(loadPin, OUTPUT);     
  pinMode(startPin, OUTPUT);     

  RC IR;                  // Variable to store custom object reads from EEPROM
  EEPROM.get(0, IR);      // Read from the beginning, there is nothing else in the EEPORM
  irRcCode    = IR.RcCode;
  irPlayKey   = IR.PlayKey;
  irCenterKey = IR.CenterKey;
  irMenuKey   = IR.MenuKey;
  irUpKey     = IR.UpKey;
  irDownKey   = IR.DownKey;
  irLeftKey   = IR.LeftKey;
  irRightKey  = IR.RightKey;

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
  startMillis = millis();     // Capture at what time we powered up. 
                              // (Note millis are 8x slower than normal here due to timer hacking)
}

void ReceivedCode(boolean Repeat) {
  int key;
  // Skip everything if we receive obvious garbage
  if ( ((receivedData>>16 & 0xFF) != 0xFF) || ((receivedData>>16 & 0xFF) != 0x00) ) { 
    if (justBooted && !learnMode) {
      irRcCode = (receivedData & 0xFFFF); // extract the RC's Address Code
      
      irPlayKey   = 0xFFFF;       // forget all pre-defined key codes to re-learn them
      irCenterKey = 0xFFFF;  
      irMenuKey   = 0xFFFF;  
      irUpKey     = 0xFFFF;
      irDownKey   = 0xFFFF;  
      irLeftKey   = 0xFFFF;    
      irRightKey  = 0xFFFF;  
  
      blinkLEDtwice();
      learnMode = true;           // let's learn some keys. Next calls go into the following if-block.
  
    } else if (learnMode) {
      if ((receivedData & 0xFFFF) == irRcCode) {
      
        blinkLEDtwice();
        key = receivedData>>16 & 0xFF;
             if (irPlayKey == 0xFFFF)   irPlayKey = key;
        else if (irCenterKey == 0xFFFF) irCenterKey = key; 
        else if (irMenuKey == 0xFFFF)   irMenuKey = key; 
        else if (irUpKey == 0xFFFF)     irUpKey = key; 
        else if (irDownKey == 0xFFFF)   irDownKey = key; 
        else if (irLeftKey == 0xFFFF)   irLeftKey = key; 
        else if (irRightKey == 0xFFFF)  {
          irRightKey = key; 
          // store all keys to EEPROM here
          RC IR = {
            irRcCode,
            irPlayKey,
            irCenterKey,
            irMenuKey,
            irUpKey,
            irDownKey,
            irLeftKey,
            irRightKey
          };
          EEPROM.put(0, IR);
          
          blinkLEDtwice();
          blinkLEDtwice();
          blinkLEDtwice();
          blinkLEDtwice();
    
          learnMode = false;
        }
      }
      
      
    } else if (!justBooted && !learnMode) {     
      // This is if we are out of the Settings Mode right after Startup. Normal Operations.
  
      if ((receivedData & 0xFFFF) != irRcCode) {// Check if Transmitter is unknown
        // This is for unknown IR Transmitters. 
        // Just does Run/Stop, executed by any key.
        // 
        if (!Repeat) blinkLED();
        key = receivedData>>16 & 0xFF;          // extracting the command byte, full 8 bits  
        if      ((key != 0xFF) && !Repeat && !digitalRead(lightmeterPin))  startRunWithMetering();
        else if ((key != 0xFF) && !Repeat &&  digitalRead(lightmeterPin))  stopRun();
    
      } else {                                  // This is comfort mode with a trained Remote or the Apple Remote. :)
        // This is known IR transmitters, either trained ones or an Apple Remote.
        // Does all the fancy functions.
        //
        if ((receivedData & 0xFFFF) == 0x87EE) {
          key = receivedData>>17 & 0x7F;        // extracting the command byte, ignoring the 1-bit to match all Apple Remotes
        } else {
          key = receivedData>>16 & 0xFF;        // extract the command byte the normal way, no bit shifting and chopping.
        }
        
        // If we receive codes unique to an alu RC, let's remember that to fight ambiguity
        if (((key == irCenterKey) && !Repeat && irRcCode == 0x87EE) || 
            ((key == irPlayKey)   && !Repeat && irRcCode == 0x87EE)) aluRemote = true;
        
        // Now let's determine what to do
        if      ((key == irPlayKey)   && !Repeat && !digitalRead(lightmeterPin)) startRunWithMetering();
        else if ((key == irPlayKey)   && !Repeat && digitalRead(lightmeterPin))  stopRun();
        else if ((key == irCenterKey) && !Repeat) {
          if (lmMode == LM_MODE_1ST_SINGLESHOT) meterOnce();
          singleFrame();
        } else if ((key == irMenuKey) && !Repeat) { //irMenuKey
          if (TIMSK & ( 1 << OCIE1A )) TIMSK &= ~(1 << OCIE1A);  // enable timer interrupt
          else                         TIMSK |= (1 << OCIE1A);   // disable timer interrupt
        } else if ((key == irDownKey) && !Repeat) {
          oldIntervalStep = newIntervalStep;
          newIntervalStep = constrain(oldIntervalStep + 10, 1, 255);
        } else if ((key == irUpKey) && !Repeat) {
          oldIntervalStep = newIntervalStep;
          if (postscaler <= 4) {  // This could lead to intervals <200ms, which we do want to avoid
            newIntervalStep = constrain(oldIntervalStep - 10, 11, 255);
          } else {
            newIntervalStep = constrain(oldIntervalStep - 10, 1, 255);
          }
        } else if ((key == irRightKey) && !Repeat) postscaler = constrain(postscaler * 2, 1, 32768);
        else if ((key == irLeftKey) && !Repeat) {
          postscaler = constrain(postscaler / 2, 1, 32768);
          if ((newIntervalStep < 11) && (postscaler <= 4)) newIntervalStep = 11; }
        else if ((key == irWhitePlayKey) && !Repeat && !aluRemote && !digitalRead(lightmeterPin))   startRunWithMetering();
        else if ((key == irWhitePlayKey) && !Repeat && !aluRemote &&  digitalRead(lightmeterPin))   stopRun();
        else blinkFlag = false;   // if we received a partial or garbled IR code, let's not confirm reception
      }
      if (blinkFlag) blinkLED();
      blinkFlag = true;   // Let's assume the next code is a valid one
    }
  }
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
/*  Our loop() is almost empty, since everthing happens through Attiny timeStamprs/Counters and Interrupts :)
 *  What happens here is supporting the Settings Mode, which is enabled by receiving an IR signal in the 
 *  first n ms after powerup.
 */
  if (justBooted) {
    if ((startMillis + 100) < millis()) {
      justBooted = false;
      if (!learnMode) blinkLEDtwice();
    }
  }
}

// Here come the various relay plays, simulating what a finger on the camera trigger would do to the built-in switches.
void blinkLED() {
  digitalWrite(ledPin, HIGH);
  _delay_ms(20);
  digitalWrite(ledPin, LOW);
}
void blinkLEDtwice() {
  digitalWrite(ledPin, HIGH);
  _delay_ms(50);
  digitalWrite(ledPin, LOW);
  _delay_ms(150);
  digitalWrite(ledPin, HIGH);
  _delay_ms(50);
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


