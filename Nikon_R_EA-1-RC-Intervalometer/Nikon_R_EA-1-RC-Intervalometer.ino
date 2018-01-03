/* ATtiny85 IR Remote Control Receiver
 * To Do:
 *  [ ] Consider lightmetering in interval modes (long ones maybe?)

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

// *** defines, constants and variables ***************************

const unsigned int   appleIrRcCode  = 0x87EE;  // Apple Remote (all models) 87EE
const byte  appleIrPlayKey          = 0x2F;    // Play (Alu RC)
const byte  appleIrOneFrameKey      = 0x2E;    // Center (Alu RC)
const byte  appleIrIntervalKey      = 0x01;    // Menu
const byte  appleIrFasterKey        = 0x05;    // Up
const byte  appleIrSlowerKey        = 0x06;    // Down
const byte  appleIrDoubleSpeedKey   = 0x04;    // Left
const byte  appleIrHalfSpeedKey     = 0x03;    // Right
const byte  appleIrWhitePlayKey     = 0x02;    // Play (old white Apple Remote)

volatile unsigned int  learnedIrRcCode          = 0;       // This is where the learned IR Codes go,
volatile unsigned int  learnedIrPlayKey         = 0;       // either loaded from EEPROM or freshly
volatile unsigned int  learnedIrOneFrameKey     = 0;       // learned.
volatile unsigned int  learnedIrIntervalKey     = 0;    
volatile unsigned int  learnedIrFasterKey       = 0;    
volatile unsigned int  learnedIrSlowerKey       = 0;    
volatile unsigned int  learnedIrDoubleSpeedKey  = 0;    
volatile unsigned int  learnedIrHalfSpeedKey    = 0;    


struct RC {           // This is used to store and retrieve learned IR codes to/from EEPROM
  int  RcCode;
  byte PlayKey;
  byte OneFrameKey;
  byte IntervalKey;
  byte FasterKey;
  byte SlowerKey;
  byte DoubleSpeedKey;
  byte HalfSpeedKey;
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
int  postscaler = 2;        // the postscaler doubles the timer1 by POT factors up to 2^16 (39:04:20)
volatile int divider = 0;   // needed in ISR to do some modulo for intervals > 4.3 sec

byte lmMode = LM_MODE_1ST_SINGLESHOT;
bool blinkFlag = true;      // This one is used to confirm reception fo valid IR codes
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

  RC IR;                              // Variable to store custom object reads from EEPROM
  EEPROM.get(0, IR);                  // Read from the beginning, there is nothing else in the EEPORM
  learnedIrRcCode         = IR.RcCode;
  learnedIrPlayKey        = IR.PlayKey;
  learnedIrOneFrameKey    = IR.OneFrameKey;
  learnedIrIntervalKey    = IR.IntervalKey;
  learnedIrFasterKey      = IR.FasterKey;
  learnedIrSlowerKey      = IR.SlowerKey;
  learnedIrDoubleSpeedKey = IR.DoubleSpeedKey;
  learnedIrHalfSpeedKey   = IR.HalfSpeedKey;

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
 
  digitalWrite(ledPin, HIGH); // Indicate that we are ready to learn a new Remte Control. 
  interrupts();
  startMillis = millis();     // Capture at what time we powered up. 
                              // (Note millis are 8x slower than normal here due to timer hacking)
}

void ReceivedCode(boolean Repeat) {
  int key;
  // Skip everything if we receive obvious garbage
  if ( ((receivedData>>16 & 0xFF) != 0xFF) && ((receivedData>>16 & 0xFF) != 0x00) ) {   
    if (justBooted && !learnMode) {
      learnedIrRcCode = (receivedData & 0xFFFF); // extract the RC's Address Code
      
      learnedIrPlayKey        = 0xFFFF;       // forget all pre-defined key codes to re-learn them
      learnedIrOneFrameKey    = 0xFFFF;       // we make the 0xFFFF to allow them being 0xFF.
      learnedIrIntervalKey    = 0xFFFF;       // use ALL the memory! :)
      learnedIrFasterKey      = 0xFFFF;
      learnedIrSlowerKey      = 0xFFFF;  
      learnedIrDoubleSpeedKey = 0xFFFF;    
      learnedIrHalfSpeedKey   = 0xFFFF;  
  
      blinkLEDtwice();
      learnMode = true;           // let's learn some keys. Next calls go into the following if-block.
  
    } else if (learnMode) {
      if (((receivedData & 0xFFFF) == learnedIrRcCode) && !Repeat) {
      
        blinkLEDtwice();
        key = receivedData>>16 & 0xFF;
             if (learnedIrPlayKey == 0xFFFF)        learnedIrPlayKey = key;
        else if (learnedIrOneFrameKey == 0xFFFF)    learnedIrOneFrameKey = key; 
        else if (learnedIrIntervalKey == 0xFFFF)    learnedIrIntervalKey = key; 
        else if (learnedIrFasterKey == 0xFFFF)      learnedIrFasterKey = key; 
        else if (learnedIrSlowerKey == 0xFFFF)      learnedIrSlowerKey = key; 
        else if (learnedIrDoubleSpeedKey == 0xFFFF) learnedIrDoubleSpeedKey = key; 
        else if (learnedIrHalfSpeedKey == 0xFFFF) {
          learnedIrHalfSpeedKey = key; 
          // store all keys to EEPROM here
          RC IR = {
            learnedIrRcCode,
            learnedIrPlayKey,
            learnedIrOneFrameKey,
            learnedIrIntervalKey,
            learnedIrFasterKey,
            learnedIrSlowerKey,
            learnedIrDoubleSpeedKey,
            learnedIrHalfSpeedKey
          };
          EEPROM.put(0, IR);
          
          blinkLEDtwice();     // This wild blinking confirms that all required keys
          blinkLEDtwice();     // are learned.
          blinkLEDtwice();
          blinkLEDtwice();
          blinkLEDtwice();
    
          learnMode = false;
        }
      }
      
      
    } else if (!justBooted && !learnMode) {     
      // This is if we are out of the Settings Mode right after Startup. Normal Operations.
  
      if (((receivedData & 0xFFFF) != learnedIrRcCode) && ((receivedData & 0xFFFF) != appleIrRcCode)) { // Check if Transmitter is unknown
        // This is for unknown IR Transmitters. 
        // Just does Run/Stop, executed by any key.
        
        key = receivedData>>16 & 0xFF;          // extracting the command byte, full 8 bits  
        if        ((key != 0xFF) && !Repeat && !digitalRead(lightmeterPin)) {
          blinkLED();
          startRunWithMetering();
        } else if ((key != 0xFF) && !Repeat &&  digitalRead(lightmeterPin)) {
          blinkLED();
          stopRun();
        }
    
      } else {                                  // This is comfort mode with a trained Remote or the Apple Remote. :)

        // This is known IR transmitters, either trained ones or an Apple Remote.
        // Does all the fancy functions.

        if ((receivedData & 0xFFFF) == appleIrRcCode) {
          key = receivedData>>17 & 0x7F;        // extracting the command byte, ignoring the 1-bit to match all Apple Remotes
        } else {
          key = receivedData>>16 & 0xFF;        // extract the command byte the normal way, no bit shifting and chopping.
        }
        
        // If we receive codes unique to an alu RC, let's remember that to fight ambiguity
        if (((key == appleIrOneFrameKey) && !Repeat && (receivedData & 0xFFFF) == appleIrRcCode) || 
            ((key == appleIrPlayKey)   && !Repeat && (receivedData & 0xFFFF) == appleIrRcCode)) aluRemote = true;
        
        // Now let's determine what to do -- as in match keys to actions.
        if      (((key == appleIrPlayKey)   || (key == learnedIrPlayKey))   && !Repeat && !digitalRead(lightmeterPin)) startRunWithMetering();
        else if (((key == appleIrPlayKey)   || (key == learnedIrPlayKey))   && !Repeat && digitalRead(lightmeterPin))  stopRun();
        else if (((key == appleIrOneFrameKey) || (key == learnedIrOneFrameKey)) && !Repeat) {
          if (lmMode == LM_MODE_1ST_SINGLESHOT) meterOnce();
          singleFrame();
        } else if (((key == appleIrIntervalKey) || (key == learnedIrIntervalKey)) && !Repeat) {         
          if (TIMSK & ( 1 << OCIE1A )) TIMSK &= ~(1 << OCIE1A);  // enable timer interrupt
          else                         TIMSK |= (1 << OCIE1A);   // disable timer interrupt
        } else if (((key == appleIrSlowerKey) || (key == learnedIrSlowerKey)) && !Repeat) {
          oldIntervalStep = newIntervalStep;
          newIntervalStep = constrain(oldIntervalStep + 10, 1, 255);
        } else if (((key == appleIrFasterKey)   || (key == learnedIrFasterKey)) && !Repeat) {
          oldIntervalStep = newIntervalStep;
          if (postscaler <= 4) {  // This could lead to intervals <200ms, which we do want to avoid
            newIntervalStep = constrain(oldIntervalStep - 10, 11, 255);
          } else {
            newIntervalStep = constrain(oldIntervalStep - 10, 1, 255);
          }
        } else if (((key == appleIrHalfSpeedKey) || (key == learnedIrHalfSpeedKey)) && !Repeat) postscaler = constrain(postscaler * 2, 1, 32768);
        else if   (((key == appleIrDoubleSpeedKey)  || (key == learnedIrDoubleSpeedKey))  && !Repeat) {
          postscaler = constrain(postscaler / 2, 1, 32768);
          if ((newIntervalStep < 11) && (postscaler <= 4)) newIntervalStep = 11; }
        else if ((key == appleIrWhitePlayKey) && !Repeat && !aluRemote && !digitalRead(lightmeterPin))   startRunWithMetering();
        else if ((key == appleIrWhitePlayKey) && !Repeat && !aluRemote &&  digitalRead(lightmeterPin))   stopRun();
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
    if ((startMillis + 250) < millis()) {
      justBooted = false;
      if (!learnMode) digitalWrite(ledPin, LOW);
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
  _delay_ms(10);                // 10ms might be short but subsequent exposures will have a pre-loaded cap.
  digitalWrite(loadPin, LOW);
  digitalWrite(startPin, HIGH);
  _delay_ms(10);
  digitalWrite(startPin, LOW);
  digitalWrite(loadPin, HIGH);  // Pre-load the start capacitor since for the next exposure. Reduces latency. :)
  _delay_ms(20);        
  digitalWrite(loadPin, LOW);
}


