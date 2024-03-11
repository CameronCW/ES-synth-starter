#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <math.h>
#include <STM32FreeRTOS.h>    //Install the 'STM32duino FreeRTOS' library with the Platformio library manager. Include its header file at the start of your source file:


//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

  //create frequency arrays
    // Middle C 261.63 Hz 
    //each step n_i = n_(i-1) = 2^(1/12)
  float fnoteA = 440;  //Base frequency
  float fSamp = 22000;
  float freq = 261.63;
  float step = (pow(2,32) * freq) / fSamp;
  float eqTemperament = pow(2.0,1.0/12.0);

  const uint32_t stepSizes [] = { 
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 9.0  ) ) ) / (fSamp),  //C
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 8.0  ) ) ) / (fSamp),  //C#
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 7.0  ) ) ) / (fSamp),  //D
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 6.0  ) ) ) / (fSamp),  //D#
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 5.0  ) ) ) / (fSamp),  //E
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 4.0  ) ) ) / (fSamp),  //F
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 3.0  ) ) ) / (fSamp),  //F#
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 2.0  ) ) ) / (fSamp),  //G
    // pow(2,32) * (fnoteA / ( pow(  eqTemperament , 1.0  ) ) ) / (fSamp),  //G#
    // pow(2,32) * (fnoteA * ( pow(  eqTemperament , 0.0  ) ) ) / (fSamp),   //A
    // pow(2,32) * (fnoteA * ( pow(  eqTemperament , 1.0  ) ) ) / (fSamp),   //A#
    // pow(2,32) * (fnoteA * ( pow(  eqTemperament , 2.0  ) ) ) / (fSamp)   //B
    static_cast<uint32_t>(pow(2,32) * (fnoteA / ( pow(  eqTemperament , 9.0  ) ) ) / (fSamp)),  //C
    static_cast<uint32_t>(pow(2,32) * (fnoteA / ( pow(  eqTemperament , 8.0  ) ) ) / (fSamp)),  //C#
    static_cast<uint32_t>(pow(2,32) * (fnoteA / ( pow(  eqTemperament , 7.0  ) ) ) / (fSamp)),  //D
    static_cast<uint32_t>(pow(2,32) * (fnoteA / ( pow(  eqTemperament , 6.0  ) ) ) / (fSamp)),  //D#
    static_cast<uint32_t>(pow(2,32) * (fnoteA / ( pow(  eqTemperament , 5.0  ) ) ) / (fSamp)),  //E
    static_cast<uint32_t>(pow(2,32) * (fnoteA / ( pow(  eqTemperament , 4.0  ) ) ) / (fSamp)),  //F
    static_cast<uint32_t>(pow(2,32) * (fnoteA / ( pow(  eqTemperament , 3.0  ) ) ) / (fSamp)),  //F#
    static_cast<uint32_t>(pow(2,32) * (fnoteA / ( pow(  eqTemperament , 2.0  ) ) ) / (fSamp)),  //G
    static_cast<uint32_t>(pow(2,32) * (fnoteA / (pow(   eqTemperament, 1.0   ) ) ) / (fSamp)),  //G#
    static_cast<uint32_t>(pow(2,32) * (fnoteA * ( pow(  eqTemperament , 0.0  ) ) ) / (fSamp)),  //A
    static_cast<uint32_t>(pow(2,32) * (fnoteA * ( pow(  eqTemperament , 1.0  ) ) ) / (fSamp)),   //A#
    static_cast<uint32_t>(pow(2,32) * (fnoteA * ( pow(  eqTemperament , 2.0  ) ) ) / (fSamp)),   //B
  };   
      //must be .0 to prevent integer division?

  volatile uint32_t currentStepSize;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}


struct {    //store system state that is used in more than one thread
  std::bitset<32> inputs;  // only contains the input bitset. The only other global variable is currentStepSize, 
          //but keep that apart from sysState because it is accessed by an ISR and the synchronisation method will be different
  SemaphoreHandle_t mutex;  //access to inputs is multiple cycles, therefore a mutex protects it from the multicycle threads accesses  
  int knob[4];              //1D array of width 4 for the knob rotation value
} sysState;

class Knob {    //TODO implement this
  private:
      int rotateVal;
      int limUpper;
      int limLower;
      SemaphoreHandle_t mutex;
  public:
      // Constructor
      Knob(int initialValue, int upperLimit, int lowerLimit) {    //In setup i.e. knob3, "Knob knob3(0,8,0)"  //for initial volume = 0, highest volume = 8, lowest = 0
          rotateVal = initialValue;
          limUpper = upperLimit;
          limLower = lowerLimit;
          mutex = xSemaphoreCreateMutex();
      }
      // Function to update the knob value
      void updateVal(int delta) { //delta to be +-1
          xSemaphoreTake(mutex, portMAX_DELAY);  // Lock the mutex (when out of scope it unlocks)
          //xSemaphoreGive(sysState.mutex);   //TODO setup knob.mutex
          rotateVal += delta;

          // Check and limit the knob value within the specified range
          if (rotateVal > limUpper) {
              rotateVal = limUpper;
          } else if (rotateVal < limLower) {
              rotateVal = limLower;
          }
          xSemaphoreGive(mutex);
      }

      // Function to read the current knob value
      int readVal() const {
          xSemaphoreTake(mutex, portMAX_DELAY);  // Lock the mutex
          int val = rotateVal;    //locking mutex requires one extra variable
          xSemaphoreGive(mutex);
          return val;
      }
};


void sampleISR() {
  static uint32_t phaseAcc = 0; //Static variable, local scope (static means it is stored between successive fn, stored for program lifetime)
                //But is shared between every instance (so if class, all instances share it)+
  uint32_t localCurrentStepSize;
  //__atomic_load_n(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);                
  //phaseAcc += localCurrentStepSize;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - sysState.knob[3]);    //knob[3] used for volumeControl, should zero based on shifts
      //Right-shift (divide by ) the phase accumlator and subtract , to scale the range to -2^7 <= Vout <= 2^7 -1:
  analogWrite(OUTR_PIN, Vout + 128);

  // In future, you will need to multiply and add signals, for example to implement a volume control or polyphony. That will be easier when samples have an offset of zero because the 
  //offset will be unaffected by mathematical operations. 
  //Meanwhile, the phase accumulator itself cannot have a zero offset because that would require a signed integer and the overflow of signed integers results in undefined behaviour in C and C++.
}

std::bitset<4> readCols(){
    //Allows for row 0 to be read, C C# D D#
    //C is LSB, D# MSB
  std::bitset<4> result;

  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

void setRow(uint8_t rowIdx){
  
  digitalWrite(REN_PIN,LOW);
  // for (byte i = 0; i<8; i++){ 
  //   
  digitalWrite(RA0_PIN, (0x01&rowIdx));
  digitalWrite(RA1_PIN, (0x02&rowIdx));
  digitalWrite(RA2_PIN, (0x04&rowIdx));
  digitalWrite(REN_PIN,HIGH);
}

std::string keyPressed() {
      //Currently only registers a single key pressed
      //Alternatively can pass no inputs and use sysState.inputs
  uint32_t localCurrentStepSize;
  std::string note = "x";
  xSemaphoreTake(sysState.mutex, portMAX_DELAY);  //Begin mutex lock
  for (int loopCount = 0; loopCount < 12; loopCount++) {
    if (sysState.inputs.test(0) ) {
      localCurrentStepSize = stepSizes[0];
      note = "C";
    } else if (sysState.inputs.test(1) ) {
      localCurrentStepSize = stepSizes[1];
      note = "C#";
    } else if (sysState.inputs.test(2) ) {
      localCurrentStepSize = stepSizes[2];
      note = "D";
    } else if (sysState.inputs.test(3) ) {
      localCurrentStepSize = stepSizes[3];
      note = "D#";
    } else if (sysState.inputs.test(4) ) {
      localCurrentStepSize = stepSizes[4];
      note = "E";
    } else if (sysState.inputs.test(5) ) {
      localCurrentStepSize = stepSizes[5];
      note = "F";
    } else if (sysState.inputs.test(6) ) {
      localCurrentStepSize = stepSizes[6];
      note = "F#";
    } else if (sysState.inputs.test(7) ) {
      localCurrentStepSize = stepSizes[7];
      note = "G";
    }else if (sysState.inputs.test(8) ) {
      localCurrentStepSize = stepSizes[8];
      note = "G#";
    }else if (sysState.inputs.test(9) ) {
      localCurrentStepSize = stepSizes[9];
      note = "A";
    }else if (sysState.inputs.test(10) ) {
      localCurrentStepSize = stepSizes[10];
      note = "A#";
    }else if (sysState.inputs.test(11) ) {
      localCurrentStepSize = stepSizes[11];
      note = "B";
    }else{
      //Default
      localCurrentStepSize = 0;    //No noise to play when not pressed
      note = "X";    // Default no key is pressed
    }
    xSemaphoreGive(sysState.mutex); //End Mutex lock
  }
  __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  return note;  
}

void scanKeysTask(void * pvParameters) {        //code for scanning the keyboard
    // Loop through the rows of the key matrix
    // Read the columns of the matrix and store the result in sysState.inputs
    // Look up the phase step size for the key that is pressed and update currentStepSize
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;    //now 20ms
  TickType_t xLastWakeTime = xTaskGetTickCount();
  std::bitset<2> knob3 = 0;
  std::bitset<2> previousKnob3 = 0;
  std::bitset<4> inputShort;
  const std::bitset<2> posTran = 0x1;
  const std::bitset<2> negTran = 0x3;
  const std::bitset<2>  noTran = 0x0;

  int knobCount[4]; //could use bitset<3> and .to_uint, but would need to type conversion both ways often, not worth it
  std::bitset<2> previousTransition = 0; //11 is negative, 01 positve, 00 no transition
  while (1) {   //Infinite loop - i.e independent thread
    //std::bitset<32> inputs; //superseded by sysState.inputs
    // std::bitset<4> inputs = readCols();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    for(int loopCount = 0; loopCount < 4; loopCount++){
      setRow(loopCount);
      delayMicroseconds(3); //needed due to parasitic cap

      inputShort = readCols();
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      for (int i = 0; i < 4; ++i) {
          sysState.inputs[( (4*loopCount+1)-1 ) + i] = ~ inputShort[i];  //bit inversion
      }
      xSemaphoreGive(sysState.mutex);
    } 
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);    //LOCK MUTEX
    knob3[0] = sysState.inputs[12];
    knob3[1] = sysState.inputs[13];

    for (int i = 0; i < 4; ++i) {     //copy the knobcount from sysState
      knobCount[i] = sysState.knob[i];
    }


    if (  ((~previousKnob3[1] & ~previousKnob3[0]) & (~knob3[1] &  knob3[0]) || 
          ( previousKnob3[1] &  previousKnob3[0]) & ( knob3[1] & ~knob3[0]) ||   //00 to 01 or 11 to 10 for +1
          ( (previousTransition == posTran) & (
             //!! used to convert int to bool and bitset to bool
          (!!((~previousKnob3[1] & ~previousKnob3[0]) & ( knob3[1] &  knob3[0])) ) ||     //00 to 11 impossible transition                              
          (!!((~previousKnob3[1] &  previousKnob3[0]) & ( knob3[1] &  knob3[0])) )||     //01 to 11 impossible transition
          (!!(( previousKnob3[1] & ~previousKnob3[0]) & (~knob3[1] &  knob3[0])) )  )))   //10 to 01 impossible tansition 
          & ( sysState.knob[3] < 8 )  ){  //8 max value for knob
          sysState.knob[3] ++;
          previousTransition = posTran;
    }else if
      ( ( (~previousKnob3[1] &  previousKnob3[0]) & (~knob3[1] & ~knob3[0]) || 
          ( previousKnob3[1] & ~previousKnob3[0]) & ( knob3[1] &  knob3[0]) ||    //01 to 00 or 10 to 11 for -1
          ( (previousTransition == negTran) &
            !!(( previousKnob3[1] &  previousKnob3[0]) & (~knob3[1] & ~knob3[0])) ) )      //11 to 00 impossible transition
          & ( sysState.knob[3] > 0 )   ){   
          sysState.knob[3] --;
          previousTransition = negTran;
    }else{  //otherwise ignore cases
          previousTransition = noTran;
    }   
    previousKnob3 = knob3;

    xSemaphoreGive(sysState.mutex);                 //UNLOCK MUTEX
  }
}


void displayUpdateTask(void * pvParameters){            //NEEDS WORK
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
      // put your main code here, to run repeatedly:
    // static uint32_t next = millis();
    // static uint32_t count = 0;

    // while (millis() < next);  //Wait for next interval

    // next += interval;

    vTaskDelayUntil( &xLastWakeTime, xFrequency );   //New delay to supersede the millis()<next

    // Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory, cursor starts at 2,10
    // u8g2.print(count++);      //Iteration count
    
    // currentStepSize handled in the keyPressed section
    std::string note = keyPressed();

    u8g2.setCursor(2,20);               //x, y: Pixel position for the cursor when printing Cursor 2 down, 20 from left 
    //u8g2.drawStr(2,20, note.c_str());  //Print the output data in HEX encoding from the position the cursor is set to

    u8g2.print(note.c_str());  //Print the output data in HEX encoding from the position the cursor is set to

    u8g2.setCursor(2, 20);
    u8g2.print("Step Sizes: ");
    u8g2.print(sysState.inputs.to_ulong(),HEX); 
    u8g2.sendBuffer();          // transfer internal memory to the display

    // Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}


void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //Setup hardware timer of 22KHZ to periodically call the ISR
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // initialise and run the independent thread
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */  //pvParameters input NULL for now
  2,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

    // initialise and run the independent thread
  TaskHandle_t displayUpdateTaskHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdateTask",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */  //pvParameters input NULL for now
  1,			/* Task priority */
  &displayUpdateTaskHandle );	/* Pointer to store the task handle */

  
  sysState.mutex = xSemaphoreCreateMutex();   //creates mutex and assigns handle

  vTaskStartScheduler();  //has to go at the end

}


void loop() { //Typically left empty
  // static uint32_t next = millis();
  // static uint32_t count = 0;

  // while (millis() < next);  //Wait for next interval

  // next += interval;


  // //Update display
  // u8g2.clearBuffer();         // clear the internal memory
  // u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  // u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory, cursor starts at 2,10
  // // u8g2.print(count++);      //Iteration count
  //   // std::bitset<4> inputs = readCols();

  // u8g2.setCursor(2,20);               //x, y: Pixel position for the cursor when printing Cursor 2 down, 20 from left 
  // //u8g2.print(inputs.to_ulong(),HEX);  //Print the output data in HEX encoding from the position the cursor is set to
  // u8g2.drawStr(2, 20, keyPressed().c_str());
  // //u8g2.print(note);  //Print the output data in HEX encoding from the position the cursor is set to

  // u8g2.sendBuffer();          // transfer internal memory to the display


  // //Toggle LED
  // digitalToggle(LED_BUILTIN);
}

