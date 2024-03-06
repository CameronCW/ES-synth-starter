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
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 9.0  ) ) ) / (fSamp),  //C
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 8.0  ) ) ) / (fSamp),  //C#
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 7.0  ) ) ) / (fSamp),  //D
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 6.0  ) ) ) / (fSamp),  //D#
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 5.0  ) ) ) / (fSamp),  //E
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 4.0  ) ) ) / (fSamp),  //F
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 3.0  ) ) ) / (fSamp),  //F#
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 2.0  ) ) ) / (fSamp),  //G
    pow(2,32) * (fnoteA / ( pow(  eqTemperament , 1.0  ) ) ) / (fSamp),  //G#
    pow(2,32) * (fnoteA * ( pow(  eqTemperament , 0.0  ) ) ) / (fSamp),   //A
    pow(2,32) * (fnoteA * ( pow(  eqTemperament , 1.0  ) ) ) / (fSamp),   //A#
    pow(2,32) * (fnoteA * ( pow(  eqTemperament , 2.0  ) ) ) / (fSamp)   //B
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

void sampleISR() {
  static uint32_t phaseAcc = 0; //Static variable, local scope (static means it is stored between successive fn, stored for program lifetime)
                //But is shared between every instance (so if class, all instances share it)+
  uint32_t localCurrentStepSize;
  //__atomic_load_n(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);                
  //phaseAcc += localCurrentStepSize;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
      //Right-shift (divide by ) the phase accumlator and subtract , to scale the range to -2^7 <= Vout <= 2^7 -1:
  analogWrite(OUTR_PIN, Vout + 128);
  // In future, you will need to multiply and add signals, for example to implement a volume control or polyphony. That will be easier when samples have an offset of zero because the 
  //offset will be unaffected by mathematical operations. 
  //Meanwhile, the phase accumulator itself cannot have a zero offset because that would require a signed integer and the overflow of signed integers results in undefined behaviour in C and C++.
}

struct {    //store system state that is used in more than one thread
  std::bitset<32> inputs;  // only contains the input bitset. The only other global variable is currentStepSize, 
          //but keep that apart from sysState because it is accessed by an ISR and the synchronisation method will be different
} sysState;

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

std::string keyPressed(std::bitset<32> inputs) {
      //Currently only registers a single key pressed
  uint32_t localCurrentStepSize;
  std::string note = "x";
  for (int loopCount = 0; loopCount < 12; loopCount++) {
    if (inputs.test(0) ) {
      localCurrentStepSize = stepSizes[0];
      note = "C";
    } else if (inputs.test(1) ) {
      localCurrentStepSize = stepSizes[1];
      note = "C#";
    } else if (inputs.test(2) ) {
      localCurrentStepSize = stepSizes[2];
      note = "D";
    } else if (inputs.test(3) ) {
      localCurrentStepSize = stepSizes[3];
      note = "D#";
    } else if (inputs.test(4) ) {
      localCurrentStepSize = stepSizes[4];
      note = "E";
    } else if (inputs.test(5) ) {
      localCurrentStepSize = stepSizes[5];
      note = "F";
    } else if (inputs.test(6) ) {
      localCurrentStepSize = stepSizes[6];
      note = "F#";
    } else if (inputs.test(7) ) {
      localCurrentStepSize = stepSizes[7];
      note = "G";
    }else if (inputs.test(8) ) {
      localCurrentStepSize = stepSizes[8];
      note = "G#";
    }else if (inputs.test(9) ) {
      localCurrentStepSize = stepSizes[9];
      note = "A";
    }else if (inputs.test(10) ) {
      localCurrentStepSize = stepSizes[10];
      note = "A#";
    }else if (inputs.test(11) ) {
      localCurrentStepSize = stepSizes[11];
      note = "B";
    }else{
      //Default
      localCurrentStepSize = 0;    //No noise to play when not pressed
      note = "X";    // Default no key is pressed
    }
  }
  __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  return note;  
}

void scanKeysTask(void * pvParameters) {        //code for scanning the keyboard
    // Loop through the rows of the key matrix
    // Read the columns of the matrix and store the result in sysState.inputs
    // Look up the phase step size for the key that is pressed and update currentStepSize
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {   //Infinite loop - i.e independent thread
      //std::bitset<32> inputs; //superseded by sysState.inputs
      // std::bitset<4> inputs = readCols();
      vTaskDelayUntil( &xLastWakeTime, xFrequency );

      for(int loopCount = 0; loopCount < 3; loopCount++){
        setRow(loopCount);
        delayMicroseconds(3); //needed due to parasitic cap
        std::bitset<4> inputShort = readCols();
        // u8g2.setCursor(3,0);               //x, y: Pixel position for the cursor when printing Cursor 2 down, 20 from left 
        // u8g2.print(inputShort.to_ulong(),HEX);

        for (int i = 0; i < 4; ++i) {
            sysState.inputs[( (4*loopCount+1)-1 ) + i] = ~ inputShort[i];  //bit inversion
        }
      } 
  }
}


void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
      // put your main code here, to run repeatedly:
    // static uint32_t next = millis();
    static uint32_t count = 0;

    // while (millis() < next);  //Wait for next interval

    // next += interval;

    vTaskDelayUntil( &xLastWakeTime, xFrequency );   //New delay to supersede the millis()<next

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory, cursor starts at 2,10
    // u8g2.print(count++);      //Iteration count
    
    // currentStepSize handled in the keyPressed section
    std::string note = keyPressed(sysState.inputs);

    u8g2.setCursor(2,20);               //x, y: Pixel position for the cursor when printing Cursor 2 down, 20 from left 
    //u8g2.print(inputs.to_ulong(),HEX);  //Print the output data in HEX encoding from the position the cursor is set to
    //u8g2.print(keyPressed(inputs).to_ulong(), HEX);  //Print the output data in HEX encoding from the position the cursor is set to
    u8g2.print(note.c_str());  //Print the output data in HEX encoding from the position the cursor is set to
    // u8g2.setCursor(2, 20);
    // u8g2.print("Step Sizes: ");
    // u8g2.print(currentStepSize); 
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
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
  "displayUpdate",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */  //pvParameters input NULL for now
  1,			/* Task priority */
  &displayUpdateTaskHandle );	/* Pointer to store the task handle */

  

  vTaskStartScheduler();  //has to go at the end

}


void loop() { //Typically left empty
  
}

