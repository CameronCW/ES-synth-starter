#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <math.h>

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
  int noteA = 440;  //Base frequency
  int fSamp = 22000;
  float freq = 261.63;
  float step = (pow(2,32) * freq) / fSamp;
  float eqTemperament = pow(2,1/12);

  const uint32_t stepSizes [] = { 
    pow(2,32) * (noteA / ( pow(  eqTemperament , 2  ) ) ) / (fSamp),  //C
    pow(2,32) * (noteA / ( pow(  eqTemperament , 7  ) ) ) / (fSamp),  //C#
    pow(2,32) * (noteA / ( pow(  eqTemperament , 6  ) ) ) / (fSamp),  //D
    pow(2,32) * (noteA / ( pow(  eqTemperament , 5  ) ) ) / (fSamp),  //D#
    pow(2,32) * (noteA / ( pow(  eqTemperament , 4  ) ) ) / (fSamp),  //E
    pow(2,32) * (noteA / ( pow(  eqTemperament , 3  ) ) ) / (fSamp),  //F#
    pow(2,32) * (noteA / ( pow(  eqTemperament , 2  ) ) ) / (fSamp),  //G
    pow(2,32) * (noteA / ( pow(  eqTemperament , 1  ) ) ) / (fSamp),  //G#
    pow(2,32) * (noteA * ( pow(  eqTemperament , 0  ) ) ) / (fSamp),  //A
    pow(2,32) * (noteA * ( pow(  eqTemperament , 1  ) ) )/ (fSamp),   //A#
    pow(2,32) * (noteA * ( pow(  eqTemperament , 2  ) ) )/ (fSamp),   //B
  };    

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


std::string keyPressed(std::bitset<32> inputs) {
  for (int loopCount = 0; loopCount < 12; loopCount++) {
    if ((inputs & 0x000000000001) == 0) {
      return "C";
    } else if ((inputs & 0x000000000010) == 0) {
      return "C#";
    } else if ((inputs & 0x000000000100) == 0) {
      return "D";
    } else if ((inputs & 0x000000001000) == 0) {
      return "D#";
    } else if ((inputs & 0x000000010000) == 0) {
      return "E";
    } else if ((inputs & 0x000000100000) == 0) {
      return "F";
    } else if ((inputs & 0x000001000000) == 0) {
      return "F#";
    } else if ((inputs & 0x000010000000) == 0) {
      return "G";
    }else if ((inputs & 0x000100000000) == 0) {
      return "G#";
    }else if ((inputs & 0x001000000000) == 0) {
      return "A";
    }else if ((inputs & 0x01000000000) == 0) {
      return "A#";
    }else if ((inputs & 0x10000000000) == 0) {
      return "B";
    }
  }
  return "XXXX";  // Default no key is pressed
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  while (millis() < next);  //Wait for next interval

  next += interval;


  //Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory, cursor starts at 2,10
  // u8g2.print(count++);      //Iteration count
  
  std::bitset<32> inputs;
  // std::bitset<4> inputs = readCols();

  for(int loopCount = 0; loopCount < 3; loopCount++){
    setRow(loopCount);
    delayMicroseconds(3); //needed due to parasitic cap
    std::bitset<4> inputShort = readCols();
    // u8g2.setCursor(3,0);               //x, y: Pixel position for the cursor when printing Cursor 2 down, 20 from left 
    // u8g2.print(inputShort.to_ulong(),HEX);

    for (int i = 0; i < 4; ++i) {
        inputs[( (4*loopCount+1)-1 ) + i] = inputShort[i];
    }
    // currentStepSize = 
  }



  
  u8g2.setCursor(2,20);               //x, y: Pixel position for the cursor when printing Cursor 2 down, 20 from left 
  u8g2.print(inputs.to_ulong(),HEX);  //Print the output data in HEX encoding from the position the cursor is set to

  u8g2.sendBuffer();          // transfer internal memory to the display




  //Toggle LED
  digitalToggle(LED_BUILTIN);
  
}