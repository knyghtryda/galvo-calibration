#include <mult16x16.h>
#include <mult16x8.h>
#include <mult32x16.h>

#include <SPI.h>
#include <fastio.h>
#include <EEPROM.h>
#include "galvo.h"

#define TRIG			7//13
#define	SCK			13//52
#define	MISO			12//50
#define	MOSI			11//51
#define	SS			10//53
#define GALVO_SS		10//61
#define LASER		32
#define Z_ENABLE       48

#define X_AXIS 0
#define Y_AXIS 1

#define STEPS 8
#define GRID_STEPS 10
#define PRINTER_HEIGHT  143.0
#define PRINTER_WIDTH   140.0
#define E_DISTANCE      7.0
#define X_SCALE 0.6
#define Y_SCALE 0.6

#define EEPROM_OFFSET 0x0000
unsigned volatile int x_index = 0;
unsigned volatile int y_index = 0;

Galvo galvo(PRINTER_WIDTH, PRINTER_WIDTH, PRINTER_HEIGHT, E_DISTANCE, X_SCALE, Y_SCALE);

unsigned volatile int counter[2] = {};

void setup() {
  // put your setup code here, to run once:
  SET_OUTPUT(MOSI);
  SET_OUTPUT(SCK);
  SET_OUTPUT(GALVO_SS);
  SET_OUTPUT(TRIG);
  //SET_OUTPUT(LASER);
  //SET_OUTPUT(Z_ENABLE);
  //WRITE(Z_ENABLE, LOW);
  //WRITE(LASER, HIGH);
  WRITE(GALVO_SS, HIGH);

  //setup galvos
  SET_OUTPUT(GALVO_SS);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  Serial.begin(115200);
  //galvo.printValues();
  counter[X_AXIS] = galvo.getMin(X_AXIS);
  counter[Y_AXIS] = galvo.getMin(Y_AXIS);
  unsigned long elapsed;
  //elapsed = micros();
  galvo.CalcCalibrationTable();
  //elapsed = micros() - elapsed;
  //galvo.printCalTable();
  //Serial.print(elapsed);
  //Serial.println(" us");
  galvo.CalcSlopeTable();
  /*
  Serial.println("Saving Calibration Table in 5 seconds");
  delay(5000);
  elapsed = micros();
  save_calibration_offsets();
  elapsed = micros() - elapsed;
  Serial.print(elapsed);
  Serial.println(" us");
  */
  /*
  unsigned int test[2] = {galvo.getMax(X_AXIS), galvo.getMax(Y_AXIS)};
  Serial.println("ApplyOffset Test");
  Serial.print("Original Coordinate = ");
  galvo.printPair(test);
  elapsed = micros();
  galvo.ApplyOffsets(test);
  elapsed = micros() - elapsed;
  Serial.print("\nOffset Coordinate = ");
  galvo.printPair(test);
  Serial.print("\ntime = ");
  Serial.print(elapsed);
  Serial.println(" us");
  test[X_AXIS] = galvo.getMax(X_AXIS);
  test[Y_AXIS] = galvo.getMax(Y_AXIS);
  Serial.println("ApplySlopeOffset Test");
  Serial.print("Original Coordinate = ");
  galvo.printPair(test);
  elapsed = micros();
  galvo.ApplySlopeOffsets(test);
  elapsed = micros() - elapsed;
  Serial.print("\nOffset Coordinate = ");
  galvo.printPair(test);
  Serial.print("\ntime = ");
  Serial.print(elapsed);
  Serial.println(" us");
  */
  //Serial.println(get_offset(g_min[X_AXIS], g_min[Y_AXIS], X_AXIS));
  //Serial.println(get_offset(g_max[X_AXIS], g_min[Y_AXIS], X_AXIS));
  
  cli();
  // waveform generation = 0100 = CTC
  TCCR1B &= ~MASK(WGM13);
  TCCR1B |= MASK(WGM12);
  TCCR1A &= ~MASK(WGM11);
  TCCR1A &= ~MASK(WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  OCR1A = 0xF00; //0xA5 == ~12kHz
				 //TCNT1 = 0;
  TIMSK1 |= MASK(OCIE1A);
  sei(); // enable global interrupts
  
}

inline volatile void transfer(unsigned char byte) {
	while (!(SPSR & _BV(SPIF)));
	SPDR = byte;
}

inline volatile void transfer16(unsigned int uint) {
	transfer(uint >> 8);
	transfer(uint);
}

inline volatile void transferDAC(unsigned int * uint) {
	WRITE(GALVO_SS, LOW);
	transfer(X_AXIS | (3 << 4));
	transfer16(uint[X_AXIS]);
	WRITE(GALVO_SS, HIGH);
	WRITE(GALVO_SS, LOW);
	transfer(Y_AXIS | (3 << 4));
	transfer16(uint[Y_AXIS]);
	WRITE(GALVO_SS, HIGH);
}

ISR(TIMER1_COMPA_vect)
{
  WRITE(TRIG, HIGH);
  unsigned int tmp[2];
  tmp[X_AXIS] = counter[X_AXIS];
  tmp[Y_AXIS] = counter[Y_AXIS];
  //galvo.ApplyOffsets(tmp);
  galvo.ApplySlopeOffsets(tmp);
  transferDAC(tmp);
  //WRITE(LASER, HIGH);
  counter[X_AXIS] = counter[X_AXIS] + galvo.getStepSize(X_AXIS);
  x_index++;
  if (x_index > galvo.getSteps()) {
    x_index = 0;
    counter[X_AXIS] = galvo.getMin(X_AXIS);
    counter[Y_AXIS] = counter[Y_AXIS] + galvo.getStepSize(Y_AXIS);
    y_index++;
    if (y_index > galvo.getSteps()) {
      y_index = 0;
      counter[Y_AXIS] = galvo.getMin(Y_AXIS);
    }
  }
  //WRITE(LASER, LOW);
  WRITE(TRIG, LOW);
}

void save_calibration_offsets() {
  unsigned int address = EEPROM_OFFSET;
  unsigned int table_size = sizeof(galvo.offsets);
  unsigned int eeprom_size = EEPROM.length();
  unsigned char data_length = sizeof(galvo.offsets[0][0]);
  int tmp[2] = {};
  if(table_size > eeprom_size) {
    Serial.println("ERROR!  EEPROM too small to fit table!");
    Serial.print("EEPROM Size = ");
    Serial.print(eeprom_size);
    Serial.println(" bytes");
    Serial.print("Table Size = ");
    Serial.print(table_size);
    Serial.println(" bytes");
    return;
  }
  Serial.println("Writing Calibration Table to EEPROM");
  for(int j = 0; j < galvo.getPoints(); j++) {
    for(int i = 0; i < galvo.getPoints(); i++) {
      EEPROM.put(address, galvo.offsets[i][j]);
      EEPROM.get(address, tmp);
      if(galvo.offsets[i][j][X_AXIS] != tmp[X_AXIS] || galvo.offsets[i][j][Y_AXIS] != tmp[Y_AXIS]) {
        Serial.println("ERROR! Data Verification failed!");
        Serial.print("Expected x = ");
        Serial.print(galvo.offsets[i][j][X_AXIS]);
        Serial.print(" Written x = ");
        Serial.println(tmp[X_AXIS]);
        Serial.print("Expected y = ");
        Serial.print(galvo.offsets[i][j][Y_AXIS]);
        Serial.print(" Written x = ");
        Serial.println(tmp[Y_AXIS]);
        return;
      }
      address += data_length;
    }
   }
  Serial.println("Calibration Table successfully saved!"); 
}


void loop() {
  // put your main code here, to run repeatedly:
}
