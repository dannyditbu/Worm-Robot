#include <ax12.h>
#include <MuxShield.h>
#include <Serialprint.h>
#include <SimpleTimer.h>

//Written by Anh Nguyen
//controlling the actuators to read sensors

MuxShield muxShield; //a MuxShield object to read sensors

SimpleTimer t; //a SimpleTimer object to control actuators and read sensors at same time

int num_PINS = 2; //number of pins being considered in the board, there're 8 pins in total

int frsReading[2]; // a variable to store the readings from sensor;

// print the current array of analog values to the serial port
void printAnalog()
{
  int pin;
  
  for(pin = 0; pin < num_PINS; pin++)
  {
      Serial.print(frsReading[pin]);
      Serial.print("; "); // ; is used as a delimiter when reading the data with matlab to convert the character array from the serial port to numerical values.
  }
  Serial.println(' '); // Start on new line after printing the array
}

// Read and print each of the analog values from every port
void readAnalog()
{  
  int row = 1;
  int pin;
  int k=0; // k is used as the index into the 1-D array of data values
  
  for (pin=8; pin< 10; pin++){ 
    frsReading[k] = muxShield.analogReadMS(row,pin);// For analogREADMS, first argument is the row (1-3) the second argument is the pin (0-15).
//    Serial.print(frsReading[k]);
//    Serial.print("; ");
    k++; 
   }

//   Serial.println(' ');

  // Uncomment this section to use the onboard pins as well. Array lengths and num_Pins must be changed as well.
//  int i;
//  for(i = 0; i <= num_OnboardPins; i++)
//  {
//    frsReading[k] = analogRead(onBoard_Pin[i]);
//    k++;
//  }
  
}

//initialize actuators
void initializeActuators(void){
  const byte returnDelay = 0;
  Serialprintf("Initializing Dynamixel \n");
  dxlInit(1000000);
  dxlSetReturnDelayTime(1, returnDelay);
  dxlSetReturnDelayTime(7, returnDelay);
  delayMicroseconds(100);
  dxlSetStartupMaxTorque(1, MAX_TORQUE_VALUE); 
  dxlSetStartupMaxTorque(8, MAX_TORQUE_VALUE); 
  Serialprintf("Dynamixels Initialized \n");
}

/*keep increasing the speed of actuator in couterclockwise direction
   * 
   */
void graduallyStretchSensor(void){
   
  int actuatorSpeed = 150;
  while (actuatorSpeed < 1023){
    dxlSetGoalSpeed(1,actuatorSpeed);
    delay(1);
    dxlSetGoalSpeed(7,actuatorSpeed);
    delay(1000);
    dxlSetGoalSpeed(1,0);
    delay(1);
    dxlSetGoalSpeed(7,0);
    delay(1000);
    actuatorSpeed = actuatorSpeed + 50;
  }
}

/* bring the speed of actuator to maximum in couterclockwise direction and then increase it gradually in clockwise direction
 *  similar to stretching the sensor to it's maximum and gradually release it
 */
void graduallyReleaseSensor(void){
   
  word actuatorSpeed = 150;
  while (actuatorSpeed < 1023){
    dxlSetGoalSpeed(1, actuatorSpeed);
    delay(1);
    dxlSetGoalSpeed(7, actuatorSpeed);
    delay(1000);
    actuatorSpeed = actuatorSpeed + 500;
  }

  while (actuatorSpeed < 2047){
    dxlSetGoalSpeed(1, actuatorSpeed);
    delay(1);
    dxlSetGoalSpeed(7, actuatorSpeed);
    delay(1000);
    dxlSetGoalSpeed(1,0);
    delay(1);
    dxlSetGoalSpeed(7,0);
    delay(1000);
    actuatorSpeed = actuatorSpeed + 50;
  }
}

/* Stretch the sensor, hold, release to minimum, then stretch again
   *  bring the speed to maximum, hold it, decrease to a certain value, hold, and then increas again
   */
void stretchAndRelease(void){
    dxlSetGoalSpeed(1, 500);
    delay(1);
    dxlSetGoalSpeed(7, 500);
    delay(1000);
    dxlSetGoalSpeed(1, 1024);
    delay(1);
    dxlSetGoalSpeed(7,1024);
    delay(1000);
    dxlSetGoalSpeed(1, 1500);
    delay(1);
    dxlSetGoalSpeed(7,1500);
}  

void setup() {
  // put your setup code here, to run once:
  muxShield.setMode(1,ANALOG_IN);
  muxShield.setMode(2,ANALOG_IN);
  muxShield.setMode(3,ANALOG_IN);

  Serial.begin(115200);

  Serialprintf("Code Started \n");
  delay(200);
  initializeActuators();
  //t.setInterval(10, readAndPrintAnalog);
  t.setInterval(4000, stretchAndRelease);
}

void loop() {
  // put your main code here, to run repeatedly:
//  graduallyStretchSensor();
//  dxlSetGoalSpeed(1, 2047);
//  delay(1);
//  dxlSetGoalSpeed(7,2047);
  readAnalog();
  printAnalog();
  t.run();
}
