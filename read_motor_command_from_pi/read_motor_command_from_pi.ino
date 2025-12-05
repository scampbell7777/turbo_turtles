#include <SparkFun_TB6612.h>

#define AIN1 34
#define AIN2 32
#define BIN1 38
#define BIN2 40
#define PWMA 4
#define PWMB 5
#define STBY 36

const int offsetA = 1;
const int offsetB = 1;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
String inputString = "";

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void parseCommand(String command) {
  int commaIndex = command.indexOf(',');
  if (commaIndex > 0) {
    String leftStr = command.substring(0, commaIndex);
    String rightStr = command.substring(commaIndex + 1);
    leftMotorSpeed = leftStr.toInt();
    rightMotorSpeed = rightStr.toInt();
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
}

void loop() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      parseCommand(inputString);
      inputString = "";
    } 
    else inputString += inChar;
  }
  // The pi reads these and uses them to make decisions.
  if (digitalRead(8) == LOW) Serial.println("1");
  if (digitalRead(9) == LOW) Serial.println("1");
  if (digitalRead(10) == LOW) Serial.println("2");
  motor1.drive(leftMotorSpeed,0);
  motor2.drive(rightMotorSpeed,0);
}
