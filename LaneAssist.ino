 /*
Code Name: Arduino Line Follower Robot Car
Code URI: https://circuitbest.com/category/arduino-projects/
Author: Make DIY
Author URI: https://circuitbest.com/author/admin/
Description: This program is used to make Arduino Line Follower Robot Car.
Note: You can use any value between 0 to 255 for M1_Speed, M2_Speed, LeftRotationSpeed, RightRotationSpeed.
Here 0 means Low Speed and 255 is for High Speed.
Version: 1.0
License: Remixing or Changing this Thing is allowed. Commercial use is not allowed.
*/

#define in1 12
#define in2 11
#define in3 7
#define in4 6
#define enA 10
#define enB 5
#define indicate 32



 int M1_Speed = 80; // speed of motor 1
 int M2_Speed = 80; // speed of motor 2
 int LeftRotationSpeed = 250;  // Left Rotation Speed
 int RightRotationSpeed = 250; // Right Rotation Speed


 void setup() {

  pinMode(idicate,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

    pinMode(enA,OUTPUT);
    pinMode(enB,OUTPUT);

      pinMode(A0, INPUT); // initialize Left sensor as an input
      pinMode(A1, INPUT); // initialize Right sensor as an input
                            digitalWrite(indicate, LOW);

    Serial.begin(9600);

}

void loop() {
  int LEFT_SENSOR = digitalRead(A0);
  int RIGHT_SENSOR = digitalRead(A1);
        // Serial.println(digitalRead(A0));
         //Serial.println(digitalRead(A1));
         //Serial.println(RIGHT_SENSOR);
         Serial.println(LEFT_SENSOR);


if(RIGHT_SENSOR==1 && LEFT_SENSOR==1) {
     //STOP
     Stop();
     
     
}

   if(RIGHT_SENSOR==1 && LEFT_SENSOR==0) {
     //Move Right
     left();
 }

   if(RIGHT_SENSOR==0 && LEFT_SENSOR==1) {
    right(); //Move Left
}

   if(RIGHT_SENSOR==0 && LEFT_SENSOR==0) {
             Serial.println("inside");
      //FORWARD
     
     forward();
 }
}



void left()
{
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);

                analogWrite(enA, M1_Speed);
                analogWrite(enB, M2_Speed);
                digitalWrite(indicate, HIGH);
}
void right()
{
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);

                analogWrite(enA, LeftRotationSpeed);
                analogWrite(enB, RightRotationSpeed);
                digitalWrite(indicate, HIGH);
}

void forward()
{
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);

                analogWrite(enA, M1_Speed);
                analogWrite(enB, M2_Speed);
                                digitalWrite(indicate, LOW);

}



void Stop()
{
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);
                            digitalWrite(indicate, LOW);

}
