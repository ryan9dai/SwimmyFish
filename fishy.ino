#include <Servo.h>
Servo myservo;
int pos = 0;
int direction = 1;
int val;

void setup() {
  myservo.attach(5);  
  pinMode(4, INPUT);

}

void loop() {
  val = digitalRead(4);
  if (val == HIGH)
  {
    if (pos==0)
    {
      direction=1;
  
    }
    if (pos==90)
    {
      direction=-1;
    }
    pos+=direction; 
    myservo.write(pos);
    delay(2);
  }
}
