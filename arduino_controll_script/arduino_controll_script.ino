#include <SPI.h>
#include <Servo.h>


Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


void setup() {
  // setup serial 
  Serial.begin(9600);

  // setup the servo motors 
  servo0.attach(7);
  servo1.attach(6);
  servo2.attach(5);
  servo3.attach(4);
  servo4.attach(3);

  
}
void loop() {
    // receive a message string from the python controll script via serial 
    String str = receive(); 

    // extract the servo anlgles from the message string 
    int servo0_pos = str.substring(0, 3).toInt();
    int servo1_pos = str.substring(3, 6).toInt();
    int servo2_pos = str.substring(6, 9).toInt();
    int servo3_pos = 180 - str.substring(9, 12).toInt();
    int servo4_pos = 180 - str.substring(12, 15).toInt();
  
    // drive the servo motors 
    servo0.write(servo0_pos);
    servo1.write(servo1_pos);
    servo2.write(servo2_pos);
    servo3.write(servo3_pos);
    servo4.write(servo4_pos);

    // print the message string to serial for debugging
    Serial.print(str);
}

/*
 *  This function receives a string over serial from the python contoll script
 */
String receive() {

  String str = "";

  // wait for the start of a message string
  char c = '0';
  while (c != '<') {
    while (Serial.available() <= 0) { /* busy wait*/ }
    c = Serial.read();  
  }

  // start receiving 
  bool receiving = true; 
  
  while (receiving) {
      while (Serial.available() <= 0) { /* busy wait*/ }
    
      c = Serial.read(); 

      // stop receiving on end of the message string 
      if (c == '>') {
        receiving = false; 
      }
      else {
        str += c; 
      }
  }
  
  return str;  
}
