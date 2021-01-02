#include <WiFi.h>
#include <FirebaseArduino.h>
#include <String.h>

void setup() {
  // put your setup code here, to run once:

  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  while(1){

    digitalWrite(10, HIGH);
    delay(1000);
  }
}
