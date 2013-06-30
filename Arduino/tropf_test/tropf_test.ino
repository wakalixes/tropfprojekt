/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int pump1 = 2;
int pump2 = 3;
int pump3 = 4;

int timeint = 280;
int timepause = 500;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(pump1, OUTPUT);     
  pinMode(pump2, OUTPUT);     
  pinMode(pump3, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {
  for (timepause = 1000; timepause>=50; timepause-=50) {
    digitalWrite(pump1, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(pump2, HIGH);
    digitalWrite(pump3, HIGH);
    delay(timeint);               // wait for a second
    digitalWrite(pump1, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(pump2, LOW);
    digitalWrite(pump3, LOW);
    delay(timepause);
  }
  delay(2000);               // wait for a second
}
