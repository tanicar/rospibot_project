
//---------------------------------------------------//
//                                                   //
//             Arduino Photoresistor reader          //
//             Deigned for ROSpiBot project          //
//                                                   //
//  Sketch by Andrea Fioroni (andrifiore@gmail.com)  //
//                                                   //
//                  GitHub repo:                     //
//  https://github.com/tanicar/rospibot_project.git  //
//                                                   //
//---------------------------------------------------//

// This sketch reads the analogic value from a photoresistor
// and communicates it to a raspberry through the serial port

#define photoPin A0   // photoresistor pin
#define LEDPin 9      // LED pin
#define spinDelay 200 // complessive loop delay
#define ledDelay 50   // LED blink delay

int photoSensor = 0;

void setup() {
  Serial.begin(9600); // initialise serial port (9600 baud)
  pinMode(photoPin, INPUT); // setupt photoresistor as input
  pinMode(LEDPin, OUTPUT);  // setupt LED pin as output
  analogWrite(LEDPin, 0);   // shutdown LED
}

void loop() {
  photoSensor = analogRead(photoPin); // read photoresistor
  Serial.println(photoSensor);  // write photoresistor value on the serial port

  blinkLED();
  
  delay(spinDelay - ledDelay);
}

void blinkLED() {
  analogWrite(LEDPin, 20);  // LED on
  delay(ledDelay);
  analogWrite(LEDPin, 0);   // LED off
}
