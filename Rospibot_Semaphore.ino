
//---------------------------------------------------//
//                                                   //
//               Simple Semaphore Sketch             //
//             Deigned for ROSpiBot project          //
//                                                   //
//  Sketch by Andrea Fioroni (andrifiore@gmail.com)  //
//                                                   //
//                  GitHub repo:                     //
//  https://github.com/tanicar/rospibot_project.git  //
//                                                   //
//---------------------------------------------------//

// LED pins
#define RED_PIN 13
#define GREEN_PIN 12
// delay values
#define RED_DELAY 10000     // red LED duration
#define GREEN_DELAY 4000    // green LED duration

void setup() {
  // setup LED pins as output
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  // turn off the LEDs
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
}

void loop() {
  // red LED on
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);
  delay(RED_DELAY);

  // green LED on
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  delay(GREEN_DELAY);
}
