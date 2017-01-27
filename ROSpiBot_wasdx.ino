
//---------------------------------------------------//
//                                                   //
//              Arduino Esplora Controller           //
//             Deigned for ROSpiBot project          //
//                                                   //
//  Sketch by Andrea Fioroni (andrifiore@gmail.com)  //
//                                                   //
//                  GitHub repo:                     //
//  https://github.com/tanicar/rospibot_project.git  //
//                                                   //
//---------------------------------------------------//

#include <Esplora.h>
#include <Keyboard.h>

int cutoff = 50;
int xMid = 6;
int yMid = -5;
int x, y;

int slideState;
int slideCutoff = 4;

boolean buttonMode = true;

void setup() {
  Serial.begin(9600);
  Keyboard.begin();
  slideState = 1024 - Esplora.readSlider();
}

void loop() {

  //------------------------- SPEED CONTROL ----------------------//
  /*
    if(1024-Esplora.readSlider() > slideState+slideCutoff) {
    Keyboard.write('i');
    Keyboard.write('\n');
    slideState = 1024-Esplora.readSlider();
    } else if(1024-Esplora.readSlider() < slideState-slideCutoff) {
    Keyboard.write('u');
    Keyboard.write('\n');
    slideState = 1024-Esplora.readSlider();
    }
  */
  //--------------------------------------------------------------//

  //-------------------------- MODE SWITCH -----------------------//
  if (Esplora.readJoystickButton() == LOW) {
    Keyboard.write('x');
    Keyboard.write('\n');
    buttonMode = !buttonMode;
  }
  //--------------------------------------------------------------//

  //-------------------------- MOTOR CONTROL ---------------------//
  if (buttonMode) {
    // BUTTONS - HOLDING SPEED
    if (Esplora.readButton(SWITCH_3) == LOW) {
      Keyboard.write('w');      // W = move forward
      Keyboard.write('\n');
    } else if (Esplora.readButton(SWITCH_1) == LOW) {
      Keyboard.write('s');      // S = move backward
      Keyboard.write('\n');
    } else if (Esplora.readButton(SWITCH_2) == LOW) {
      Keyboard.write('a');      // A = turn left
      Keyboard.write('\n');
    } else if (Esplora.readButton(SWITCH_4) == LOW) {
      Keyboard.write('d');      // D = turn right
      Keyboard.write('\n');
    }
  } else {
    // JOYSTICK - TOGGLE SPEED
    x = Esplora.readJoystickX();
    y = Esplora.readJoystickY();
    if (y < yMid - cutoff) {
      Keyboard.write('w');
      Keyboard.write('\n');
    } else if (y > yMid + cutoff) {
      Keyboard.write('s');
      Keyboard.write('\n');
    } else if (x > xMid + cutoff) {
      Keyboard.write('a');
      Keyboard.write('\n');
    } else if (x < xMid - cutoff) {
      Keyboard.write('d');
      Keyboard.write('\n');
    } else {
      Keyboard.write('x');
      Keyboard.write('\n');
    }
  }
  //--------------------------------------------------------------//

  delay(300);
}
/*




*/

