
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
#include <SPI.h>
#include <TFT.h>
#include <Keyboard.h>

int cutoff = 50;
int xMid = 6;
int yMid = -5;
int x, y;
int lastInput; // 0 = x, 1 = w, 2 = a, 3 = s, 4 = d

int slideState;
int slideCutoff = 4;

boolean buttonMode = true;
int switched = 0;
int lastDir = -1;

void setup() {
  Serial.begin(9600);
  Keyboard.begin();

  lastInput = 0; // x
  slideState = 1024 - Esplora.readSlider();

  Esplora.writeRGB(200, 200, 200);
  EsploraTFT.begin();
  EsploraTFT.background(0, 0, 0);
  EsploraTFT.stroke(0, 250, 200);
  EsploraTFT.text("Sketch by Andrea Fioroni", 10, 115);
  EsploraTFT.setTextSize(2);
  EsploraTFT.text("ROSpiBot", 30, 10);
  EsploraTFT.stroke(200, 200, 0);
  EsploraTFT.text("Mode: ", 14, 32);
  EsploraTFT.stroke(0, 200, 0);
  EsploraTFT.text("Auto", 74, 32);
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
    TFTdraw(-1);
    buttonMode = !buttonMode;
    switched = 1;
  }
  //--------------------------------------------------------------//

  //-------------------------- MOTOR CONTROL ---------------------//
  if (buttonMode) {

    if (switched == 1) {
      EsploraTFT.stroke(0, 0, 0);
      EsploraTFT.fill(0, 0, 0);
      EsploraTFT.rect(12, 30, 150, 20);
      switched = 0;
      EsploraTFT.stroke(200, 200, 0);
      EsploraTFT.text("Mode: ", 14, 32);
      EsploraTFT.stroke(0, 200, 0);
      EsploraTFT.text("Auto", 74, 32);
    }

    // BUTTONS - HOLDING SPEED
    if (Esplora.readButton(SWITCH_3) == LOW) {
      Keyboard.write('w');      // W = move forward
      Keyboard.write('\n');
      TFTdraw(0);
    } else if (Esplora.readButton(SWITCH_1) == LOW) {
      Keyboard.write('s');      // S = move backward
      Keyboard.write('\n');
      TFTdraw(1);
    } else if (Esplora.readButton(SWITCH_2) == LOW) {
      Keyboard.write('a');      // A = turn left
      Keyboard.write('\n');
      TFTdraw(2);
    } else if (Esplora.readButton(SWITCH_4) == LOW) {
      Keyboard.write('d');      // D = turn right
      Keyboard.write('\n');
      TFTdraw(3);
    }
  } else {

    if (switched == 1) {
      EsploraTFT.stroke(0, 0, 0);
      EsploraTFT.fill(0, 0, 0);
      EsploraTFT.rect(12, 30, 150, 20);
      switched = 0;
      EsploraTFT.stroke(200, 200, 0);
      EsploraTFT.text("Mode: ", 14, 32);
      EsploraTFT.stroke(200, 0, 0);
      EsploraTFT.text("Manual", 74, 32);
    }

    // JOYSTICK - TOGGLE SPEED
    x = Esplora.readJoystickX();
    y = Esplora.readJoystickY();
    if ((y < yMid - cutoff) && lastInput != 1) {
      Keyboard.write('w');      // W = move forward
      Keyboard.write('\n');
      TFTdraw(0);
      lastInput = 1;
    } else if ((y > yMid + cutoff) && lastInput != 3) {
      Keyboard.write('s');      // S = move backward
      Keyboard.write('\n');
      TFTdraw(1);
      lastInput = 3;
    } else if ((x > xMid + cutoff) && lastInput != 2) {
      Keyboard.write('a');      // A = turn left
      Keyboard.write('\n');
      TFTdraw(2);
      lastInput = 2;
    } else if ((x < xMid - cutoff) && lastInput != 4) {
      Keyboard.write('d');      // D = turn right
      Keyboard.write('\n');
      TFTdraw(3);
      lastInput = 4;
    } else {
      if (lastInput != 0 && xState(x,y,xMid,yMid, cutoff)) {
        Keyboard.write('x');
        Keyboard.write('\n');
        TFTdraw(-1);
        lastInput = 0;
      }
    }
  }
  //--------------------------------------------------------------//

  delay(200);
}

boolean xState(int x, int y, int xMid, int yMid, int cutoff) {
  if (!(x < xMid - cutoff) && !(x > xMid + cutoff) && !(y < yMid - cutoff) && !(y > yMid + cutoff)) {
    return true;
  } else {
    return false;
  }
}

void TFTdraw(int dir) { // TFT DISPLAY
  if (dir == lastDir) { // nothing changed
    // DO NOTHING
  } else {
    lastDir = dir;
    EsploraTFT.stroke(0, 0, 0);
    EsploraTFT.fill(0, 0, 0);
    EsploraTFT.rect(60, 55, 40, 40); // if dir == -1 only runs this
    if (dir == 0) { // w
      EsploraTFT.stroke(255, 0, 0);
      EsploraTFT.fill(255, 0, 0);
      EsploraTFT.rect(70, 75, 20, 20);
      for (int i = 0; i < 20; i++) {
        EsploraTFT.line(60 + i, 75 - i, 100 - i, 75 - i);
      }
    } else if (dir == 1) { // s
      EsploraTFT.stroke(255, 0, 0);
      EsploraTFT.fill(255, 0, 0);
      EsploraTFT.rect(70, 55, 20, 20);
      for (int i = 0; i < 20; i++) {
        EsploraTFT.line(60 + i, 75 + i, 100 - i, 75 + i);
      }
    } else if (dir == 2) { // a
      EsploraTFT.stroke(255, 0, 0);
      EsploraTFT.fill(255, 0, 0);
      EsploraTFT.rect(80, 65, 20, 20);
      for (int i = 0; i < 20; i++) {
        EsploraTFT.line(80 - i, 55 + i, 80 - i, 95 - i);
      }
    } else if (dir == 3) { // d
      EsploraTFT.stroke(255, 0, 0);
      EsploraTFT.fill(255, 0, 0);
      EsploraTFT.rect(60, 65, 20, 20);
      for (int i = 0; i < 20; i++) {
        EsploraTFT.line(80 + i, 55 + i, 80 + i, 95 - i);
      }
    }
  }
}



/*
  / test
  / area

*/

