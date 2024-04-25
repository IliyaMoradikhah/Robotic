void initiall() {
  // Motor
  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB15, OUTPUT);

  pinMode(PB8, PWM);
  pinMode(PB7, PWM);
  pinMode(PB6, PWM);
  pinMode(PA8, PWM);
  stop();
  // shoot
  pinMode(PC15, OUTPUT);
  // Pixy
  pixy.init();
  // OLED Start Up And requied rules for displaying
  display.begin(0X2, 0X3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();
  // Gy
  Serial1.begin(115200);
  Serial1.write(0xA5);
  Serial1.write(0x54);
  delay(500);
  Serial1.write(0xA5);
  Serial1.write(0x55);
  delay(500);
  Serial1.write(0xA5);
  Serial1.write(0x51);
}
void rotate() {
  KP = Heading;
  if (millis() - LastTime > 2000) {
    if ((Heading > -5 && Heading < 0) || (Heading < 5 && Heading >= 0)) {
      KP = 0, KI = 0, KD = 0;
    } else {
      KP = Heading;
      KI = (Heading + KI) * (millis() - LastTime) + KI;
      KD = (Heading - LastHeading) / (millis() - LastTime);
    }
    LastTime = millis();
    LastHeading = Heading;
  } else {
    if ((Heading > -5 && Heading < 0) || (Heading < 5 && Heading >= 0)) {
      KP = 0, KI = 0, KD = 0;
    } else {
      if (Heading >= 0) KP = 20 * pow(Heading, 0.4);
      else KP = -20 * pow(-Heading, 0.4);
      KI = 0, KD = 0;
    }
    gy = 70 * KP + 80 * KI + 20 * KD;
  }
}
void Sensors() {
  // Pixy
  uint16_t Blocks;
  isBall = false;
  isGoal = false;
  Blocks = pixy.getBlocks();
  if (Blocks) {
    // Sign 1
    for (int i = 0; i < Blocks; i++) {
      if (pixy.blocks[i].signature == 1) {
        isBall = true;
        xBall = pixy.blocks[i].y;
        yBall = pixy.blocks[i].x;
        angleBall = getAngle(xBall, yBall);
        distanceBall = sqrt(pow(xBall - xRobot, 2) + pow(yBall - yRobot, 2));
      }
    }
    // Sign 2
    for (int i = 0; i < Blocks; i++) {
      if (pixy.blocks[i].signature == 2) {
        isGoal = true;
        xGoal = pixy.blocks[i].y;
        yGoal = pixy.blocks[i].x;
        angleGoal = getAngle(xGoal, yGoal);
        if (angleGoal <= 180) goal = angleGoal;
        else goal = angleGoal - 360;
        distanceGoal = sqrt(pow(xGoal - xRobot, 2) + pow(yGoal - yRobot, 2));
      }
    }
  }
  // Sharp
  shr = analogRead(PA2);
  shl = analogRead(PA0);
  shb = analogRead(PA1);
  Diff = shr - shl;
  Diff *= 14;
  // LDR
  LDR_R = analogRead(PA6) - LDRR;
  LDR_L = analogRead(PA4) - LDRL;
  LDR_B = analogRead(PA5) - LDRB;
  LDR_F = analogRead(PA3) - LDRF;
  // Shoot
  shootAmount = analogRead(PB0);
  if (shootAmount > shootSense) shootSet = true;
  else shootSet = false;
}
void startUpLDR() {
  LDRR = analogRead(PA6);
  LDRL = analogRead(PA4);
  LDRB = analogRead(PA5);
  LDRF = analogRead(PA3);
}
void sensorGy() {
  Serial1.write(0xA5);
  Serial1.write(0x51);
  while (true) {
    buff[counter] = Serial1.read();
    if (counter == 0 && buff[0] != 0xAA) break;
    counter++;
    if (counter == 8) {
      counter = 0;
      if (buff[0] = 0xAA && buff[7] == 0x55) {
        Heading = (int16_t)(buff[1] << 8 | buff[2]) / 100;
      }
    }
  }
}
void motor(int ML1, int ML2, int MR2, int MR1) {
  ML1 += gy;
  ML2 += gy;
  MR2 += gy;
  MR1 += gy;
  // Requierd <<if>> statment for more acccurate VELOCITY
  if (ML1 > 65535) ML1 = 65535;
  if (ML2 > 65535) ML2 = 65535;
  if (MR2 > 65535) MR2 = 65535;
  if (MR1 > 65535) MR1 = 65535;

  if (ML1 < -65535) ML1 = -65535;
  if (ML2 < -65535) ML2 = -65535;
  if (MR2 < -65535) MR2 = -65535;
  if (MR1 < -65535) MR1 = -65535;

  if (ML1 > 0) {
    digitalWrite(PB15, 0);
    pwmWrite(PA8, ML1);
  } else {
    digitalWrite(PB15, 1);
    pwmWrite(PA8, 65535 + ML1);
  }
  if (ML2 > 0) {
    digitalWrite(PB14, 0);
    pwmWrite(PB8, ML2);
  } else {
    digitalWrite(PB14, 1);
    pwmWrite(PB8, 65535 + ML2);
  }
  if (MR2 > 0) {
    digitalWrite(PB13, 0);
    pwmWrite(PB7, MR2);
  } else {
    digitalWrite(PB13, 1);
    pwmWrite(PB7, 65535 + MR2);
  }
  if (MR1 > 0) {
    digitalWrite(PB12, 0);
    pwmWrite(PB6, MR1);
  } else {
    digitalWrite(PB12, 1);
    pwmWrite(PB6, 65535 + MR1);
  }
}
void printAll() {
  if (digitalRead(PB5)) {
    if (LCD == 0) {
      Serial1.write(0xA5);
      Serial1.write(0x54);
      delay(100);
      Serial1.write(0xA5);
      Serial1.write(0x55);
      delay(100);
      Serial1.write(0xA5);
      Serial1.write(0x51);
    } else if (LCD == 1) {
      startUpLDR();
    }
    while (digitalRead(PB5))
      ;
  }
  if (digitalRead(PB4)) {
    LCD++;
    LCD %= 2;
    while (digitalRead(PB4))
      ;
  }
  if (digitalRead(PA15)) {
    shoot();
    while (digitalRead(PA15));
  }
  if (LCD == 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("x: ");
    display.println(xBall);
    display.print("y: ");
    display.println(yBall);
    display.print("Angle: ");
    display.println(angleBall);
    display.print("GY: ");
    display.println(Heading);
    display.print("shr: ");
    display.println(shr);
    display.print("shl: ");
    display.println(shl);
    display.print("shb: ");
    display.println(shb);
    display.setCursor(64, 0);
    display.print("anG: ");
    display.println(angleGoal);
    display.setCursor(64, 10);
    display.print("dG: ");
    display.println(distanceGoal);
    display.display();
  }
  if (LCD == 1) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("LDR_F: ");
    display.println(LDR_F);
    display.print("LDR_L: ");
    display.println(LDR_L);
    display.print("LDR_R: ");
    display.println(LDR_R);
    display.print("LDR_B: ");
    display.println(LDR_B);
    display.print("shoot: ");
    display.println(shootAmount);
    display.print("distanceBall: ");
    display.println(distanceBall);
    // display.setTextSize(2);
    // display.setCursor(0, 0);
    // display.print(Diff);
    display.display();
  }
}
void move(int direct) {
  if (direct == 0) motor(v, v, -v, -v);
  else if (direct == 1) motor(v, v / 2, -v, v / 2);
  else if (direct == 2) motor(v, 0, -v, 0);
  else if (direct == 3) motor(v, -v / 2, -v, v / 2);
  else if (direct == 4) motor(v, -v, -v, v);
  else if (direct == 5) motor(v / 2, -v, -v / 2, v);
  else if (direct == 6) motor(0, -v, 0, v);
  else if (direct == 7) motor(-v / 2, -v, v / 2, v);
  else if (direct == 8) motor(-v, -v, v, v);
  else if (direct == 9) motor(-v, -v / 2, v, v / 2);
  else if (direct == 10) motor(-v, 0, v, 0);
  else if (direct == 11) motor(-v, v / 2, v, -v / 2);
  else if (direct == 12) motor(-v, v, v, -v);
  else if (direct == 13) motor(-v / 2, v, v / 2, -v);
  else if (direct == 14) motor(0, v, 0, -v);
  else if (direct == 15) motor(v / 2, v, -v / 2, -v);
}
void OUT() {
  LDR_COUNT = 0;
  if (LDR_F > LDRSense) {
    moveForSec(8, 5);
    while ((angleBall >= 270 || angleBall <= 90) && isBall == true && LDR_COUNT <= 150) {
      moveInside();
      Sensors();
      sensorGy();
      rotate();
      printAll();
      LDR_COUNT++;
    }
  } else if (LDR_L > LDRSense) {
    moveForSec(4, 6);
    while (angleBall >= 180 && isBall == true && LDR_COUNT <= 150) {
      moveInside();
      Sensors();
      sensorGy();
      rotate();
      printAll();
      LDR_COUNT++;
    }
  } else if (LDR_B > LDRSense) {
    moveForSec(0, 5);
    while ((angleBall >= 90 && angleBall <= 270) && isBall == true && LDR_COUNT <= 150) {
      moveInside();
      Sensors();
      sensorGy();
      rotate();
      printAll();
      LDR_COUNT++;
    }
  } else if (LDR_R > LDRSense) {
    moveForSec(12, 6);
    while (angleBall <= 180 && isBall == true && LDR_COUNT <= 150 && LDR_COUNT <= 150) {
      moveInside();
      Sensors();
      sensorGy();
      rotate();
      printAll();
      LDR_COUNT++;
    }
  } else if (LDR_F > LDRSense && LDR_R > LDRSense) {
    moveForSec(10, 4);
    while (angleBall >= 344 && angleBall <= 125 && isBall == true && LDR_COUNT <= 150) {
      moveInside();
      Sensors();
      sensorGy();
      rotate();
      printAll();
      LDR_COUNT++;
    }
  } else if (LDR_F > LDRSense && LDR_L > LDRSense) {
    moveForSec(6, 4);
    while (angleBall <= 4.5 && angleBall >= 235 && isBall == true && LDR_COUNT <= 150) {
      moveInside();
      Sensors();
      sensorGy();
      rotate();
      printAll();
      LDR_COUNT++;
    }
  } else if (LDR_B > LDRSense && LDR_R > LDRSense) {
    moveForSec(14, 4);
    while (angleBall >= 45 && angleBall <= 236.5 && isBall == true && LDR_COUNT <= 150) {
      moveInside();
      Sensors();
      sensorGy();
      rotate();
      printAll();
      LDR_COUNT++;
    }
  } else if (LDR_B > LDRSense && LDR_L > LDRSense) {
    moveForSec(2, 4);
    while (angleBall <= 320 && angleBall >= 135 && isBall == true && LDR_COUNT <= 150) {
      moveInside();
      Sensors();
      sensorGy();
      rotate();
      printAll();
      LDR_COUNT++;
    }
  }
}
void directions() {
  if ((angleBall >= 0 && angleBall < 11.5) || (angleBall > 349 && angleBall < 360)) dir = 0;
  else if (angleBall >= 11.5 && angleBall < 34) dir = 1;
  else if (angleBall >= 34 && angleBall < 56.5) dir = 2;
  else if (angleBall >= 56.5 && angleBall < 79) dir = 3;
  else if (angleBall >= 79 && angleBall < 101.5) dir = 4;
  else if (angleBall >= 101.5 && angleBall < 124.5) dir = 5;
  else if (angleBall >= 124.5 && angleBall < 146.5) dir = 6;
  else if (angleBall >= 146.5 && angleBall < 169) dir = 7;
  else if (angleBall >= 169 && angleBall < 191.5) dir = 8;
  else if (angleBall >= 191.5 && angleBall < 214) dir = 9;
  else if (angleBall >= 214 && angleBall < 236.5) dir = 10;
  else if (angleBall >= 236.5 && angleBall < 259) dir = 11;
  else if (angleBall >= 259 && angleBall < 281.5) dir = 12;
  else if (angleBall >= 281.5 && angleBall < 304) dir = 13;
  else if (angleBall >= 304 && angleBall < 326.5) dir = 14;
  else if (angleBall >= 326.5 && angleBall < 349) dir = 15;
}
void moveInside() {
  if (LDR_F > LDRSense) move(8);
  else if (LDR_L > LDRSense) move(4);
  else if (LDR_B > LDRSense) move(0);
  else if (LDR_R > LDRSense) move(12);
  else if (LDR_F > LDRSense && LDR_R > LDRSense) move(10);
  else if (LDR_F > LDRSense && LDR_L > LDRSense) move(6);
  else if (LDR_B > LDRSense && LDR_R > LDRSense) move(14);
  else if (LDR_B > LDRSense && LDR_L > LDRSense) move(2);
  else stop();
}
void moveForSec(int direct, int sec) {
  move(direct);
  for (int i = 0; i < sec; i++) {
    move(direct);
    Sensors();
    sensorGy();
    rotate();
    directions();
    printAll();
  }
}
void shootForSec(int sec) {
  for (int i = 0; i < sec; i++) {
    Sensors();
    sensorGy();
    OUT();
    rotate();    
    directions();
    printAll();
  }
}
void shoot() {
  if (shootSet && shootCount == 0) {
    digitalWrite(PC15, 1);
    shootForSec(3);
    digitalWrite(PC15, 0);
    shootForSec(9);
    shootCount = 1;
  } else if (shootSet == false) shootCount = 0;
}
int getAngle(int x, int y) {
  int Angle;
  Angle = atan2(y - yRobot, x - xRobot) * 180 / PI;
  Angle += 90;
  if (Angle < 0) Angle += 360;
  return Angle;
}
void moveAngle(int Angle) {
  int x = v * sin(Angle * PI / 180);
  int y = v * cos(Angle * PI / 180);
  motor((x + y), (y - x), (-x - y), (x - y));
}
void stop() {
  motor(0, 0, 0, 0);
}
void Algorithm1() {
  Sensors();
  printAll();
  sensorGy();
  rotate();
  OUT();
  shoot();

  if (isBall == true) {
      if (distanceBall < 85) shift();
      else moveAngle(angleBall);
  } else {
    if (shb > 1400) {
      motor(-Diff + 30000, Diff + 30000, Diff - 30000, -Diff - 30000);
    } else if (shb < 1100) {
      motor(-Diff - 30000, Diff - 30000, Diff + 30000, -Diff + 30000);
    } else {
      motor(-Diff, Diff, Diff, -Diff);
    }
  }
}
void shift() {
  if (angleBall > 354 || angleBall < 6)        moveAngle(angleBall);
  else if (angleBall > 6 && angleBall < 90)    moveAngle(angleBall + 35);
  else if (angleBall > 90 && angleBall < 180)  moveAngle(angleBall + 40);
  else if (angleBall > 180 && angleBall < 270) moveAngle(angleBall - 40);
  else if (angleBall > 270 && angleBall < 355) moveAngle(angleBall - 35);
}
void Algorithm2() {
  Sensors();
  printAll();
  sensorGy();
  OUT();
  shoot();
  // Area 1
  if (isBall == true) {
    if (Diff < -11000) {
      Heading += 41;
      rotate();
      if (distanceBall < 65) shift();
      else moveAngle(angleBall);
    }
    // Area 2
    else if (Diff > 8200) {
      Heading -= 41;
      rotate();
      if (distanceBall < 65) shift();
      else moveAngle(angleBall);
    }
    else {
      Heading = Heading;
      rotate();
      if (distanceBall < 65) shift();
      else moveAngle(angleBall);
    }
  }
  else {
    Heading = Heading;
    rotate();
  if (shb > 1400) {
    motor(-Diff + 30000, Diff + 30000, Diff - 30000, -Diff - 30000);
  } 
  else if (shb < 1100) {
    motor(-Diff - 30000, Diff - 30000, Diff + 30000, -Diff + 30000);
  } 
  else {
    motor(-Diff, Diff, Diff, -Diff);
  }
}
}