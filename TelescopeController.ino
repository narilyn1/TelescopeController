//#include <SoftwareSerial.h>
#include "MotorController.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
   Ref http://www.company7.com/library/meade/LX200CommandSet.pdf
*/

//SoftwareSerial  s_sserial(2, 3);
MotorController s_raMotor(7, 6, 5, 4), s_decMotor(11, 10, 9, 8);
unsigned short  s_buttonPins[6] = {15, 16,  2, 17,  12, 14};
MOTOR_SPEED     s_speedMode = MOTOR_SPEED_FASTEST;
MOTOR_DIRECTION s_motorDirection = MOTOR_DIRECTION_SOUTH_DEC;
bool s_buttonStates[6] = {false, false, false, false, false, false};  // 0: left, 1: right, 2: up, 3: down, 4:speed, 5:direction
unsigned long   s_earthSpinLastUpdate = 0;
unsigned long   s_bothDecButtonPressMsec = 0;

#define BAUDRATE (115200)

static void initTimer();
static void updateButtons();
static String deg2DDMMSS(double direction);
static String deg2HHMMSS(double direction);
static void readSerial();
static Adafruit_SSD1306 s_display(-1);
static void initDisplay();
static void updateDisplay();
static void processEarthRotation();

void setup() {
  Serial.begin(BAUDRATE);
  //s_sserial.begin(9600);
  initDisplay();

  for (int i = 0; i < 6; i++) { // setup buttons
    pinMode(s_buttonPins[i], INPUT_PULLUP);
  }

  initTimer();

  s_raMotor.init();
  s_decMotor.init();
  s_raMotor.setCancelBacklashOnGuide();

  delay(100);

  s_earthSpinLastUpdate = micros();
}

ISR(TIMER1_OVF_vect) {
  // Timer割り込み

  // 自転の処理
  processEarthRotation();

  // ボタンの処理
  updateButtons();

  s_raMotor.update();
  s_decMotor.update();

  TCNT1 = 65500;
}

void loop() {
  updateDisplay();

  // checkUpdateDirection(); //この処理いっぺんやめて:CM#にまかせる

  delay(50);
}

void serialEvent() {
  // ASIAIRとの通信
  delay(20);
  readSerial();
}

static void initTimer()
{
  // https://qiita.com/suzukinori/items/939cc9f49e535c4eadd7
  TCCR1A = 0; // 初期化
  TCCR1B = 0; // 初期化
  TCNT1 = 3036;
  TCCR1B |= (1 << CS12); // CS12 -> 1  prescaler = 256
  TIMSK1 |= (1 << TOIE1); //TOIE -> 1
}

static void processEarthRotation()
{ // 自転の処理
  unsigned long nextUpdateTime = s_earthSpinLastUpdate + HALFSTEP_RATE_USEC;
  if (micros() > nextUpdateTime) {
    s_raMotor.cwStep();
    s_earthSpinLastUpdate = nextUpdateTime;
  }
}

static void updateButtons()
{
  // check for buttons.
  for (int i = 0; i < 5; i++) {
    if (s_buttonStates[i] == false) {
      if (digitalRead(s_buttonPins[i]) == LOW) {
        // ボタンが押された
        s_buttonStates[i] = true;
        switch (i) {
          case 0: // left (East)
            if (!s_buttonStates[1]) {
              s_raMotor.ccwRotate();
            } else {
              s_raMotor.rotateStop(); // 反対ボタン操作で動作停止
            }
            break;
          case 1: // right (West)
            if (!s_buttonStates[0]) {
              s_raMotor.cwRotate();
            } else {
              s_raMotor.rotateStop(); // 反対ボタン操作で動作停止
            }
            break;
          case 2: // up
            if (!s_buttonStates[3]) {
              s_decMotor.ccwRotate();
            } else {
              s_decMotor.rotateStop(); // 反対ボタン操作で動作停止
              s_bothDecButtonPressMsec = millis();
            }
            break;
          case 3: // down
            if (!s_buttonStates[2]) {
              s_decMotor.cwRotate();
            } else {
              s_decMotor.rotateStop(); // 反対ボタン操作で動作停止
              s_bothDecButtonPressMsec = millis();
            }
            break;
          case 4: // speed change
            break;
        }
        //Serial.print(i);
        //Serial.println(" Button Pressed");
      }
    } else if (s_buttonStates[i] == true) {
      if (digitalRead(s_buttonPins[i]) == HIGH) {
        // ボタンが離された
        s_buttonStates[i] = false;
        switch (i) {
          case 0: // right
          case 1: // left
            s_raMotor.rotateStop();
            break;
          case 2: // up
          case 3: // down
            s_decMotor.rotateStop();
            break;
          case 4: // speed change:
            switch (s_speedMode) {
              case MOTOR_SPEED_FASTEST:
                s_speedMode = MOTOR_SPEED_FAST;
                break;
              case MOTOR_SPEED_FAST:
                s_speedMode = MOTOR_SPEED_SLOW;
                break;
              case MOTOR_SPEED_SLOW:
                s_speedMode = MOTOR_SPEED_SLOWEST;
                break;
              case MOTOR_SPEED_SLOWEST:
                s_speedMode = MOTOR_SPEED_FASTEST;
                break;
            }
            s_raMotor.setMotorSpeed(s_speedMode);
            s_decMotor.setMotorSpeed(s_speedMode);
            break;
        }
        //Serial.print(i);
        //Serial.println(" Button Released");
      }
    }
  }
  // check direction switch
  switch (s_motorDirection) {
    case MOTOR_DIRECTION_NORTH_DEC:
      if (digitalRead(s_buttonPins[5]) == LOW) {
        s_motorDirection = MOTOR_DIRECTION_SOUTH_DEC;
        s_raMotor.setDirectionInversion(false);
        //Serial.println("Switch to South Dec direction");
      }
      break;
    case MOTOR_DIRECTION_SOUTH_DEC:
      if (digitalRead(s_buttonPins[5]) == HIGH) {
        s_motorDirection = MOTOR_DIRECTION_NORTH_DEC;
        s_raMotor.setDirectionInversion(true);
        //Serial.println("Switch to North Dec direction");
      }
      break;
  }

  // Decボタン二つ押し長押し判定
  if (s_bothDecButtonPressMsec > 0 && s_bothDecButtonPressMsec + 1000 < millis()) {
    if (s_decMotor.getDirectionInversion()) {
      s_decMotor.setDirectionInversion(false);
    } else {
      s_decMotor.setDirectionInversion(true);
    }
    s_bothDecButtonPressMsec = 0;
  }
}

static String dig2(int number) {
  number = abs(number);
  if (number < 10) {
    return String("0") + String(number);
  }
  return String(number);
}

static String deg2DDMMSS(double direction)
{
  int dd, mm, ss;
  char s = (direction >= 0) ? '+' : '-';
  dd = (int)(direction);
  mm = (int)((direction - (int)(direction)) * 60.0);
  ss = (int)((direction - (int)(direction) - (mm / 60.0)) * 3600.0);
  String ret = s + String(dig2(dd) + "*" + dig2(mm) + "\'" + dig2(ss));
  return ret;
}

static String deg2HHMMSS(double direction)
{
  int hh, mm, ss;
  while (direction < 0) {
    direction += 360.0;
  }
  hh = (int)(direction * 24 / 360);
  mm = (int)((direction * 24.0 / 360.0 - hh) * 60.0);
  ss = (int)((direction * 24.0 / 360.0 - hh - (mm / 60.0)) * 3600.0);
  String ret = dig2(hh) + ':' + dig2(mm) + ':' + dig2(ss);
  return ret;
}

static void readSerial()
{
  const static char nak[2] = {0x15, 0};

  int byteLength;
  if ((byteLength = Serial.available()) > 0) {
    //String r_str = Serial.readString(); // Serial.readStringはめちゃくちゃ遅いので使用禁止
    char buff[128];
    if (byteLength > 127) {
      byteLength = 127;
    }
    Serial.readBytes(buff, byteLength);
    buff[byteLength] = 0;
    String r_str = buff;

    if (r_str.c_str()[0] == 0x06) {
      // ACK
      Serial.println("P"); // Set Polar Mode.
    } else {
      int startp, endp = 0, len = r_str.length();

      while (endp < len) {
        startp = endp;
        while (r_str.c_str()[startp] != ':') {
          startp++;
          if (startp >= len) {
            return;
          }
        }
        endp = startp;
        while (r_str.c_str()[endp] != '#') {
          endp++;
          if (endp >= len) {
            return;
          }
        }

        String str = r_str.substring(startp, endp + 1);

        if (str == ":GVP#") {
          // Product Name
          Serial.print("NRT#");
        }
        if (str == ":GVT#") {
          // Firmware Time
          Serial.print("00:00:00#");
        }
        if (str == ":GVN#") {
          // Firmware Number
          Serial.print("00.0#");
        }
        if (str == ":GW#") {
          // Telescope Status T#: Tracking on, 0#: Not in slew, P#: Parked
          // TODO: Slew状態を判別して返す
          Serial.print("T#");
        }
        if (str == ":GA#") {
          // Alignedment mode P#: Polar, A: AltAz, L: Land
          Serial.print("P#");
        }
        if (str == ":GR#") {
          // Asked for telescope Ra. "HH:MM:SS#"
          Serial.print(deg2HHMMSS(s_raMotor.getDirection()) + "#");
        }
        if (str == ":GD#") {
          // Asked for telescope Dec. "+DD*MM'SS"
          Serial.print(deg2DDMMSS(s_decMotor.getDirection()) + "#");
        }
        if (str == ":Gr#") {
          // Asked for target Ra. "HH:MM:SS#"
          Serial.print(deg2HHMMSS(s_raMotor.getTargetDirection()) + "#");
        }
        if (str == ":Gd#") {
          // Asked for target Dec. "+DD*MM'SS"
          Serial.print(deg2DDMMSS(s_decMotor.getTargetDirection()) + "#");
        }
        if (str.substring(0, 3) == ":Sr" && str.substring(8, 9) == ":") {
          // Set target object RA HH:MM:SS
          int hh = str.substring(3, 5).toInt();
          int mm = str.substring(6, 8).toInt();
          int ss = str.substring(9, 11).toInt();
          double ra = ((double)hh + (double)mm / 60.0 + (double)ss / 3600.0) * 360.0 / 24.0;
          s_raMotor.setTargetDirection(ra);
          Serial.print(1);
        }
        if (str.substring(0, 3) == ":Sr" && str.substring(8, 9) == ".") {
          // Set target object RA HH:MM.T
          int    hh = str.substring(3, 5).toInt();
          float  mm = str.substring(6, 10).toFloat();
          double ra = ((double)hh + (double)mm / 60.0) * 360.0 / 24.0;
          s_raMotor.setTargetDirection(ra);
          Serial.print(1);
        }
        if (str.substring(0, 3) == ":Sd") {
          if (str.substring(9, 10) == ":") {
            // Set target object DEC sDD*MM:SS
            int dd = str.substring(3, 6).toInt();
            int mm = str.substring(7, 9).toInt();
            int ss = str.substring(10, 12).toInt();
            double dec = (double)dd + (double)mm / 60.0 * (dd >= 0 ? 1.0 : -1.0) + (double)ss / 3600.0 * (dd >= 0 ? 1.0 : -1.0);
            s_decMotor.setTargetDirection(dec);
            Serial.print(1);
          } else {
            // Set target object DEC sDD*MM
            int dd = str.substring(3, 6).toInt();
            int mm = str.substring(7, 9).toInt();
            double dec = (double)dd + (double)mm / 60.0 * (dd >= 0 ? 1.0 : -1.0);
            s_decMotor.setTargetDirection(dec);
            Serial.print(1);
          }
        }
        if (str == ":CM#") {
          // Synchronizes the telescope's position with the currently selected database object's coordinates.
          s_raMotor.setDirection(s_raMotor.getTargetDirection());
          s_decMotor.setDirection(s_decMotor.getTargetDirection());
          Serial.print("Target name#");
        }
        if (str == ":Gc#") {
          // Get Calendar Format.
          Serial.print("24#");
        }
        if (str == ":GC#") {
          // Get date.
          Serial.print("04/23/77#");
        }
        if (str == ":GT#") {
          // Get Tracking rate.
          Serial.print("0.5#");
        }
        if (str == ":Gg#") {
          // Get Longitude.
          Serial.print("220*30#");
        }
        if (str == ":Gt#") {
          // Get Lat.
          Serial.print("+36.30#");
        }
        if (str == ":GG#") {
          // Get UTC
          Serial.print("-09#");
        }
        if (str == ":GM#") {
          // Get Site 1 Name
          Serial.print("Site Name#");
        }
        if (str == ":GL#") {
          // Get Local Time.
          Serial.print("00:00:00#");
        }
        if (str == ":SyGPDCO#") {
          // Sets the object selection string used by the FIND/BROWSE command.
          Serial.print("1");
        }
        if (str.substring(0, 3) == ":SG") {
          // Set Time UTC
          Serial.print("1");
        }
        if (str.substring(0, 3) == ":SL") {
          // Set Local Time
          Serial.print("1");
        }
        if (str.substring(0, 3) == ":Sg") {
          // Set longitude
          Serial.print("1");
        }
        if (str.substring(0, 3) == ":St") {
          // Set latitdue
          Serial.print("1");
        }
        if (str.substring(0, 3) == ":SS") {
          // Sets the local sideral time
          Serial.print("1");
        }
        if (str.substring(0, 3) == ":SC") {
          // Set Date
          Serial.print("1");
        }
        if (str.substring(0, 3) == ":Sf") {
          // Set faint magnitude limit
          Serial.print("1");
        }
        if (str == ":D#") {
          // Return progress bar
          String str;
          double rDiff = s_raMotor.getTargetDirection() - s_raMotor.getDirection();
          double dDiff = s_decMotor.getTargetDirection() - s_decMotor.getDirection();
          double diff = rDiff * rDiff + dDiff * dDiff;
          if (diff * diff > 5.0 * 5.0) { //  over 5deg
            str = "---#";
          } else if (diff * diff > 1.0 * 1.0) { // within 5deg
            str = "--#";
          } else if (diff * diff > 1.0 / 60.0 / 60.0) { // within 1deg
            str = "-#";
          } else {  // within 1min
            str = "#";
          }
          Serial.print(str);
        }

        // Slew Operation
        if (str == ":Me#") {
          // Move Telescope to East at current slew rate
          s_raMotor.ccwRotate(); // left
        }
        if (str == ":Qe#") {
          // Halt eastward Slews
          s_raMotor.rotateStop();
        }
        if (str == ":Mn#") {
          // Move Telescope to East at current slew rate
          s_decMotor.ccwRotate(); // up
        }
        if (str == ":Qn#") {
          // Halt northward Slews
          s_decMotor.rotateStop();
        }
        if (str == ":Ms#") {
          // Move Telescope to East at current slew rate
          s_decMotor.cwRotate(); // down
        }
        if (str == ":Qs#") {
          // Halt southward Slews
          s_decMotor.rotateStop();
        }
        if (str == ":Mw#") {
          // Move Telescope to East at current slew rate
          s_raMotor.cwRotate(); // right
        }
        if (str == ":Qw#") {
          // Halt westward Slews
          s_raMotor.rotateStop();
        }
        if (str == ":MS#") {
          // Slew to Target Object
          s_raMotor.goTo();
          s_decMotor.goTo();
          Serial.print("0");
        }
        if (str == ":Q#") {
          // Halt all current slewing
          s_raMotor.rotateStop();
          s_decMotor.rotateStop();
        }

        // Guide Operation  :Mg?DDDD#
        if (str.substring(0, 3) == ":Mg") {
          String dir = str.substring(3, 4);
          String msecStr = str.substring(4, 8);
          int msec = msecStr.toInt();
          switch (dir.c_str()[0]) {
            case 'e':
              s_raMotor.ccwAdjust(msec);
              break;
            case 'n':
              s_decMotor.ccwAdjust(msec);
              break;
            case 's':
              s_decMotor.cwAdjust(msec);
              break;
            case 'w':
              s_raMotor.cwAdjust(msec);
              break;
          }
        }

        // Slew Options
        if (str == ":RC#") {
          // Set Slew reate to Centering Rate
          s_speedMode = MOTOR_SPEED_SLOW;
        }
        if (str == ":RG#") {
          // Set Slew reate to Guiding Rate
          s_speedMode = MOTOR_SPEED_SLOWEST;
        }
        if (str == ":RM#") {
          // Set Slew reate to Find Rate
          s_speedMode = MOTOR_SPEED_FAST;
        }
        if (str == ":RS#") {
          // Set Slew reate to Fastest Rate
          s_speedMode = MOTOR_SPEED_FASTEST;
        }
      }
    }
  }
}

static void initDisplay()
{
  s_display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  s_display.setRotation(2);
  s_display.clearDisplay();

  s_display.setTextSize(2);
  s_display.setTextColor(WHITE);
}

static void updateDisplay()
{
  s_display.clearDisplay();
  s_display.setCursor(0, 0);

  String raMark, decMark, spdMark;
  if (s_raMotor.getDirectionInversion()) {
    raMark = "N";
  } else {
    raMark = "S";
  }
  if (s_decMotor.getDirectionInversion()) {
    decMark = "";
  } else {
    decMark = "R";
  }

  switch (s_speedMode) {
    case MOTOR_SPEED_FASTEST:
      spdMark = "S";
      break;
    case MOTOR_SPEED_FAST:
      spdMark = "M";
      break;
    case MOTOR_SPEED_SLOW:
      spdMark = "C";
      break;
    case MOTOR_SPEED_SLOWEST:
      spdMark = "G";
      break;
  }

  s_display.println(raMark + deg2HHMMSS(s_raMotor. getDirection()) + spdMark);
  s_display.println(         deg2DDMMSS(s_decMotor.getDirection()) + decMark);

  s_display.display();
}
