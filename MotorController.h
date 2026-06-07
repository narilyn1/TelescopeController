
// 86164.098903691[sec] / (48[motor step] * 120[gear] * 144[worm gear]) / 2[half step] + 0.0519411285[sec]
#define HALFSTEP_RATE_USEC (51941)
#define BACKLASH_STEP (420)

typedef enum {
  MOTOR_SPEED_FASTEST = 0,
  MOTOR_SPEED_FAST,
  MOTOR_SPEED_SLOW,
  MOTOR_SPEED_SLOWEST,
} MOTOR_SPEED;

typedef enum {
  MOTOR_STATE_STOP = 0,
  MOTOR_STATE_ROTATE_CCW = -1,
  MOTOR_STATE_ROTATE_CW = 1,
} MOTOR_STATE;

typedef enum {
  MOTOR_DIRECTION_NORTH_DEC = 1,
  MOTOR_DIRECTION_SOUTH_DEC = -1,
} MOTOR_DIRECTION;


class MotorController
{
  public:
    MotorController(unsigned short pin0, unsigned short pin1, unsigned short pin2, unsigned short pin3);  
    virtual ~MotorController();

    void init();

    void update();

    // 回転速度を設定する
    void setMotorSpeed(MOTOR_SPEED motorSpeed) {
      m_motorSpeed = motorSpeed;
    }

    // 回転速度を取得する
    MOTOR_SPEED getMotorSpeed() {
      return m_motorSpeed;
    }

    // 時計回りに1Step回転するように内部状態を設定する
    void cwStep() {
      // 時計回りに1step
      if(!m_directionInversion) {
        incrementStep();
      } else {
        decrementStep();
      }
    }
    
    // 反時計回りに1Step回転するように内部状態を設定する
    void ccwStep() {
      // 反時計回りに1step
      if(!m_directionInversion) {
        decrementStep();
      } else {
        incrementStep();
      }
    }

    // 時計回りに回転を開始する
    void cwRotate() {
      m_motorState = MOTOR_STATE_ROTATE_CW;
    }
    // 反時計回りに回転を開始する
    void ccwRotate() {
      m_motorState = MOTOR_STATE_ROTATE_CCW;
    }
    // 回転を停止する
    void rotateStop() {
      m_motorState = MOTOR_STATE_STOP;
      m_isInGoto = false;
    }

    // 一定時間MOTOR_SPEED_SLOWESTで微調整回転する
    void cwAdjust(int msec);
    
    // 一定時間MOTOR_SPEED_SLOWESTで微調整回転する
    void ccwAdjust(int msec);

    // 現在の向きを取得する
    double getDirection() {
      return m_direction;
    }

    // 現在の向きを設定する
    void setDirection(double direction) {
      m_direction = direction;
    }

    // 対象物の向きを設定する
    void setTargetDirection(double target) {
      m_targetDirection = target;
    }

    // 対象物の向きを取得する
    double getTargetDirection() {
      return m_targetDirection;
    }

    // 対象物に向けて回転を開始する
    void goTo();
    
    // 軸が反転しており，向きが反転する場合に設定する
    void setDirectionInversion(bool inversion) {
      m_directionInversion = inversion;
    }

    bool getDirectionInversion() {
      return m_directionInversion;
    }

    bool isInBacklash() {
      return m_backlashStep > 0 && m_backlashStep < BACKLASH_STEP;
    }

    void setCancelBacklashOnGuide() {
      m_cancelGuideBacklash = true;
    }
    
  private:
    // Stepping Motorの状態を内部状態にシンクする
    void sync();

    void sleepMotor();

    // 内部状態を1Step進める
    void incrementStep() {
      m_motorStep = (++m_motorStep) % 8;
      if(m_backlashStep < BACKLASH_STEP) {
        m_backlashStep++;
      }
    }

    // 内部状態を1Step戻す
    void decrementStep() {
      m_motorStep = (7 + m_motorStep) % 8;
      if(m_backlashStep > 0) {
        m_backlashStep--;
      }
    }

    int m_motorStep;
    int m_lastMotorStep;
    unsigned short m_pin[4];
    double m_direction;
    double m_targetDirection;
    MOTOR_SPEED m_motorSpeed;
    MOTOR_STATE m_motorState;
    unsigned long m_lastUpdate; // usec
    unsigned long m_lastSync;   // usec
    bool m_directionInversion;
    bool m_isInGoto;
    short m_isInAdjustRotation; // +: cw, -: ccw
    unsigned long m_adjustLastUpdate; // usec
    unsigned long m_adjustUntilMsec;
    int m_backlashStep;
    bool m_cancelGuideBacklash;
};
