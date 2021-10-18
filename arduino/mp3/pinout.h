#define SENSOR_LEFT A1
#define SENSOR_RIGHT A0

#define MOTOR_LEFT 2
#define MOTOR_RIGHT 1

struct SensorData{
  bool left;
  bool right;

  uint16_t rawLeft;
  uint16_t rawRight;
};

struct MotorData {
  int16_t left;
  int16_t right;
};
