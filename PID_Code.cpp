// Motor control pins
#define rmf 2 // IN1
#define rmb 3 // IN2
#define lmf 4 // IN3
#define lmb 5 // IN4

// Motor speed pins
#define rms 6  // EnA
#define lms 9  // EnB

#define sensorNumber 8
int sensor[sensorNumber];

// Adjust based on your surface calibration (black & white)
int threshold[sensorNumber] = {566, 1226, 1256, 1291, 1311, 1331, 741, 1361};

// Timings for different maneuvers
#define turn_delay 5
#define u_turn_delay 10
#define stop_timer 30

// Speed variables
int max_speed = 250;
int left_motor_speed = 50;
int right_motor_speed = 50;
int turn_speed = 80;

// PID variables
int kp = 15;   // Smoother response
int kd = 80;   // Damping
int sensor_sum;
float calculated_pos;

float center_point = 3.5; // center of 8 sensors (index 0-7)
float line_position = 0;
float error = 0;
float previous_error = 0;
float PID = 0;
int turn_value = 0;

// LEDs for debugging (optional)
int led13 = 13;
int led12 = 12;

void setup() {
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);

  pinMode(rms, OUTPUT);
  pinMode(lms, OUTPUT);

  pinMode(led13, OUTPUT);
  pinMode(led12, OUTPUT);
  
  Serial.begin(9600);
}

void read_sensor() {
  line_position = 0;
  sensor_sum = 0;

  for (byte i = 0; i < sensorNumber; i++) {
    sensor[i] = analogRead(i);
    if (sensor[i] > threshold[i]) sensor[i] = 1;
    else sensor[i] = 0;
  }

  sensor_sum = (sensor[0] + sensor[1] + sensor[2] + sensor[3] +
                sensor[4] + sensor[5] + sensor[6] + sensor[7]);
  line_position = (sensor[0]*1 + sensor[1]*2 + sensor[2]*3 + sensor[3]*4 +
                   sensor[4]*5 + sensor[5]*6 + sensor[6]*7 + sensor[7]*8);
  if (sensor_sum) calculated_pos = line_position / sensor_sum;
}

void motor(int left, int right) {
  if (right > 0) {
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
  } else {
    right = -right;
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
  }

  if (left > 0) {
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  } else {
    left = -left;
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
  }

  if (left > max_speed) left = max_speed;
  if (right > max_speed) right = max_speed;

  analogWrite(lms, left);
  analogWrite(rms, right);
}

void Line_Follow() {
  while (true) {
    read_sensor();

    error = center_point - calculated_pos;
    PID = error * kp + kd * (error - previous_error);
    previous_error = error;

    int right_motor = right_motor_speed - PID;
    int left_motor = left_motor_speed + PID;

    // Constrain motor speeds
    if (right_motor > max_speed) right_motor = max_speed;
    if (left_motor > max_speed) left_motor = max_speed;

    motor(left_motor, right_motor);

    // --- Smart turn detection based on outer sensors ---

    // Sharp LEFT turn pattern
    if ((sensor[0] || sensor[1]) && !(sensor[2] || sensor[3] || sensor[4] || sensor[5] || sensor[6] || sensor[7])) {
      turn_value = 1;
    }

    // Sharp RIGHT turn pattern
    else if ((sensor[6] || sensor[7]) && !(sensor[0] || sensor[1] || sensor[2] || sensor[3] || sensor[4] || sensor[5])) {
      turn_value = 2;
    }

    // --- Turn Execution when line is lost ---
    if (sensor_sum == 0) {
      if (turn_value == 1) {  // LEFT
        delay(turn_delay);
        motor(-turn_speed, turn_speed);
        while (sensor[3] == 0 && sensor[4] == 0) read_sensor();
        turn_value = 0;
      }
      else if (turn_value == 2) {  // RIGHT
        delay(turn_delay);
        motor(turn_speed, -turn_speed);
        while (sensor[3] == 0 && sensor[4] == 0) read_sensor();
        turn_value = 0;
      }
      else {  // U-TURN
        delay(u_turn_delay);
        motor(turn_speed, -turn_speed);
        while (sensor[3] == 0 && sensor[4] == 0) read_sensor();
        turn_value = 0;
      }
    }

    // --- Stop Condition ---
    else if (sensor_sum == 8) {
      delay(stop_timer);
      read_sensor();
      if (sensor_sum == 8) {
        motor(0, 0);  // STOP
        while (sensor_sum == 8) read_sensor();  // Wait until it moves off
      } else if (sensor_sum == 0) {
        turn_value = 2;  // T-Section → Turn Right
      }
    }
  }
}

void loop() {
  Line_Follow();
}