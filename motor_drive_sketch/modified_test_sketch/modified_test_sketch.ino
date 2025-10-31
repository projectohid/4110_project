#include <PID_v1.h>

///////////////////////////////////////////  PIN LIST /////////////////////////////////////////
// INPUT PINS 
short int power_button_pin = 6;
short int reverse_button_pin = 7;
short int control_mode_button_pin = 8;
#define accelerator_pin A0
#define voltage_sensor_pin A1

// FLAGS (software command flags)
bool power_button = false;        // true if MATLAB sent POWER_ON
bool reverse_button = false;      // true if MATLAB sent REVERSE
bool control_mode_button = false; // true if MATLAB sent CLOSED_LOOP
bool emergency_shutdown = false;  // reserved for future use

// OUTPUT PINS
short int rev_rot_pin = 2;     
short int forward_rot_pin = 3;
short int Q2_pin = 4;
short int Q4_pin = 5;
short int generator_terminal_changing_switch = 9;
short int rheostatic_brake_switch = 10;

/////////////////////////////////////// GLOBAL VARIABLE LIST /////////////////////////////////////

double ref_duty_cycle = 0.8;
double active_duty_cycle = 0.8;
double controller_generated_duty_cycle = 0.0;
double error_signal = 0.0;
double measured_speed = 0.0;

double Kp = 2;
double Ki = 3;
double Kd = 2;

float time_period_sec = 0.05; // 50ms switching period
int motor_stop_delay = 1000;  // ms delay for braking
short int gear_value = 1;

PID pid(&measured_speed, &controller_generated_duty_cycle, &ref_duty_cycle, Kp, Ki, Kd, DIRECT);

/////////////////////////////////////////// FUNCTION LIST /////////////////////////////////////////

void handleSerial(void);
void read_user_data(void);
void setup_and_run_motor(void);
void run_motor(bool, float);
void stop_motor(void);
void wait_till_motor_stop(void);
float read_voltage_sensor(void);
float speed2Dout_transducer(void);
void summing_junc(void);
void run_feedback_path(void);

//////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);

  pinMode(power_button_pin, INPUT);
  pinMode(reverse_button_pin, INPUT);
  pinMode(control_mode_button_pin, INPUT);
  pinMode(accelerator_pin, INPUT);
  pinMode(voltage_sensor_pin, INPUT);

  pinMode(rev_rot_pin, OUTPUT);
  pinMode(forward_rot_pin, OUTPUT);
  pinMode(Q2_pin, OUTPUT);
  pinMode(Q4_pin, OUTPUT);
  pinMode(generator_terminal_changing_switch, OUTPUT);
  pinMode(rheostatic_brake_switch, OUTPUT);

  // Initial relay states
  digitalWrite(generator_terminal_changing_switch, HIGH); // Active low
  digitalWrite(rheostatic_brake_switch, LOW);             // Active low

  pid.SetMode(AUTOMATIC);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  handleSerial();       // process MATLAB commands
  read_user_data();     // read combined user + hardware input
  setup_and_run_motor(); // run motor according to state
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// SERIAL HANDLER

void handleSerial()
{
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // remove \r, spaces, and newlines

    Serial.print("Received from MATLAB: ");
    Serial.println(cmd);

    if (cmd.equalsIgnoreCase("POWER_ON"))
      power_button = true;
    else if (cmd.equalsIgnoreCase("POWER_OFF"))
      power_button = false;
    else if (cmd.equalsIgnoreCase("FORWARD"))
      reverse_button = false;
    else if (cmd.equalsIgnoreCase("REVERSE"))
      reverse_button = true;
    else if (cmd.equalsIgnoreCase("CLOSED_LOOP"))
      control_mode_button = true;
    else if (cmd.equalsIgnoreCase("OPEN_LOOP"))
      control_mode_button = false;
    else if (cmd.equalsIgnoreCase("EMER_SHUTDOWN"))
      emergency_shutdown = true; // reserved for future use

    Serial.print("ACK: ");
    Serial.print(cmd);
    Serial.print(" | Power: "); Serial.print(power_button);
    Serial.print(" | Reverse: "); Serial.print(reverse_button);
    Serial.print(" | Control: "); Serial.print(control_mode_button);
    Serial.print(" | Emer: "); Serial.println(emergency_shutdown);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL + MOTOR MANAGEMENT

void read_user_data()
{
  // Combine hardware and software button inputs
  bool hw_power   = digitalRead(power_button_pin);
  bool hw_reverse = digitalRead(reverse_button_pin);
  bool hw_mode    = digitalRead(control_mode_button_pin);

  bool power_active  = hw_power   || power_button;
  bool reverse_active = hw_reverse || reverse_button;
  bool closed_loop_active = hw_mode || control_mode_button;

  // Update reference duty cycle from accelerator
  ref_duty_cycle = double(map(analogRead(accelerator_pin), 0, 1023, 50, 100)) / 100.0;

  // Update global flags (these determine the behavior in setup_and_run_motor)
  power_button = power_active;
  reverse_button = reverse_active;
  control_mode_button = closed_loop_active;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// MOTOR CONTROL

void setup_and_run_motor()
{
  if (power_button == true)
  {
    if (control_mode_button == false) // OPEN LOOP
    {
      active_duty_cycle = ref_duty_cycle;

      if (reverse_button == false)
      {
        digitalWrite(generator_terminal_changing_switch, HIGH);
        run_motor(0, active_duty_cycle); // forward
      }
      else
      {
        digitalWrite(generator_terminal_changing_switch, LOW);
        run_motor(1, active_duty_cycle); // reverse
      }
    }
    else // CLOSED LOOP
    {
      run_feedback_path();

      if (reverse_button == false)
      {
        digitalWrite(generator_terminal_changing_switch, HIGH);
        run_motor(0, active_duty_cycle);
      }
      else
      {
        digitalWrite(generator_terminal_changing_switch, LOW);
        run_motor(1, active_duty_cycle);
      }
    }
  }
  else
  {
    stop_motor();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// MOTOR ACTIONS

void run_motor(bool dir, float duty_cycle)
{
  if (dir == false)
  {
    digitalWrite(Q2_pin, HIGH);
    digitalWrite(forward_rot_pin, HIGH);
    delay(duty_cycle * time_period_sec * 1000);
    digitalWrite(forward_rot_pin, LOW);
    delay((1 - duty_cycle) * time_period_sec * 1000);
    digitalWrite(Q2_pin, LOW);
  }
  else
  {
    digitalWrite(Q4_pin, HIGH);
    digitalWrite(rev_rot_pin, HIGH);
    delay(duty_cycle * time_period_sec * 1000);
    digitalWrite(rev_rot_pin, LOW);
    delay((1 - duty_cycle) * time_period_sec * 1000);
    digitalWrite(Q4_pin, LOW);
  }
}

void stop_motor()
{
  digitalWrite(forward_rot_pin, LOW);
  digitalWrite(rev_rot_pin, LOW);
  digitalWrite(Q2_pin, LOW);
  digitalWrite(Q4_pin, LOW);
}

void wait_till_motor_stop()
{
  delay(motor_stop_delay);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// FEEDBACK CONTROL

float read_voltage_sensor()
{
  return (float(analogRead(voltage_sensor_pin)) / 570.0) * 4.8;
}

float speed2Dout_transducer()
{
  float Vout = read_voltage_sensor();

  if (Vout < 0)
    Vout = (-1) * Vout;

  float output_duty_cycle = Vout / 4.8;

  if (output_duty_cycle <= 0.5)
    output_duty_cycle = 0.5;
  else if (output_duty_cycle >= 1.0)
    output_duty_cycle = 1.0;

  return output_duty_cycle;
}

void summing_junc()
{
  measured_speed = speed2Dout_transducer();
  error_signal = ref_duty_cycle - measured_speed;
}

void run_feedback_path()
{
  summing_junc();
  pid.Compute();

  if (controller_generated_duty_cycle >= 1)
    active_duty_cycle = 1.0;
  else if (controller_generated_duty_cycle <= 0.5)
    active_duty_cycle = 0.5;
  else
    active_duty_cycle = controller_generated_duty_cycle;
}
