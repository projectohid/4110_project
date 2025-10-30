#include <PID_v1.h>
///////////////////////////////////////////  PIN LIST /////////////////////////////////////////
// INPUT PINS 
short int power_button_pin = 6;
short int reverse_button_pin = 7;
short int control_mode_button_pin = 8;
#define accelerator_pin A0
#define voltage_sensor_pin A1

// FLAGS
bool power_button = false;        // same as power_button_pin = HIGH (true)
bool reverse_button = false;      // same as reverse_button_pin == HIGH (true)
bool control_mode_button = false; // same as control_mode_button_pin = HIGH (true, close loop control)
bool emergency_shutdown = false;  // no pins defined yet 


// OUTPUT PINS
short int rev_rot_pin = 2;     // Q3 pin
short int forward_rot_pin = 3; // Q1 pin
short int Q2_pin = 4;
short int Q4_pin = 5;
short int generator_terminal_changing_switch = 9;
short int rheostatic_brake_switch = 10; // what does it do?


/////////////////////////////////////// GLOBAL VARIABLE LIST /////////////////////////////////////

double ref_duty_cycle = 0.8;
double active_duty_cycle = 0.8;
double controller_generated_duty_cycle = 0.0;
double error_signal = 0.0;

double Kp = 2;
double Ki = 3;
double Kd = 2;

float time_period_sec = 0.05; // 50ms switching period

int motor_stop_delay = 1000; // in ms



short int gear_value = 1;
int sensor_read_timer = 0;

PID pid(&error_signal, &controller_generated_duty_cycle, &ref_duty_cycle, Kp, Ki, Kd, DIRECT);
/////////////////////////////////////////// FUNCTION LIST /////////////////////////////////////////

void read_user_data(void);       /// Updates input port
void motor_on_forward(void);     /// Forward movement
void motor_off_forward(void);    /// Forward movement
void motor_on_reverse(void);     /// Reverse movement
void motor_off_reverse(void);    /// Reverse movement
void wait_till_motor_stop(void); /// Gives time to motor to slow down before counter rotating it in order to avoid wear and tear of motor by hard breaking
void setup_and_run_motor(void);  /// Configures motor action
void run_motor(bool, float);     /// Manipulates H-bridge motor driver
void stop_motor(void);           /// Ceases motor action from any state

float speed2Dout_transducer(void); /// Produces output_duty_cycle
void summing_junc();               /// Generates error signal which is a global variable
float read_voltage_sensor(void);   /// Reads Vout from sensor connected across generator terminal and returns it
void run_feedback_path(void);      /// Accumulates all the components of closed loop controll process

//////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);

  pinMode(power_button_pin, INPUT);
  pinMode(reverse_button_pin, INPUT);
  pinMode(control_mode_button_pin, INPUT);
  pinMode(accelerator_pin, INPUT);
  pinMode(voltage_sensor_pin, INPUT);

  pinMode(rev_rot_pin, OUTPUT);     /// Reverse driver mosfet set
  pinMode(forward_rot_pin, OUTPUT); /// Forward driver mosfet set
  pinMode(Q2_pin, OUTPUT);
  pinMode(Q4_pin, OUTPUT);
  pinMode(generator_terminal_changing_switch, OUTPUT);
  pinMode(rheostatic_brake_switch, OUTPUT);

  // Initializing relays
  digitalWrite(generator_terminal_changing_switch, HIGH); // Active low
  digitalWrite(rheostatic_brake_switch, LOW);             // Active low
}

void loop()
{
  // Don't use any delay function inside loop
  if (Serial.available())
{
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); // removes \r and spaces

  Serial.print("Received from MATLAB: ");
  Serial.println(cmd); // print exactly what was read

  if (cmd.equalsIgnoreCase("POWER_ON")) {
    power_button = true;
  } 
  else if (cmd.equalsIgnoreCase("FORWARD")) {
    reverse_button = false;
  } 
  else if (cmd.equalsIgnoreCase("REVERSE")) {
    reverse_button = true;
  } 
  else if (cmd.equalsIgnoreCase("CLOSED_LOOP")) {
    control_mode_button = true;
  } 
  else if (cmd.equalsIgnoreCase("OPEN_LOOP")) {
    control_mode_button = false;
  } 
  else if (cmd.equalsIgnoreCase("EMER_SHUTDOWN")) {
    emergency_shutdown = true;
  }

  Serial.print("ACK: ");
  Serial.print(cmd);
  Serial.print(" | Power: "); Serial.print(power_button);
  Serial.print(" | Reverse: "); Serial.print(reverse_button);
  Serial.print(" | Control: "); Serial.print(control_mode_button);
  Serial.print(" | Emer: "); Serial.println(emergency_shutdown);
}

  read_user_data();

  // what is this doing here?
  setup_and_run_motor();
}

//////////////////////////////////////////////////////////// Control action //////////////////////////////////////////////////////

float read_voltage_sensor()
{
  return (float(analogRead(voltage_sensor_pin) / 570.0) * 4.8);
}

float speed2Dout_transducer()
{
  float Vout = read_voltage_sensor();

  if (Vout < 0)         // If motor operates in reverse direction, voltage produced by generator will alter polarity
    Vout = (-1) * Vout; // We will take the absolute value

  float output_duty_cycle = ((Vout) / 4.8);

  if (output_duty_cycle <= 0.5)
    output_duty_cycle = 0.5;
  else if (output_duty_cycle >= 1.0)
    ;
  output_duty_cycle = 1.0;

  return output_duty_cycle;
}

void summing_junc()
{
  error_signal = ref_duty_cycle - speed2Dout_transducer(); /// error signal
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

////////////////////////////////////////////////////////////////// General operation ////////////////////////////////////////////////////

void read_user_data() {
  ref_duty_cycle = double(map(analogRead(accelerator_pin), 0, 1023, 50, 100)) / 100;

  if ((digitalRead(power_button_pin) == HIGH || power_button == true) && reverse_button == false) {
    if(power_button != true) power_button = true;

    for (;;) {
      setup_and_run_motor();
      if (digitalRead(power_button_pin) == LOW || power_button == false || reverse_button == true)
        break;
    }
    // what is this part doing??
    if (power_button == false) {
      digitalWrite(rheostatic_brake_switch, HIGH);
      stop_motor();
      wait_till_motor_stop();
      digitalWrite(rheostatic_brake_switch, LOW);
    }
    return;
  }

  if (power_button == true && (digitalRead(reverse_button_pin) == HIGH || reverse_button == true)) {
    if(reverse_button != true) reverse_button = true;

    digitalWrite(rheostatic_brake_switch, HIGH);
    stop_motor();
    wait_till_motor_stop();
    digitalWrite(rheostatic_brake_switch, LOW);

    for (;;)
    {
      setup_and_run_motor();
      if (power_button == false || digitalRead(reverse_button_pin) == LOW || reverse_button == false)
        break;
    }
    return;
  }

  if (power_button == true && (digitalRead(control_mode_button_pin) == HIGH || control_mode_button == true)) {
    // control_mode_button = !control_mode_button;
    if(control_mode_button != true) control_mode_button = true;
    for (;;)
    {
      setup_and_run_motor();
      if (power_button == false || digitalRead(control_mode_button_pin) == LOW || control_mode_button == false)
        break;
    }
    return;
  }
}

void setup_and_run_motor() // checked
{
  if (power_button == true)
  {
    if (control_mode_button == false) /// OPEN LOOP CONTROL
    {
      active_duty_cycle = ref_duty_cycle;

      if (reverse_button == false)
      {
        digitalWrite(generator_terminal_changing_switch, HIGH);
        run_motor(0, active_duty_cycle); // dir = 0 => forward direction
      }
      else
      {
        digitalWrite(generator_terminal_changing_switch, LOW);
        run_motor(1, active_duty_cycle);// dir = 1 => reverse direction
      }
    }
    else /// CLOSE LOOP CONTROL
    {
      if (reverse_button == false)
      {
        digitalWrite(generator_terminal_changing_switch, HIGH);
        run_feedback_path();
        run_motor(0, active_duty_cycle);
      }
      else
      {
        digitalWrite(generator_terminal_changing_switch, LOW);
        run_feedback_path();
        run_motor(1, active_duty_cycle);
      }
    }
  }
  else
  {
    stop_motor();
  }
}

void run_motor(bool dir, float duty_cycle) // dir = 0 => forward, dir = 1 => reverse rotation
{
  // float time_period_sec = 0.05; // 50ms switching period

  if (dir == false)
  {
    digitalWrite(Q2_pin, HIGH);
    // digitalWrite(Q4_pin, LOW); // NOT SURE
    motor_on_forward();
    delay(duty_cycle * time_period_sec * 1000);
    motor_off_forward();
    delay((1 - duty_cycle) * time_period_sec * 1000);
  }
  else
  {
    digitalWrite(Q4_pin, HIGH);
    // digitalWrite(Q2_pin, LOW); // NOT SURE
    motor_on_reverse();
    delay(duty_cycle * time_period_sec * 1000);
    motor_off_reverse();
    delay((1 - duty_cycle) * time_period_sec * 1000);
  }
}

// checked
void stop_motor() {
  motor_off_forward(); motor_off_reverse();
  digitalWrite(Q2_pin, LOW); digitalWrite(Q4_pin, LOW);
}

// checked
void motor_on_forward() { digitalWrite(forward_rot_pin, HIGH); }
void motor_off_forward() { digitalWrite(forward_rot_pin, LOW); }
void motor_on_reverse() { digitalWrite(rev_rot_pin, HIGH); }
void motor_off_reverse() { digitalWrite(rev_rot_pin, LOW); }
void wait_till_motor_stop() { delay(motor_stop_delay); }
