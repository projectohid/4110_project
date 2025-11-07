# include <PID_v1.h>
# include <Servo.h>

//# define show_A1_pin_data_on_serial_monitor // Comment this line if you don't want to show A1 pin's ADC count. This should be used only for calibration
//# define show_voltage_sensor_reading_on_serial_monitor // Comment this line if you don't want to show sensor data. This should only be used for calibration.
//# define show_pid_response_on_serial_plotter // Comment this line if you don't want pid response to visualize in oscilloscope.

///////////////////////////////////////////  PIN LIST /////////////////////////////////////////

short int rev_rot_pin = 2;                  // Q3 and Q4 pin
short int forward_rot_pin = 3;              // Q1 and Q2 pin
short int rheostatic_brake_switch = 4;
short int steering_wheel_pin = 5;
short int power_button_pin = 6;
short int reverse_button_pin = 7;
short int control_mode_button_pin = 8;
short int generator_terminal_changing_switch = 9;

// # define Kp_pin A3 // PID tuning
// # define Ki_pin A4 // PID tuning
// # define Kd_pin A5 // PID tuning

# define accelerator_pin A0
# define voltage_sensor_pin A1
# define steering_pot_sensor_pin A2

/////////////////////////////////////// GLOBAL VARIABLE LIST /////////////////////////////////////

double ref_duty_cycle = 0.8;
double active_duty_cycle = 0.8;
double controller_generated_duty_cycle = 0.6;
double error_signal = 0.0;

double Kp = 0.1;  //  Tuned
double Ki = 0.4;  //  Tuned
double Kd = 0.01; //  Tuned

bool power_button = false;
bool reverse_button = false;
bool control_mode_button = false;
bool emergency_shutdown = false;

int sensor_read_timer = 0;

Servo steer_motor;
PID pid(&error_signal,&controller_generated_duty_cycle,&ref_duty_cycle,Kp,Ki,Kd,REVERSE);

/////////////////////////////////////////// FUNCTION LIST /////////////////////////////////////////

void read_user_data(void);        /// Updates input port
void motor_on_forward(void);      /// Forward movement
void motor_off_forward(void);     /// Forward movement
void motor_on_reverse(void);      /// Reverse movement
void motor_off_reverse(void);     /// Reverse movement
void wait_till_motor_stop(void);  /// Gives time to motor to slow down before counter rotating it in order to avoid wear and tear of motor by hard breaking
void setup_and_run_motor(void);   /// Configures motor action
void run_motor(bool,float);       /// Manipulates H-bridge motor driver
void stop_motor(void);            /// Ceases motor action from any state

float speed2Dout_transducer(void);/// Produces output_duty_cycle
void summing_junc();              /// Generates error signal which is a global variable
float read_voltage_sensor(void);  /// Reads Vout from sensor connected across generator terminal and returns it
void run_feedback_path(void);     /// Accumulates all the components of closed loop controll process

//////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(9600);

  pinMode(power_button_pin,INPUT);
  pinMode(reverse_button_pin,INPUT);
  pinMode(control_mode_button_pin,INPUT);
  pinMode(accelerator_pin,INPUT);
  pinMode(voltage_sensor_pin,INPUT);
  pinMode(steering_wheel_pin,INPUT);

  // pinMode(Kp_pin,INPUT); // PID tuning
  // pinMode(Ki_pin,INPUT); // PID tuning
  // pinMode(Kd_pin,INPUT); // PID tuning

  pinMode(rev_rot_pin,OUTPUT); /// Reverse driver mosfet set
  pinMode(forward_rot_pin,OUTPUT); /// Forward driver mosfet set
  pinMode(generator_terminal_changing_switch,OUTPUT);
  pinMode(rheostatic_brake_switch,OUTPUT);
  pinMode(steering_wheel_pin,OUTPUT);
  steer_motor.attach(steering_wheel_pin);

  // Initializing relay
  digitalWrite(generator_terminal_changing_switch,HIGH); // Active low
  digitalWrite(rheostatic_brake_switch,HIGH); // Active low // Forward motion -> Relay untriggered


  // Initializing optocouplers
  digitalWrite(forward_rot_pin,LOW);
  digitalWrite(rev_rot_pin,LOW);

  // PID setup
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(90);
  pid.SetOutputLimits(0.5,1);
}

void loop() 
  {
    // Don't use any delay function inside loop

  // /////////////////////////////////////////////////////// PID tuning

  // #ifdef show_A1_pin_data_on_serial_monitor // Only for calibration
  //     Serial.println(analogRead(voltage_sensor_pin)); 
  // #endif 
  // #ifdef show_voltage_sensor_reading_on_serial_monitor // Only for calibration
  //     Serial.println(read_voltage_sensor());
  // #endif
  // #ifdef show_pid_response_on_serial_plotter // Only for data analysis
  //     sensor_read_timer ++;
  //     if(sensor_read_timer > 50)
  //       {
  //         Serial.print(ref_duty_cycle);
  //         Serial.print("  -----  ");
  //         Serial.print(controller_generated_duty_cycle);
  //         Serial.print("  -------  ");
  //         Serial.println(speed2Dout_transducer());
  //         sensor_read_timer = 0;
  //       }
  // #endif


  // Kp = double(map(analogRead(Kp_pin),0,1023,1,100)) / 10;
  // Ki = double(map(analogRead(Ki_pin),0,1023,0,100)) / 10;
  // Kd = double(map(analogRead(Kd_pin),0,1023,0,100)) / 1000;


  //pid.SetTunings(Kp ,Ki, Kd);

  // sensor_read_timer ++;
  // if(sensor_read_timer > 100)
  // {
  //   Serial.print("Kp ");
  //   Serial.print(Kp);
  //   Serial.print(" Ki ");
  //   Serial.print(Ki);
  //   Serial.print(" Kd ");
  //   Serial.println(Kd);
    
  //   sensor_read_timer = 0;
  // }
  // //////////////////////////////////////////////////////
  read_user_data();
  setup_and_run_motor();
}

//////////////////////////////////////////////////////////// Control action //////////////////////////////////////////////////////

float read_voltage_sensor()
{
    return (float(analogRead(voltage_sensor_pin) / 640.0) * 5.6);
}



float speed2Dout_transducer()
{
    float Vout = read_voltage_sensor();

    if(Vout < 0)        // If motor operates in reverse direction, voltage produced by generator will alter polarity
      Vout = (-1)*Vout;  // We will take the absolute value

    float output_duty_cycle = ((Vout)/5.6);

    if(output_duty_cycle < 0.5)
      output_duty_cycle = 0.5;
    else if(output_duty_cycle > 1.0)
      output_duty_cycle = 1.0;
     
    return output_duty_cycle;
}



void summing_junc()
{
  error_signal = 5*(ref_duty_cycle - speed2Dout_transducer());  /// error signal
}



void run_feedback_path()
{
  summing_junc();
  pid.Compute();
  
  if(controller_generated_duty_cycle >= 1)
    active_duty_cycle = 1.0;
  else if(controller_generated_duty_cycle <= 0.5)
    active_duty_cycle = 0.5;
  else
    active_duty_cycle = controller_generated_duty_cycle;
}

////////////////////////////////////////////////////////////////// General operation ////////////////////////////////////////////////////

void read_user_data()
{
    ref_duty_cycle = double(map(analogRead(accelerator_pin),0,1023,50,100)) / 100;

    int steer_value = analogRead(steering_pot_sensor_pin);
    steer_value = map(steer_value,0,1023,0,180);
    steer_motor.write(steer_value);

    if(digitalRead(power_button_pin) == HIGH)
      {
        power_button = !power_button;

        for(;;)
          {
            setup_and_run_motor();
            if(digitalRead(power_button_pin) == LOW)
              break;
          }
        if(power_button == false)
          {
            stop_motor();
            wait_till_motor_stop();
          }
        return;
      }
      
    
    if(digitalRead(reverse_button_pin) == HIGH)
      {
        reverse_button = !reverse_button;

        stop_motor();
        wait_till_motor_stop();

        if(reverse_button == false)
          {
            digitalWrite(rheostatic_brake_switch,HIGH); // Forward motion -> Relay untriggered
            digitalWrite(generator_terminal_changing_switch,HIGH);
          }
        else
          {
            digitalWrite(rheostatic_brake_switch,LOW); // Reverse motion -> Relay triggered
            digitalWrite(generator_terminal_changing_switch,LOW);
          }

        for(;;)
          {
            setup_and_run_motor();
            if(digitalRead(reverse_button_pin) == LOW)
              break;
          }
        return;
      }
    
      
    if(digitalRead(control_mode_button_pin) == HIGH)
      {
        control_mode_button = !control_mode_button;
        for(;;)
          {
            setup_and_run_motor();
            if(digitalRead(control_mode_button_pin) == LOW)
              break;
          }
        return;
      }
}



void setup_and_run_motor()
{
  if(power_button == true)
  {
   if(control_mode_button == false)     /// OPEN LOOP CONTROL
   {
      active_duty_cycle = ref_duty_cycle;

      if(reverse_button == false) // Forward motion
        {
          digitalWrite(generator_terminal_changing_switch,HIGH); // Relay untriggered
          run_motor(0,active_duty_cycle); 
        }
      else  // Reverse motion
        {
          digitalWrite(generator_terminal_changing_switch,LOW); // Relay triggered
          run_motor(1,active_duty_cycle); 
        }
   }
   else                                   /// CLOSE LOOP CONTROL
    {
      if(reverse_button == false)  // Forward motion
      { 
         digitalWrite(generator_terminal_changing_switch,HIGH);
         run_feedback_path();
         run_motor(0,active_duty_cycle);
      }
      else // Reverse motion 
      {
        digitalWrite(generator_terminal_changing_switch,LOW);
        run_feedback_path();
        run_motor(1,active_duty_cycle);
      }
    }
  }
  else
  {
    stop_motor();
  }
}



void run_motor(bool dir,float duty_cycle)   /// Motoring function
{
  float time_period_sec = 0.03;  // 30ms switching period
  
  if(dir == false)    
    {
      motor_on_forward();
      delay(duty_cycle * time_period_sec * 1000);
      motor_off_forward();
      delay((1 - duty_cycle) * time_period_sec * 1000);
    }
  else
    {
      motor_on_reverse();
      delay(duty_cycle * time_period_sec * 1000);
      motor_off_reverse();
      delay((1 - duty_cycle) * time_period_sec * 1000);
    }

}



void stop_motor()
{
  motor_off_forward();
  motor_off_reverse();
}

void motor_on_forward()
{
  digitalWrite(forward_rot_pin,HIGH);
}
void motor_off_forward()
{
  digitalWrite(forward_rot_pin,LOW);
}
void motor_on_reverse()
{
  digitalWrite(rev_rot_pin,HIGH);
}
void motor_off_reverse()
{
  digitalWrite(rev_rot_pin,LOW);
}



void wait_till_motor_stop()
{
 //for(;;)
  {
    //if(read_voltage_sensor() < 0.3)
      {
        delay(1000);
        return;
      }
  }
}
