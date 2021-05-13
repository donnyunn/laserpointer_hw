#include <Stepper.h>
#include <Wire.h>
#include <math.h>
#include <avr/wdt.h>
//#include "LowPower.h"
#define degrees(rad) ((rad)*RAD_TO_DEG)

#define endX A1                               //connect X endstop to A1 pin(PITCH)
#define endZ A2                               //connect Z endstop to A2 pin(ROLL)
#define LASER 13                              //connect laser to A3 pin
#define MOTOR 12
#define ID 0x41                                  //set I2C address
//-------------------------I2C REGISTERS--------------------------
enum {
  CTRL = 0,
  MOVEX_H,
  MOVEX_L,
  MOVEY_H,
  MOVEY_L,
  OFFSETX_H,
  OFFSETX_L,
  OFFSETY_H,
  OFFSETY_L,
  XCAL_H,
  XCAL_L,
  ZCAL_H,
  ZCAL_L,
  YDIST_H,
  YDIST_L,
  TICK_X,
  TICK_Y,
  I2C_REG_NUM
} I2C_REGISTER;
volatile unsigned char i2c_reg[I2C_REG_NUM];
typedef struct {
  bool hw_reset;
  bool laser;
  bool motor;
  int move_x;
  int move_y;
  int offset_x;
  int offset_y;
  char tick_x;
  char tick_y;

  bool event;
} i2c_cmd_t;
i2c_cmd_t cmd = {
  false, false, false, 0, 0, 0, 0, 0, 0, false
};
i2c_cmd_t complete;
//------------------------INPUT PARAMETERS------------------------

double y = 150;                                 //distance from surface to ceiling
double w = 284.4;                               //Width of table
double h = 142.2;                               //Length of table
double w_offset = 0;                           //Width offset
double h_offset = 0;                           //Length offset
double x_cal = 490;                              // home to center angle(PITCH)
double z_cal = 291;                               //home to center angle(ROLL)
double roll_prev = 0;
double pitch_prev = 0;
//-----------------------------------------------------------------

double motor_roll = 0;
double motor_pitch = 0;
int step_roll = 0;
int step_pitch = 0;
int final_roll_step = 0;
int final_pitch_step = 0;

typedef struct {                              //Structure to save calculated angles
  double yaw;
  double pitch;
} angles;

angles calc;

const double steps = 0.17578125;                       //no. of steps for 1 rotation
const int st = 2048;
Stepper Motor_Pitch(st, 7, 5, 6, 4);            //PITCH Motor
Stepper Motor_Roll(st, 11, 9, 10, 8);          //ROLL Motor

void motor_calib();

void setup()
{
  pinMode(endX, INPUT_PULLUP);                      //Pitch Endstop
  pinMode(endZ, INPUT_PULLUP);                      //Roll Endstop
  pinMode(LASER, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  motor_ctrl(1);

  Motor_Pitch.setSpeed(15);                         //set rpm
  Motor_Roll.setSpeed(15);                         //set rpm
  Serial.begin(115200);
//  Serial.println("------------------------Start------------------------");
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  Wire.begin(ID);                             //begin I2C
  Wire.onReceive(i2c_handler);
  
  motor_calib();
  laser_ctrl(1);delay(200);
  laser_ctrl(0);delay(200);
  laser_ctrl(1);delay(200);
  laser_ctrl(0);
//  motor_ctrl(0);

  wdt_enable(WDTO_4S);
}

void loop()
{
  wdt_reset();
  delay(100);
//  LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_ON);

  if (cmd.event) 
  {
    cmd.event = false;
    
    i2c_decode();
    wdt_reset();delay(100);
    i2c_action();
    
    wdt_reset();delay(100);
  }
  else
  {
    if (cmd.laser == true) {
      laser_ctrl(1);
    } else {
      laser_ctrl(0);
    }
    if (cmd.motor == true) {
      motor_ctrl(1);
    } else {
      motor_ctrl(0);
    }
  }
}

void motor_calib() {
//  Serial.println("Calibrating");
//  while (digitalRead(endZ) != LOW) {
//    Motor_Roll.step(-1);
//  }
//  while (digitalRead(endX) != LOW) {
//    Motor_Pitch.step(-1);
//  }
  while (((digitalRead(endZ) != LOW) || (digitalRead(endX) != LOW))) {
    if (digitalRead(endZ) != LOW) {
      Motor_Roll.step(-1);
    }
    if (digitalRead(endX) != LOW) {
      Motor_Pitch.step(-1);
    }
    delay(1);
  }
  delay(100);

  int x_calib_step = (int)(x_cal / 10 / steps);
  int z_calib_step = (int)(z_cal / 10 / steps);

//  while (x_calib_step != 0) {
//    x_calib_step--;
//    Motor_Pitch.step(1);
//  }
//  while (z_calib_step != 0) {
//    z_calib_step--;
//    Motor_Roll.step(1);
//  }
  
  while (((x_calib_step != 0) || (z_calib_step != 0))) {
    if (x_calib_step != 0) {
      x_calib_step--;
      Motor_Pitch.step(1);
    }
    if (z_calib_step != 0) {
      z_calib_step--;
      Motor_Roll.step(1);
    }
    delay(1);
  }
  delay(100);
  
//  Serial.println("Calibration Completed!\n");

  roll_prev = 0;
  pitch_prev = 0;
}

void laser_ctrl(int dot) {
  if (dot == 0) {
    digitalWrite(LASER, LOW);
  }
  else if (dot == 1) {
    digitalWrite(LASER, HIGH);
  }
}

void motor_ctrl(int dot) {
  if (dot == 0) {
    digitalWrite(MOTOR, LOW);
    for (int i = 4; i < 12; i++) {
      digitalWrite(i, LOW);
    }
  } else if (dot == 1) {
    digitalWrite(MOTOR, HIGH);
  }
}

void get_angle(double a, double b) {              //a,b is point coordinate

  motor_roll = degrees(atan2(b + h_offset, y));
  motor_pitch = degrees(atan2(a + w_offset, y));

  calc.yaw = motor_roll;                      //save motor.yaw value to structure calc.yaw
  calc.pitch = motor_pitch;                    //save motor.pitch value to structure calc.pitch
  return;
}

void i2c_action() {
//    motor_ctrl(1);

    if (cmd.hw_reset == true) {
      cmd.hw_reset = false;
      wdt_enable(WDTO_60MS);
      while(1);
    } else if ( (cmd.move_x != complete.move_x) ||
       (cmd.move_y != complete.move_y) ||
       (cmd.offset_x != complete.offset_x) ||
       (cmd.offset_y != complete.offset_y) ) {
      laser_ctrl(0);
      motor_calib();
      
      get_angle((double)cmd.move_x/10 + (double)cmd.offset_x/10, (double)cmd.move_y/10 + (double)cmd.offset_y/10);                          //angle calculation fuction
      step_roll = (int)(motor_roll / steps);    //convert angle to steps
      step_pitch = (int)(motor_pitch / steps);

      int targetRoll = step_roll - roll_prev;
      int targetPitch = step_pitch - pitch_prev;
      while ((targetRoll | targetPitch)) {
        if (targetRoll != 0) {
          if (targetRoll < 0) {
            Motor_Roll.step(-1);
            targetRoll++;
          } else {
            Motor_Roll.step(1);        
            targetRoll--;
          }
        }
        if (targetPitch != 0) {
          if (targetPitch < 0) {
            Motor_Pitch.step(-1); 
            targetPitch++;
          } else {
            Motor_Pitch.step(1);    
            targetPitch--;
          }
        }
        delay(1);
      }
      delay(100);

      // laser_ctrl(cmd.laser ? 1:0);
      memcpy(&complete, &cmd, sizeof(i2c_cmd_t));

    } else if ( (cmd.tick_x != 0) ||
                (cmd.tick_y != 0) ) {
      while ((cmd.tick_x | cmd.tick_y) ) {
        if (cmd.tick_x < 0) {
          Motor_Pitch.step(-1);
          cmd.tick_x++;
        } else if (cmd.tick_x > 0) {
          Motor_Pitch.step(1);
          cmd.tick_x--;
        }
        if (cmd.tick_y < 0) {
          Motor_Roll.step(-1);
          cmd.tick_y++;
        } else if (cmd.tick_y > 0) {
          Motor_Roll.step(1);
          cmd.tick_y--;
        }
        delay(1);
      }
    }
//    motor_ctrl(0);

    if (cmd.motor == true) {
      motor_ctrl(1);
    } else {
      motor_ctrl(0);
    }
    
    if (cmd.laser == true) {
      laser_ctrl(1);
    } else {
      laser_ctrl(0);
    }
}

void i2c_decode() {
//  Serial.print("i2c_reg:");
//  for (int i = 0; i < I2C_REG_NUM; i++) {
//    Serial.print(i2c_reg[i], 16);
//    Serial.print(" ");
//  }
//  Serial.println();
  if (i2c_reg[CTRL] == 4) // reset
  {
    cmd.hw_reset = true;
  }
  else if (i2c_reg[CTRL] == 2) // motor on
  {
    cmd.motor = true;
  }
  else if (i2c_reg[CTRL] == 1) // laser on
  {
    cmd.laser = true;
  } 
  else if (i2c_reg[CTRL] == 0) // laser off
  {
    cmd.laser = false;
    cmd.motor = false;
  }

  cmd.move_x = i2c_reg[MOVEX_H] * 256 + i2c_reg[MOVEX_L];
  cmd.move_y = i2c_reg[MOVEY_H] * 256 + i2c_reg[MOVEY_L];
  cmd.offset_x = i2c_reg[OFFSETX_H] * 256 + i2c_reg[OFFSETX_L];
  cmd.offset_y = i2c_reg[OFFSETY_H] * 256 + i2c_reg[OFFSETY_L];

  // init data
  x_cal = (double)(i2c_reg[XCAL_H] * 256 + i2c_reg[XCAL_L]);
  z_cal = (double)(i2c_reg[ZCAL_H] * 256 + i2c_reg[ZCAL_L]);
  y = (double)(i2c_reg[YDIST_H] * 256 + i2c_reg[YDIST_L]) / 10;

  cmd.tick_x = (char)(i2c_reg[TICK_X]);
  i2c_reg[TICK_X] = 0;
  cmd.tick_y = (char)(i2c_reg[TICK_Y]);
  i2c_reg[TICK_Y] = 0;
}

void i2c_handler(int length) {
  // int length = Wire.available();
  char reg = Wire.read();
  length--;

  for (char i = reg; i < (reg + length); i++)
  {
    i2c_reg[i] = Wire.read();
  }

  cmd.event = true;
}
