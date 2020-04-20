/* Adafruit 9dof Accel & Gyro
 * VIN  -->   5 V
 * GND  -->   GND
 * SCL  -->   A5
 * SDA  -->   A4
*/
#include <Wire.h>

// ===== GYRO =====
const int L3GD20_address = 0x6B; // Gyro sensor address - 0b01101011
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
float pitch_gyro_rate = 0, roll_gyro_rate = 0, yaw_gyro_rate = 0;
long gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

// ===== ACCEL =====
const int LSM303_address = 0x19; // Accel sensor address - or 0x32 >> 1;
int16_t acc_x_raw, acc_y_raw, acc_z_raw;
double vector_acc_total;
float pitch_acc = 0, roll_acc = 0;
float acc_x, acc_y, acc_z, accx_bias, accy_bias, accz_bias;

// ===== MAG =====
const int LSM303_mag_address = 0x3C >> 1;
int16_t mag_x_raw, mag_y_raw, mag_z_raw;
float mag[3], mag_y, mag_x, mag_x_offset, mag_y_offset, mag_z_offset, mag_norm;
float orientation, headingDegrees, declination;

// ===== SONSTIGES =====
unsigned long loop_timer, print_loop, time_last_loop = 5300;

// ===== KALMAN =====
// gyro noise density = dps/sqrt(Hz) ,dps i.e. 250 0.001 / sqrt(190) = 0.018
float pitch = 0, roll = 0, head = 0;
float q = 0.02, r = 10.441;
int   a = 1;
// for pitch
float x_hat_pitch = 0, p_pitch = 0, k_pitch = 0 ,dt;
// for roll
float x_hat_roll = 0, p_roll = 0, k_roll = 0;
// for heading
float x_hat_head = 0, p_head = 0, k_head = 0;
float q_head = 0.02, r_head = 0.7148;


void setup() {
  Serial.begin(115200);
  initialize(); // init magnetometer, accelerometer and gyro
  delay(1000);
  calibrate();  // calculate the gyros offset
  loop_timer = micros();
  print_loop = micros();
}

void loop() {
  getData();
  // ===== Gyro rate calculations =====
  gyro_x_raw -= gyro_x_offset;
  gyro_y_raw -= gyro_y_offset;
  gyro_z_raw -= gyro_z_offset;

  // ===== Acceleration calculations =====
  // acc = raw_value * mg/lsb/1000 * standard_earth_gravity
  accx_bias = acc_x_raw + 32.853954; 
  accy_bias = acc_y_raw + 11.928418;
  accz_bias = acc_z_raw - 18.129956;
  
  acc_x = (0.966771 * accx_bias + 0.000634 * accy_bias - 0.000244 * accz_bias) / 100; 
  acc_y = (0.000634 * accx_bias + 0.938452 * accy_bias - 0.000220 * accz_bias) / 100;
  acc_z = (-0.000244 * accx_bias - 0.000220 * accy_bias + 0.960528 * accz_bias) / 100;
  
  // ===== Gyro rate calculations =====
  // (LBSmdg/s/1000)/update_rate = (8.75mdg/s/1000)/190
  /*  Sensitivity   specifications    Datasheet page 9, table 3. Mechanical characteristics
   *  250  DPS      8.75 mDPS/digit
   *  500  DPS      17.5 mDPS/digit
   *  2000 DPS      70.0 mDPS/digit
   */
  roll_gyro_rate = gyro_x_raw * (0.0175);
  pitch_gyro_rate = -gyro_y_raw * (0.0175);
  yaw_gyro_rate = gyro_y_raw * (0.0175);

  // ===== Accelerometer angle calculations =====
  vector_acc_total = sqrt(pow(acc_x_raw,2) + pow(acc_y_raw,2) + pow(acc_z_raw,2));
  roll_acc = asin(acc_y_raw / vector_acc_total) * 180 / PI;
  pitch_acc = -asin(acc_x_raw / vector_acc_total) * -180 / PI;

  // ===== Kalman-Filter for pitch & roll =====
  pitch = kalmanCalculate_pitch(pitch_acc, pitch_gyro_rate, time_last_loop);
  roll = kalmanCalculate_roll(roll_acc, roll_gyro_rate, time_last_loop);

  // ===== Heading angle calculations =====
  calcHeading();
  head = kalmanCalculate_head(headingDegrees, yaw_gyro_rate, time_last_loop);
  
  // ===== Print for Python with 20 Hz =====
  if(micros() - print_loop >= 50000){   // 20 Hz = 50000
    Serial.print(pitch); Serial.print(",");
    Serial.print(roll);  Serial.print(",");
    Serial.println(head);

    print_loop = micros();
  }
  
  // ===== Set loop timer to 5.3 ms cause sensors update rate of of 190 Hz =====
  while(micros() - loop_timer < 5300);    // 190 Hz ~ 5300
  time_last_loop = micros() - loop_timer;
  loop_timer = micros();
}

// ==========================================================
// ========== FUNCTIONS =====================================
// ==========================================================

// ========== KALMAN CALCULATION ==========
float kalmanCalculate_pitch(float newAngle, float newRate,int looptime){
    dt = float(looptime) * 0.000001; // dt = 1/Hz
    // prediction
    x_hat_pitch = a * x_hat_pitch + dt * newRate;
    p_pitch = p_pitch + q;
    // update
    k_pitch = p_pitch * (1.0 / (p_pitch + r));
    x_hat_pitch = x_hat_pitch + (k_pitch * (newAngle - x_hat_pitch));
    p_pitch = (1 - k_pitch) * p_pitch;
    return x_hat_pitch;
}
float kalmanCalculate_roll(float newAngle, float newRate,int looptime){
    dt = float(looptime) * 0.000001; // dt = 1/Hz
    // prediction
    x_hat_roll = a * x_hat_roll + dt * newRate;
    p_roll = p_roll + q;
    // update
    k_roll = p_roll * (1.0 / (p_roll + r));
    x_hat_roll = x_hat_roll + (k_roll * (newAngle - x_hat_roll));
    p_roll = (1 - k_roll) * p_roll;
    return x_hat_roll;
}
float kalmanCalculate_head(float newAngle, float newRate,int looptime){
    if((newAngle <= 3) || (newAngle >= 357)) x_hat_head = newAngle;
    else{
    dt = float(looptime) * 0.000001; // dt = 1/Hz
    // prediction
    x_hat_head = a * x_hat_head + dt * newRate;
    if(x_hat_head > 360) x_hat_head -= 360;
    if(x_hat_head < 0) x_hat_head += 360;
    p_head = p_head + q_head;
    // update
    k_head = p_head * (1.0 / (p_head + r_head));
    x_hat_head = x_hat_head + (k_head * (newAngle - x_hat_head));
    p_head = (1 - k_head) * p_head;
    }
    return x_hat_head;
}
// ========== CALIBRATION ==========
void calibrate(){
  /*
  int cal_i;
  for(cal_i = 0; cal_i < 1000; cal_i++){
    getData();
    gyro_x_offset += gyro_x_raw;
    gyro_y_offset += gyro_y_raw;
    gyro_z_offset += gyro_z_raw;
    delay(500);
    Serial.println(cal_i);
  }
  gyro_x_offset /= 1000;
  gyro_y_offset /= 1000;
  gyro_z_offset /= 1000;
  */
  gyro_x_offset = 88;
  gyro_y_offset = 152;
  gyro_z_offset = 31;
}
// ========== CALC HEADING ==========
void calcHeading(){
  // CALIBRATION
  // Calibration Software Magneto v1.2 (https://sites.google.com/site/sailboatinstruments1/home)
  // Code by Chris Holm Jul 25, 2018 (https://forums.adafruit.com/viewtopic.php?f=8&t=136357&p=685932)
  // Offset (Werte in nT)
  mag_x_offset = mag_x_raw*(100000.0/1100.0) - 4702.561101;
  mag_y_offset = mag_y_raw*(100000.0/1100.0) - 3067.987428;
  mag_z_offset = mag_z_raw*(100000.0/980.0) - 13619.506938;  
  // correction matrix
  mag[0] = 0.996058 * mag_x_offset + 0.008493 * mag_y_offset - 0.016802 * mag_z_offset;
  mag[1] = 0.008493 * mag_x_offset + 1.122115 * mag_y_offset + 0.001001 * mag_z_offset;
  mag[2] = -0.016802 * mag_x_offset + 0.001001 * mag_y_offset + 0.883519 * mag_z_offset;
  // Normalice the magnetic field vector
  mag_norm = sqrt(pow(mag[0],2) + pow(mag[1],2) + pow(mag[2],2));
  mag[0] = mag[0] / mag_norm;
  mag[1] = mag[1] / mag_norm;
  mag[2] = mag[2] / mag_norm;
  // change y and z up and down
  mag[1] = -1.0 * mag[1];
  mag[2] = -1.0 * mag[2];
  // tilt compensation
  mag_x = mag[0] * cos(pitch * 0.017545) + mag[2] * sin(pitch * 0.01745);
  mag_y = mag[1] * cos(roll * 0.01745) - mag[2] * sin(roll * 0.01745);
  
  orientation = atan2(-mag_y, mag_x); // tilt orientation
  // Declination from: https://www.ngdc.noaa.gov/geomag-web/#igrfwmm
  declination = 0.06736;
  orientation += declination;
  if(orientation <0) orientation += 2*PI;
  if(orientation > 2*PI)orientation -= 2*PI;
  // calc heading in degrees
  headingDegrees = orientation * 180/PI;
}

// ========== GET DATA FROM GYRO AND ACCEL ==========
void getData(){
  // ===== DATA MAG  =====
  // =====================
  Wire.beginTransmission(LSM303_mag_address);
  Wire.write(0x03);
  Wire.endTransmission(true);
  Wire.requestFrom(LSM303_mag_address, 6);
  while(Wire.available() < 6);
  // reading data, high befor low
  mag_x_raw = int16_t(Wire.read()<<8 | Wire.read());
  mag_z_raw = int16_t(Wire.read()<<8 | Wire.read());
  mag_y_raw = int16_t(Wire.read()<<8 | Wire.read());
  
  // ===== DATA GYRO =====
  // =====================
  Wire.beginTransmission(L3GD20_address);
  Wire.write(0x28 | 0x80);  //register OUT_X_L
  Wire.endTransmission(true);
  Wire.requestFrom(L3GD20_address,6);
  while(Wire.available()<6);
  gyro_x_raw = (int16_t)(Wire.read() | (Wire.read() << 8));
  gyro_y_raw = (int16_t)(Wire.read() | (Wire.read() << 8));
  gyro_z_raw = (int16_t)(Wire.read() | (Wire.read() << 8));

  // ===== DATA ACCEL =====
  // ======================
  Wire.beginTransmission(LSM303_address);
  Wire.write(0x28 | 0x80);
  Wire.endTransmission(true);
  Wire.requestFrom(LSM303_address,6);
  while(Wire.available()<6);
  acc_x_raw = (int16_t)((uint16_t)Wire.read() | ((uint16_t)Wire.read() << 8)) >> 4;
  acc_y_raw = (int16_t)((uint16_t)Wire.read() | ((uint16_t)Wire.read() << 8)) >> 4;
  acc_z_raw = (int16_t)((uint16_t)Wire.read() | ((uint16_t)Wire.read() << 8)) >> 4;
}

// ========== INITIALIZE MPU AND ACCEL ==========
void initialize(){
  Wire.begin();
  // ===== INIT MAG ======
  // =====================
  Wire.beginTransmission(LSM303_mag_address);
  Wire.write(0x02); // Register MR_REG_M
  Wire.write(0x00); // wake up an set to Continuous-conversation mode (datasheet page 38, tab 78.)
  Wire.endTransmission(true);
  // set magnetic gain
  Wire.beginTransmission(LSM303_mag_address);
  Wire.write(0x01); // Register CRB_REG_M
  Wire.write(0x20); // set magnetic gain
  Wire.endTransmission(true);
  /*  Settings magnetic gain
   *  HEX   BIN         Field range [Gauss]   Gain X,Y [LSB/Gauss]    Gain Z [LSB/Gauss]  Output range
   *  0x20  0b00100000  +/- 1.3               1100                    980                 -2048 - 2047
   *  0x40  0b01000000  +/- 1.9               855                     760                 -2048 - 2047
   *  0x60  0b01100000  +/- 2.5               670                     600                 -2048 - 2047
   *  0x80  0b10000000  +/- 4.0               450                     400                 -2048 - 2047
   *  0xA0  0b10100000  +/- 4.7               400                     355                 -2048 - 2047
   *  0xC0  0b11000000  +/- 5.6               330                     295                 -2048 - 2047
   *  0xE0  0b11100000  +/- 8.1               230                     205                 -2048 - 2047
  */
  // set magnetic update rate
  Wire.beginTransmission(LSM303_mag_address);
  Wire.write(0x00); // Register CRA_REG_M
  Wire.write(0x1C); // set magnetic update rate
  Wire.endTransmission(true);
  /*  Setting magnetic update rate
   *  HEX   BIN         Output rate [Hz]
   *  0x00  0b00000000  0.75
   *  0x04  0b00000100  1.5
   *  0x08  0b00001000  3.0
   *  0x0C  0b00001100  7.5
   *  0x10  0b00010000  15.0
   *  0x14  0b00010100  30.0
   *  0x18  0b00011000  75.0
   *  0x1C  0b00011100  220.0
  */  
  // ===== INIT GYRO =====
  // =====================
  // WAKE UP
  Wire.beginTransmission(L3GD20_address);
  Wire.write(0x20); // CTRL_REG1 (0x20)
  Wire.write(0x00); // reset register to normal mode (sleep mode, disable all channel)
  Wire.endTransmission(true);
  // SET DATA OUTPUT RATE
  Wire.beginTransmission(L3GD20_address);
  Wire.write(0x20); // CTRL_REG1 (0x20)
  Wire.write(0x4F); // enable normal mode, data output rate and x, y, z channel 
  Wire.endTransmission(true);
  /*
   *  Data output rate:     datasheet page 32, table 20. DR and BW configuration setting
   *  0x0F  0b00001111    95  Hz
   *  0x4F  0b01001111   190  Hz
   *  0x8F  0b10001111   380  Hz
   *  0xCF  0b11001111   760  Hz
  */
  // SET DATA RANGE CONFIGURATION
  Wire.beginTransmission(L3GD20_address);
  Wire.write(0x23); // CTRL_REG4 (0x23)
  Wire.write(0x10); // Data range configuration
  Wire.endTransmission(true);
  /*
   *  Data range configuration:
   *  0x00  0b00000000   250  DPS
   *  0x10  0b00010000   500  DPS
   *  0x20  0b00100000  2000  DPS
  */
  // ===== INIT ACCEL =====
  // ======================
  // WAKE UP AND SET DATA RATE CONFIGURATION
  Wire.beginTransmission(LSM303_address);
  Wire.write(0x20); // CTRL_REG1_A (0x20)
  Wire.write(0x67); // Data rate configuration
  Wire.endTransmission(true);
  /*
   * Data rate configuration:
   *  0x57  0b01010111  100 Hz
   *  0x67  0b01100111  200 Hz
   *  0x77  0b01110111  400 Hz
  */
  // SET FULL SCALE SELECTION
  Wire.beginTransmission(LSM303_address);
  Wire.write(0x23); // CTRL_REG4_A (0x23)
  Wire.write(0x00); // Full-scale selection
  Wire.endTransmission(true);
  /*
   * Full-scale selection
   *  HEX   BIN         FULL-SCALE    mg/LSB
   *  0x00  0b00000000  +/- 2   g     1
   *  0x10  0b00010000  +/- 4   g     2
   *  0x20  0b00100000  +/- 8   g     4
   *  0x30  0b00110000  +/- 16  g     12
  */
}
