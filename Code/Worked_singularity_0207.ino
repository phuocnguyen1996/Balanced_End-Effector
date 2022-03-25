

#include <MPU9250_WE.h>
#include <Wire.h>
#include <Servo.h>
#define MPU9250_ADDR 0x68


MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;

float pi = 3.1415926536;
int response_time = 15;
int singularity_flag = 1;


float rotational_x = 0;
float rotational_y = 0;
float rotational_z = 0;
float axis[3] = {0, 0, 0};
float rotational_speed = 0;
float rotation_angle = 0;
float rotation_matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float orientation_matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float transpose_orientation_matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float R04_matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float target[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float q1 = 0;
float q2 = pi/18;
float q3 = 0;
Servo servo1;
Servo servo2;
Servo servo3;


int calib_time = 500;

int dt = 0;
unsigned long Time = 0;
float roll_angle = 0;
float pitch_angle = 0;
float yaw_angle = 0;

int flag = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

   Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");


  myMPU9250.enableGyrDLPF();
  //myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_8800); // bandwdith without DLPF
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);


  myMPU9250.setSampleRateDivider(5);

   
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  myMPU9250.enableAccDLPF(true);

  myMPU9250.setAccDLPF(MPU9250_DLPF_6);

  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);

  servo1.attach(10); //attach it to pin 9
  servo2.attach(11);
  servo3.attach(9);
  servo1.write(90);
  servo2.write(100); //100
  servo3.write(90);
  delay(500);
  create_R04_matrix(R04_matrix, q1, q2, q3);
  //calibration();
  Time = millis();
}

void loop() {
  //xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  rotational_x = gyr.x - gyro_x_offset;
  rotational_y = gyr.y - gyro_y_offset;
  rotational_z = gyr.z - gyro_z_offset;
  rotational_speed = sqrt(rotational_x*rotational_x + rotational_y*rotational_y + rotational_z*rotational_z);


  if ((rotational_x < 5) and (rotational_x > -5))
    rotational_x = 0;
  if ((rotational_y < 5) and (rotational_y > -5))
    rotational_y = 0;
  if ((rotational_z < 5) and (rotational_z > -5))
    rotational_z = 0;

  if (rotational_speed == 0)
  {
    axis[0] = 0;
    axis[1] = 0;
    axis[2] = 0;
  }
  else
  {
    axis[0] = rotational_x / rotational_speed;
    axis[1] = rotational_y / rotational_speed;
    axis[2] = rotational_z / rotational_speed;
  }

  dt = millis() - Time;

  rotation_angle = rotational_speed * dt*1e-3;

  roll_angle += rotational_x*dt*1e-3;
  pitch_angle += rotational_y*dt*1e-3;
  yaw_angle += rotational_z*dt*1e-3;

  create_rotation_matrix(rotation_matrix, axis, rotation_angle);
  multiply_matrices(orientation_matrix, rotation_matrix);
  transpose_a_matrix(orientation_matrix, transpose_orientation_matrix);
  create_target_matrix(target, R04_matrix, transpose_orientation_matrix);



    if (singularity_flag == 1)
    {
      q1 = atan2(target[1][0], target[0][0]);
      q2 = atan2(sqrt(target[1][0]*target[1][0] + target[0][0]*target[0][0]), -target[2][0]);
      q3 = atan2(target[2][2], target[2][1]);
    }

    if (singularity_flag == 2)
    {
      q1 = atan2(-target[1][0], -target[0][0]);
      q2 = atan2(-sqrt(target[1][0]*target[1][0] + target[0][0]*target[0][0]), -target[2][0]);
      q3 = atan2(-target[2][2], -target[2][1]);
    }

    if ((q1 > -0.01) and (q1 < 0.01))
      q1 = 0;
    if ((q3 > -0.01) and (q3 < 0.01))
      q3 = 0;
    if (q1 > 3.14)
      q1 = 3.14;
    if (q1 < -3.14)
      q1 = -3.14;
    if (q3 > 3.14)
      q3 = 3.14;
    if (q3 < -3.14)
      q3 = -3.14;

  int Servo1_value = q1*180/pi + 90;
  int Servo2_value = q2*180/pi + 90;
  int Servo3_value = q3*180/pi + 90;


  if ((Servo2_value > 90) and (Servo2_value < 93))
  {
    singularity_flag = 2;
    q1 = atan2(-target[1][0], -target[0][0]);
    q2 = atan2(-sqrt(target[1][0]*target[1][0] + target[0][0]*target[0][0]), -target[2][0]);
    q3 = atan2(-target[2][2], -target[2][1]);
    Servo1_value = q1*180/pi + 90;
    Servo2_value = 86;
    Servo3_value = q3*180/pi + 90;
    servo1.write(Servo1_value);
    servo2.write(Servo2_value);
    servo3.write(Servo3_value);
    
  }

  else if ((Servo2_value < 90) and (Servo2_value > 87))

  {
    singularity_flag = 1;
    q1 = atan2(target[1][0], target[0][0]);
    q2 = atan2(sqrt(target[1][0]*target[1][0] + target[0][0]*target[0][0]), -target[2][0]);
    q3 = atan2(target[2][2], target[2][1]);
    Servo1_value = q1*180/pi + 90;
    Servo2_value = 94;
    Servo3_value = q3*180/pi + 90;
    servo1.write(Servo1_value);
    servo2.write(Servo2_value);
    servo3.write(Servo3_value);
    
  }
  else
  {
    servo1.write(Servo1_value);
    servo2.write(Servo2_value);
    servo3.write(Servo3_value);
  }
  

  delay(response_time);

  create_R04_matrix(R04_matrix, q1, q2 ,q3);
  
//  rotation_matrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  if (flag == 1000)
  {
//    Serial.println("Gyroscope data in degrees/s: ");
//    Serial.print(rotational_x);
//    Serial.print("   ");
//    Serial.print(rotational_y);
//    Serial.print("   ");
//    Serial.println(rotational_z);
//    /*Serial.print("Roll: ");
//    Serial.println(roll_angle);
//    Serial.print("Pitch: ");
//    Serial.println(pitch_angle);
//    Serial.print("Yaw: ");
//    Serial.println(yaw_angle);
//    Serial.print("Axis: ");
//    Serial.print(axis[0]);
//    Serial.print("  ");
//    Serial.print(axis[1]);
//    Serial.print("  ");
//    Serial.println(axis[2]);
//    Serial.println();*/
//    //Serial.println(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
//    //Serial.println("Rotation matrix");
//    //print_matrix(rotation_matrix);
//    Serial.println("Orientation matrix");
//    print_matrix(orientation_matrix);
//    Serial.println("Transpose orientation matrix");
//    print_matrix(transpose_orientation_matrix);
//    Serial.println("R04: ");
//    print_matrix(R04_matrix);
//    Serial.println("Target: ");
//    print_matrix(target);
//    //Serial.println(target[0][0]);
//    Serial.print("q1: ");
//    Serial.print(q1);
//    Serial.print("    q2: ");
//    Serial.print(q2);
//    Serial.print("    q3: ");
//    Serial.println(q3);
//    Serial.print("Servo1: ");
//    Serial.print(Servo1_value);
//    Serial.print("    Servo2: ");
//    Serial.print(Servo2_value);
//    Serial.print("    Servo3: ");
//    Serial.println(Servo3_value);       
//    Serial.println("********************************************");
//
//    reset_orientation(orientation_matrix);
//    
    flag = 0;
  }
  else
   flag++;


  Time = millis();
  


//  delay(1000);
}

void calibration()
{
  xyzFloat gyro_offset;
  Serial.println("Calibration time: ");
  for (int i = 1; i <= calib_time; i++)
  {
    gyro_offset = myMPU9250.getGyrValues();
    gyro_x_offset += gyro_offset.x;
    gyro_y_offset += gyro_offset.y;
    gyro_z_offset += gyro_offset.z;
    delay(10);
  }
  gyro_x_offset /= calib_time;
  gyro_y_offset /= calib_time;
  gyro_z_offset /= calib_time;
  
}

void create_rotation_matrix(float matrix[3][3], float axis[3], float rotation_angle)
{
  float angle_in_rad = rotation_angle*0.0174532925;
  matrix[0][0] = axis[0]*axis[0]*(1-cos(angle_in_rad)) + cos(angle_in_rad);
  matrix[0][1] = axis[0]*axis[1]*(1-cos(angle_in_rad)) - axis[2]*sin(angle_in_rad);
  matrix[0][2] = axis[0]*axis[2]*(1-cos(angle_in_rad)) + axis[1]*sin(angle_in_rad);
  matrix[1][0] = axis[0]*axis[1]*(1-cos(angle_in_rad)) + axis[2]*sin(angle_in_rad);
  matrix[1][1] = axis[1]*axis[1]*(1-cos(angle_in_rad)) + cos(angle_in_rad);
  matrix[1][2] = axis[1]*axis[2]*(1-cos(angle_in_rad)) - axis[0]*sin(angle_in_rad);
  matrix[2][0] = axis[0]*axis[2]*(1-cos(angle_in_rad)) - axis[1]*sin(angle_in_rad);
  matrix[2][1] = axis[1]*axis[2]*(1-cos(angle_in_rad)) + axis[0]*sin(angle_in_rad);
  matrix[2][2] = axis[2]*axis[2]*(1-cos(angle_in_rad)) + cos(angle_in_rad);
}

void create_R04_matrix(float matrix[3][3], float q1, float q2, float q3)
{
  matrix[0][0] = cos(q1)*sin(q2);
  matrix[0][1] = cos(q1)*cos(q2)*cos(q3) - sin(q1)*sin(q3);
  matrix[0][2] = cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3);
  matrix[1][0] = sin(q1)*sin(q2);
  matrix[1][1] = cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1);
  matrix[1][2] = cos(q2)*sin(q1)*sin(q3) - cos(q1)*cos(q3);
  matrix[2][0] = -cos(q2);
  matrix[2][1] = cos(q3)*sin(q2);
  matrix[2][2] = sin(q2)*sin(q3);
}

void print_matrix(float matrix[3][3])
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      Serial.print(matrix[i][j]);
      Serial.print("  ");
    }
    Serial.println();
  } 
}

void multiply_matrices(float pose[3][3], float rotation_matrix[3][3])
{
  float result[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      for (int k = 0; k < 3; k++)
        result[i][j] += pose[i][k] * rotation_matrix[k][j];
    }
  }
  pose[0][0] = result[0][0];
  pose[0][1] = result[0][1];
  pose[0][2] = result[0][2];
  pose[1][0] = result[1][0];
  pose[1][1] = result[1][1];
  pose[1][2] = result[1][2];
  pose[2][0] = result[2][0];
  pose[2][1] = result[2][1];
  pose[2][2] = result[2][2];
}



void transpose_a_matrix(float source_matrix[3][3], float transposed_matrix[3][3])
{
  transposed_matrix[0][0] = source_matrix[0][0];
  transposed_matrix[0][1] = source_matrix[1][0];
  transposed_matrix[0][2] = source_matrix[2][0];
  transposed_matrix[1][0] = source_matrix[0][1];
  transposed_matrix[1][1] = source_matrix[1][1];
  transposed_matrix[1][2] = source_matrix[2][1];
  transposed_matrix[2][0] = source_matrix[0][2];
  transposed_matrix[2][1] = source_matrix[1][2];
  transposed_matrix[2][2] = source_matrix[2][2];
}

void create_target_matrix(float target[3][3], float R04[3][3], float R4W[3][3])
{
  float result[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      for (int k = 0; k < 3; k++)
        result[i][j] += R04[i][k] * R4W[k][j];
    }
  }
  target[0][0] = result[0][0];
  target[0][1] = result[0][1];
  target[0][2] = result[0][2];
  target[1][0] = result[1][0];
  target[1][1] = result[1][1];
  target[1][2] = result[1][2];
  target[2][0] = result[2][0];
  target[2][1] = result[2][1];
  target[2][2] = result[2][2];
}

void reset_orientation(float matrix[3][3])
{
  matrix[0][0] = 1;
  matrix[0][1] = 0;
  matrix[0][2] = 0;
  matrix[1][0] = 0;
  matrix[1][1] = 1;
  matrix[1][2] = 0;
  matrix[2][0] = 0;
  matrix[2][1] = 0;
  matrix[2][2] = 1;
}
