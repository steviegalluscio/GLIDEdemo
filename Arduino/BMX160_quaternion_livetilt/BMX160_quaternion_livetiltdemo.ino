#include <Wire.h>
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor v1.1.14 by Adafruit
#include <Mahony_BMX160.h>
//#include <Madgwick_BMX160.h>
#include <DPEng_BMX160.h> // source: https://github.com/drcpattison/BMX160

/* Note on linux may need to run:
 sudo usermod -a -G dialout yourname 
 sudo tty -a -G dialout yourname 
 or
 sudo chmod 777 /dev/ttyUSB0 */

// Create sensor instance.
DPEng_BMX160 dpEng = DPEng_BMX160(0x160A, 0x160B, 0x160C);

// Mag calibration values are calculated via ahrs_calibration example sketch results.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 118.73F, -79.59F, 44.67F };

float mag_softiron_matrix[3][3] = { {  0.853,  -0.388,  0.300 },
                                    {  -0.388,  0.535, -0.219 },
                                    {  0.300,  -0.219,  3.390 } };

float mag_field_strength        = 63.19F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
Mahony_BMX160 filter;
//Madgwick_BMX160 filter;

void setup()
{
  Serial.begin(115200);

  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
  // while(!Serial);

  Serial.println(F("DPEng AHRS Fusion Example")); Serial.println("");

  // Initialize the sensors.
  if(!dpEng.begin(BMX160_ACCELRANGE_4G, GYRO_RANGE_250DPS))
  {
    /* There was a problem detecting the BMX160 ... check your connections */
    Serial.println("Ooops, no BMX160 detected ... Check your wiring!");
    while(1);
  }
  
  filter.begin();
}

void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event;

  // Get new data samples
  dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz,
				mag_event.timestamp);

  float w_,x_,y_,z_;
  

  filter.getQuaternion(&w_,&x_,&y_,&z_);
  

  byte* wPtr = (byte*) &w_; 
  byte* xPtr = (byte*) &x_;
  byte* yPtr = (byte*) &y_;
  byte* zPtr = (byte*) &z_;

  Serial.write(wPtr, sizeof(w_));
  Serial.write(xPtr, sizeof(x_));
  Serial.write(yPtr, sizeof(y_));
  Serial.write(zPtr, sizeof(z_));

  delay(10);
}
