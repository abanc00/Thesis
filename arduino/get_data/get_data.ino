#include <MPU9250.h>

MPU9250 mpu;

// Kalman filter variables for acceleration
float ay_est; // Estimated acceleration
float P_ay = 1; // Error covariance
float Q_a = 0.1; // Process noise for acceleration
float R_a = 0.1; // Measurement noise for acceleration
float K_ay; // Kalman gain

// Kalman filter variables for gyroscope
float gz_est; // Estimated gyro on z-axis
float P_gz = 1; // Error covariance
float Q_g = 0.1; // Process noise for gyro
float R_g = 0.1; // Measurement noise for gyro
float K_gz; // Kalman gain for gyro

// Velocity and position variables
float velocityY = 0;
float positionY = 0;
float angleZ = 0; // orientation in the 2D plane
float initialGyroZ;
unsigned long previousTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!mpu.setup(0x68)) {
    Serial.println("MPU setup failed");
    return;
  }

  // Initialize the estimates
  ay_est = 0;
  gz_est = 0;
  previousTime = millis();

  mpu.update();
  initialGyroZ = mpu.getGyroZ(); // Store initial gyro reading on z-axis
}

void loop() {
  mpu.update();

  // Read the current acceleration and gyro values
  float ay = mpu.getAccY();
  float gz = mpu.getGyroZ() - initialGyroZ; // Subtract initial value for relative rotation

  // Prediction step
  // Since we're not modeling any system dynamics, the predictions are just the previous estimate
  // The error covariance increases by process noise
  P_ay += Q_a;
  P_gz += Q_g;

  // Update step
  // Compute the Kalman gain
  K_ay = P_ay / (P_ay + R_a);
  K_gz = P_gz / (P_gz + R_g);

  // Update the estimates and error covariance
  ay_est += K_ay * (ay - ay_est);
  gz_est += K_gz * (gz - gz_est);
  P_ay *= (1 - K_ay);
  P_gz *= (1 - K_gz);

  // Update the velocities and positions
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // time in seconds

  // Update the angle using the gyro z-axis
  angleZ += gz_est * deltaTime;
  angleZ = fmod(angleZ, 360.0); // Keep angle in the range [0, 360)

  // Update the velocity and position using the y-axis acceleration
  velocityY += ay_est * deltaTime;
  positionY += velocityY * deltaTime;
  
  previousTime = currentTime;

  // Print values
  Serial.print("Position Y: ");
  Serial.print(positionY);
  Serial.print(" , Angle Z: ");
  Serial.println(angleZ);

  delay(100);
}
