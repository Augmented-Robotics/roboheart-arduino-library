/* This example shows how to calculate rotation
 * from the built-in IMU data.
 *
 * Created  15/10/2024
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>
#include <Wire.h>
#include <math.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#define CALIBRATION_COUNTDOWN 5
#define ACCELEROMETER_CUTOFF 32

//#define DEBUG

#define NEW_IMU_DATA_BIT 1
static EventGroupHandle_t xIMUeventGroup;
static SemaphoreHandle_t xVarMutex;

RoboHeart heart = RoboHeart(Serial);

float phi = 0.0, theta = 0.0, psi = 0.0;

static tIMUdata imuData = {0};

void computeEulerRates(float omega_x, float omega_y, float omega_z, float phi, float theta, float* dphi, float* dtheta, float* dpsi) {
    // Transformation matrix elements
    float cos_phi = cosf(phi);
    float sin_phi = sinf(phi);
    float cos_theta = cosf(theta);

    // Compute the rates of change of Euler angles
    *dphi = omega_x + (omega_y * sin_phi + omega_z * cos_phi) * tanf(theta);
    *dtheta = omega_y * cos_phi - omega_z * sin_phi;
    *dpsi = (omega_y * sin_phi + omega_z * cos_phi) / cos_theta;
}

void rungeKutta4(float* phi, float* theta, float* psi, float omega_x, float omega_y, float omega_z, float dt) {
    // Intermediate variables for Runge-Kutta
    float k1_phi, k2_phi, k3_phi, k4_phi;
    float k1_theta, k2_theta, k3_theta, k4_theta;
    float k1_psi, k2_psi, k3_psi, k4_psi;

    float phi_temp, theta_temp, psi_temp;
    float dphi, dtheta, dpsi;

    // k1 values
    computeEulerRates(omega_x, omega_y, omega_z, *phi, *theta, &dphi, &dtheta, &dpsi);
    k1_phi = dphi * dt;
    k1_theta = dtheta * dt;
    k1_psi = dpsi * dt;

    // k2 values
    phi_temp = *phi + k1_phi / 2;
    theta_temp = *theta + k1_theta / 2;
    psi_temp = *psi + k1_psi / 2;
    computeEulerRates(omega_x, omega_y, omega_z, phi_temp, theta_temp, &dphi, &dtheta, &dpsi);
    k2_phi = dphi * dt;
    k2_theta = dtheta * dt;
    k2_psi = dpsi * dt;

    // k3 values
    phi_temp = *phi + k2_phi / 2;
    theta_temp = *theta + k2_theta / 2;
    psi_temp = *psi + k2_psi / 2;
    computeEulerRates(omega_x, omega_y, omega_z, phi_temp, theta_temp, &dphi, &dtheta, &dpsi);
    k3_phi = dphi * dt;
    k3_theta = dtheta * dt;
    k3_psi = dpsi * dt;

    // k4 values
    phi_temp = *phi + k3_phi;
    theta_temp = *theta + k3_theta;
    psi_temp = *psi + k3_psi;
    computeEulerRates(omega_x, omega_y, omega_z, phi_temp, theta_temp, &dphi, &dtheta, &dpsi);
    k4_phi = dphi * dt;
    k4_theta = dtheta * dt;
    k4_psi = dpsi * dt;

    // Update the Euler angles
    *phi += (k1_phi + 2 * k2_phi + 2 * k3_phi + k4_phi) / 6.0f;
    *theta += (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta) / 6.0f;
    *psi += (k1_psi + 2 * k2_psi + 2 * k3_psi + k4_psi) / 6.0f;
}

void rungeKutta2(float* phi, float* theta, float* psi, float omega_x, float omega_y, float omega_z, float dt) {
    // Intermediate variables for Runge-Kutta
    float k1_phi, k2_phi;
    float k1_theta, k2_theta;
    float k1_psi, k2_psi;

    float phi_temp, theta_temp, psi_temp;
    float dphi, dtheta, dpsi;

    // k1 values (slopes at the beginning of the interval)
    computeEulerRates(omega_x, omega_y, omega_z, *phi, *theta, &dphi, &dtheta, &dpsi);
    k1_phi = dphi * dt;
    k1_theta = dtheta * dt;
    k1_psi = dpsi * dt;

    // Intermediate angles using k1 (midpoint approximation)
    phi_temp = *phi + k1_phi / 2;
    theta_temp = *theta + k1_theta / 2;
    psi_temp = *psi + k1_psi / 2;

    // k2 values (slopes at the midpoint)
    computeEulerRates(omega_x, omega_y, omega_z, phi_temp, theta_temp, &dphi, &dtheta, &dpsi);
    k2_phi = dphi * dt;
    k2_theta = dtheta * dt;
    k2_psi = dpsi * dt;

    // Update the Euler angles using the second-order RK method
    *phi += k2_phi;
    *theta += k2_theta;
    *psi += k2_psi;
}

void IMUgetDataTask(void *pvParameter) {
  uint8_t status = 0;
  tIMUdata data = {0};

  while(true) {
    heart.imu.readRegister(&status, LSM6DS3_ACC_GYRO_STATUS_REG);
    if (status & LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL) {
      heart.imu.readRegisterRegion((uint8_t *) &data, LSM6DS3_ACC_GYRO_OUTX_L_G, 12);
      if (xSemaphoreTake(xVarMutex, portMAX_DELAY)) {
        imuData = data;
        xSemaphoreGive(xVarMutex);
      }
      xEventGroupSetBits(xIMUeventGroup, NEW_IMU_DATA_BIT);
    }
    vTaskDelay(((1000/heart.imu.settings.gyroSampleRate)-1)/portTICK_PERIOD_MS);
  }
}

void IMUcalculateDataTask(void *pvParameter) {
  int64_t offsetRX = 0.0;
  int64_t offsetRY = 0.0;
  int64_t offsetRZ = 0.0;

  int16_t previousAX = 0;
  int16_t previousAY = 0;
  int16_t previousAZ = 0;

  int64_t sumRX = 0;
  int64_t sumRY = 0;
  int64_t sumRZ = 0;

  float dphi, dtheta, dpsi;

  bool calibration = false;
  int64_t calibrationSteps = 0;
  uint8_t calibrationCountdown = CALIBRATION_COUNTDOWN;

  tIMUdata inputData = {0};

  while(true) {
#ifdef DEBUG
    Serial.print(millis());
    Serial.print(": phi: ");
    Serial.print(phi * RAD_TO_DEG);
    Serial.print(" theta: ");
    Serial.print(theta * RAD_TO_DEG);
    Serial.print(" psi: ");
    Serial.print(psi * RAD_TO_DEG);
#endif

    xEventGroupWaitBits(xIMUeventGroup, NEW_IMU_DATA_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    if (xSemaphoreTake(xVarMutex, portMAX_DELAY)) {
      inputData = imuData;
      xSemaphoreGive(xVarMutex);
    }

    if ((abs(previousAX - inputData.aX) < ACCELEROMETER_CUTOFF) && (abs(previousAY - inputData.aY) < ACCELEROMETER_CUTOFF) && (abs(previousAZ - inputData.aZ) < ACCELEROMETER_CUTOFF)) {
      previousAX = inputData.aX;
      previousAY = inputData.aY;
      previousAZ = inputData.aZ;

      if (calibration) {
#ifdef DEBUG
        Serial.println(" R");
#endif

        sumRX += inputData.rX;
        sumRY += inputData.rY;
        sumRZ += inputData.rZ;
        calibrationSteps++;
      } else {
#ifdef DEBUG
        Serial.println(" C");
#endif
        if (calibrationCountdown) {
          calibrationCountdown--;
          continue;
        }

        calibrationCountdown = CALIBRATION_COUNTDOWN;
        calibration = true;
        sumRX = inputData.rX;
        sumRY = inputData.rY;
        sumRZ = inputData.rZ;
        calibrationSteps = 1;
      }
      continue;
    } else {
#ifdef DEBUG
      Serial.println(" N");
#endif
      calibrationCountdown = CALIBRATION_COUNTDOWN;
      previousAX = inputData.aX;
      previousAY = inputData.aY;
      previousAZ = inputData.aZ;

      if (calibration) {
        calibration = false;
        offsetRX = sumRX / calibrationSteps;
        offsetRY = sumRY / calibrationSteps;
        offsetRZ = sumRZ / calibrationSteps;
      }
    }

    float omega_x = heart.imu.calcGyro(inputData.rX - offsetRX) * DEG_TO_RAD;
    float omega_y = heart.imu.calcGyro(inputData.rY - offsetRY) * DEG_TO_RAD;
    float omega_z = heart.imu.calcGyro(inputData.rZ - offsetRZ) * DEG_TO_RAD;

    //rungeKutta2(&phi, &theta, &psi, omega_x, omega_y, omega_z, 1.0 / imu.settings.gyroSampleRate);
    rungeKutta4(&phi, &theta, &psi, omega_x, omega_y, omega_z, 1.0 / heart.imu.settings.gyroSampleRate);

    if (phi >= TWO_PI) {
      phi -= TWO_PI;
    } else if (phi < 0.0) {
      phi += TWO_PI;
    }

    if (theta >= TWO_PI) {
      theta -= TWO_PI;
    } else if (theta < 0.0) {
      theta += TWO_PI;
    }

    if (psi >= TWO_PI) {
      psi -= TWO_PI;
    } else if (psi < 0.0) {
      psi += TWO_PI;
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("RoboHeart LSM6DS3 rotation demo");

  xVarMutex = xSemaphoreCreateMutex();
  xIMUeventGroup = xEventGroupCreate();

  heart.imu.settings.gyroEnabled = 1;
  heart.imu.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  heart.imu.settings.gyroSampleRate = 104;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  heart.imu.settings.gyroBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;

  heart.imu.settings.accelEnabled = 1;
  heart.imu.settings.accelODROff = 0;
  heart.imu.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  heart.imu.settings.accelSampleRate = heart.imu.settings.gyroSampleRate;
  heart.imu.settings.accelBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;

  heart.imu.settings.tempEnabled = 0;

  // Set up the RoboHeart
  heart.begin();

  xTaskCreatePinnedToCore(IMUcalculateDataTask, "IMUcalculateDataTask", 8192, NULL, configMAX_PRIORITIES - 10, NULL, 0);
  xTaskCreatePinnedToCore(IMUgetDataTask, "IMUgetDataTask", 8192, NULL, configMAX_PRIORITIES - 10, NULL, 0);
}

void loop() {
  Serial.print(millis());
  Serial.print(": phi: ");
  Serial.print(phi * RAD_TO_DEG);
  Serial.print(" theta: ");
  Serial.print(theta * RAD_TO_DEG);
  Serial.print(" psi: ");
  Serial.print(psi * RAD_TO_DEG);
  Serial.println();
  vTaskDelay(100/portTICK_PERIOD_MS);
}
