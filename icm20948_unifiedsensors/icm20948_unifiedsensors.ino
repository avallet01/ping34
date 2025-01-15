#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>

// Instances pour les deux capteurs
Adafruit_ICM20948 icm1; // Capteur 1 (adresse 0x68)
Adafruit_ICM20948 icm2; // Capteur 2 (adresse 0x69)

// Pointeurs pour les capteurs individuels
Adafruit_Sensor *icm1_temp, *icm1_accel, *icm1_gyro, *icm1_mag;
Adafruit_Sensor *icm2_temp, *icm2_accel, *icm2_gyro, *icm2_mag;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initialisation des deux capteurs ICM20948");

  // Initialisation du capteur 1 (adresse 0x68)
  if (!icm1.begin_I2C(0x68)) {
    Serial.println("Erreur : impossible de détecter le capteur 1 (0x68)");
    while (1) delay(10);
  }
  Serial.println("Capteur 1 (0x68) détecté !");
  icm1_temp = icm1.getTemperatureSensor();
  icm1_accel = icm1.getAccelerometerSensor();
  icm1_gyro = icm1.getGyroSensor();
  icm1_mag = icm1.getMagnetometerSensor();

  // Initialisation du capteur 2 (adresse 0x69)
  if (!icm2.begin_I2C(0x69)) {
    Serial.println("Erreur : impossible de détecter le capteur 2 (0x69)");
    while (1) delay(10);
  }
  Serial.println("Capteur 2 (0x69) détecté !");
  icm2_temp = icm2.getTemperatureSensor();
  icm2_accel = icm2.getAccelerometerSensor();
  icm2_gyro = icm2.getGyroSensor();
  icm2_mag = icm2.getMagnetometerSensor();
}

void loop() {
  // Variables pour stocker les données des capteurs
  sensors_event_t temp, accel, gyro, mag;

  // Lecture et transmission des données du capteur 1
  icm1_temp->getEvent(&temp);
  icm1_accel->getEvent(&accel);
  icm1_gyro->getEvent(&gyro);
  icm1_mag->getEvent(&mag);

  Serial.print("C1,");
  Serial.print(temp.temperature); Serial.print(",");
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");
  Serial.print(gyro.gyro.x); Serial.print(",");
  Serial.print(gyro.gyro.y); Serial.print(",");
  Serial.print(gyro.gyro.z); Serial.print(",");
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.println(mag.magnetic.z);

  // Lecture et transmission des données du capteur 2
  icm2_temp->getEvent(&temp);
  icm2_accel->getEvent(&accel);
  icm2_gyro->getEvent(&gyro);
  icm2_mag->getEvent(&mag);

  Serial.print("C2,");
  Serial.print(temp.temperature); Serial.print(",");
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");
  Serial.print(gyro.gyro.x); Serial.print(",");
  Serial.print(gyro.gyro.y); Serial.print(",");
  Serial.print(gyro.gyro.z); Serial.print(",");
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.println(mag.magnetic.z);

  delay(100); // Pause de 100 ms entre les lectures
}
