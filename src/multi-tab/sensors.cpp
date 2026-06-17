#include "sensors.h"
#include "globals.h"
#include "config.h"

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 30.0f;
  return (float)duration * 0.01715f;
}

void readAllSensors() {
  dist1 = readUltrasonic(TRIG1, ECHO1);
  delay(SENSOR_DELAY);
  dist2 = readUltrasonic(TRIG2, ECHO2);
  delay(SENSOR_DELAY);
  dist3 = readUltrasonic(TRIG3, ECHO3);
  delay(SENSOR_DELAY);
}

int detectVehicle(float distance_cm) {
  return (distance_cm < BATAS_LEBAR_JALAN) ? 1 : 0;
}
