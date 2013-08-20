#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <string>

bool calibrateJoint(int joint,
                    double mass=0, double comx=0, double comy=0, double comz=0,
                    int resolution=20);

bool calibrateJoint(std::string joint,
                    double mass=0, double comx=0, double comy=0, double comz=0,
                    int resolution=20);

void calibrateArm(int side, bool prompt=false,
                  double mass=0, double comx=0, double comy=0, double comz=0,
                  int resolution=20);

void calibrateRightArm(bool prompt=false,
                       double mass=0, double comx=0, double comy=0, double comz=0,
                       int resolution=20);

void calibrateLeftArm(bool prompt=false,
                      double mass=0, double comx=0, double comy=0, double comz=0,
                      int resolution=20);

























#endif // CALIBRATION_H
