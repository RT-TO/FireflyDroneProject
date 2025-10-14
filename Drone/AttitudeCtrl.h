#ifndef ATTITUDECRTL_H
#define ATTITUDECRTL_H

#include "quaternion.h"

void quatGyroUpdate(Quaternion *q, double gx, double gy, double gz, double dt);
void acceAngleComp(Quaternion *q, double ax, double ay, double az, double dt);
Quaternion bodyToEarth(Quaternion q, Quaternion v_b);
double getAltitude(double pressure, double temperature);

#endif