#include "quaternion.h"


void quatGyroUpdate(Quaternion *q, double gx, double gy, double gz, double dt) {
    Quaternion q_gyro = {0, gx, gy, gz};
    //运算
    Quaternion q_tnext = quatMultiply(*q, q_gyro);
    q_tnext.w *= 0.5;
    q_tnext.x *= 0.5;
    q_tnext.y *= 0.5;
    q_tnext.z *= 0.5;

    q_gyro.w += q_tnext.w * dt;
    q_gyro.x += q_tnext.x * dt;
    q_gyro.y += q_tnext.y * dt;
    q_gyro.z += q_tnext.z * dt;
    //归一
    quatNormalize(q);
}

void acceAngleComp(Quaternion *q, double ax, double ay, double az, double dt) {
    //TODO
}

Quaternion bodyToEarth(Quaternion q, Quaternion v_b) {
    //q 机体姿态, v_b 机体坐标系
    Quaternion q_con = quatConjugate(q);
    Quaternion res = quatMultiply(quatMultiply(q, v_b), q_con);
    return res;
}

double getAltitude(double pressure, double temperature) {
    // Pressures(Pa)
    // Temperature(Celcius)

    double temp_K = temperature + 273.15;
    double ratio = pressure / 101325.0;
    double altitude = (temp_K / 0.0065) * (1 - pow(ratio, (287.05 * 0.0065 / 9.80665)));
    return altitude;
}