#include <math.h>

typedef struct {
    double w;
    double x;
    double y;
    double z;
} Quaternion;

Quaternion quatMultiply(Quaternion q1, Quaternion q2) {
    Quaternion res;
    res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    res.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    res.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    res.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return res;
}

void quatNormalize(Quaternion *q) {
    double norm = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

Quaternion quatConjugate(Quaternion q) {
    Quaternion res = q;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return res;
}

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

