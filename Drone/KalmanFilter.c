#include <math.h>
#include "quaternion.h"

#define DEG2RAD (3.14159265358979323846f / 180.0f)
#define G_TO_MS2 9.80665f

typedef struct {
    Quaternion q;
    double bg[3]; // 陀螺仪偏置
    double P[6][6]; // 状态cov角度+偏置
} MEKF_State;