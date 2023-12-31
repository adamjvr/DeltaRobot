#include "Delta_Robot.h"
#include <cmath>

Delta_Robot::Delta_Robot(float base_length, float upper_arm_length, float lower_arm_length) :
    base_length(base_length), upper_arm_length(upper_arm_length), lower_arm_length(lower_arm_length) {}

CartesianCoordinates Delta_Robot::forwardKinematics(float theta1, float theta2, float theta3) {
    float t = (sqrt(3.0) / 2.0) * base_length;

    float y1 = -t;
    float y2 = t;

    float x1 = -base_length;
    float x2 = 0;
    float x3 = base_length;

    float z1 = calculateZ(theta1);
    float z2 = calculateZ(theta2);
    float z3 = calculateZ(theta3);

    float delta = (pow(y2 - y1, 2) + pow(x2 - x1, 2)) * (pow(z3 - z1, 2) + pow(x3 - x1, 2)) -
                  (pow(z2 - z1, 2) + pow(x2 - x1, 2)) * (pow(y3 - y1, 2) + pow(x3 - x1, 2));

    CartesianCoordinates result;
    result.x = ((y2 - y1) * (pow(z3 - z1, 2) + pow(x3 - x1, 2)) - (y3 - y1) * (pow(z2 - z1, 2) + pow(x2 - x1, 2))) / (2 * delta);
    result.y = ((x3 - x1) * (pow(z2 - z1, 2) + pow(x2 - x1, 2)) - (x2 - x1) * (pow(z3 - z1, 2) + pow(x3 - x1, 2))) / (2 * delta);
    result.z = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / (2 * delta);

    return result;
}

JointAngles Delta_Robot::inverseKinematics(float x, float y, float z) {
    float t = (sqrt(3.0) / 2.0) * base_length;

    float y1 = -t;
    float y2 = t;

    float x1 = -base_length;
    float x2 = 0;
    float x3 = base_length;

    float a1 = x1 * x1 + y1 * y1 - x * x - y * y + z * z;
    float a2 = x2 * x2 + y2 * y2 - x * x - y * y + z * z;
    float a3 = x3 * x3 + y3 * y3 - x * x - y * y + z * z;

    float b1 = 2 * (y * (y1 - y2) + x * (x2 - x1) + z * (z1 - z2));
    float b2 = 2 * (y * (y2 - y3) + x * (x3 - x2) + z * (z2 - z3));
    float b3 = 2 * (y * (y3 - y1) + x * (x1 - x3) + z * (z3 - z1));

    float c1 = x1 * x1 + y1 * y1 + z1 * z1 - x * x - y * y - z * z;
    float c2 = x2 * x2 + y2 * y2 + z2 * z2 - x * x - y * y - z * z;
    float c3 = x3 * x3 + y3 * y3 + z3 * z3 - x * x - y * y - z * z;

    float d1 = 2 * (y1 * (y2 - y3) + x1 * (x3 - x2) + z1 * (z2 - z3));
    float d2 = 2 * (y2 * (y3 - y1) + x2 * (x1 - x3) + z2 * (z3 - z1));
    float d3 = 2 * (y3 * (y1 - y2) + x3 * (x2 - x1) + z3 * (z1 - z2));

    JointAngles result;

    // Calculate joint angles using atan and sqrt functions
    result.theta1 = 2 * atan((-b1 - sqrt(b1 * b1 - 4 * a1 * c1)) / (2 * a1));
    result.theta2 = 2 * atan((-b2 - sqrt(b2 * b2 - 4 * a2 * c2)) / (2 * a2));
    result.theta3 = 2 * atan((-b3 - sqrt(b3 * b3 - 4 * a3 * c3)) / (2 * a3));

    return result;
}

float Delta_Robot::calculateZ(float theta) {
    return upper_arm_length * cos(theta) + lower_arm_length;
}
