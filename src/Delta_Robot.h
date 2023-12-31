#ifndef DELTA_ROBOT_H
#define DELTA_ROBOT_H

struct CartesianCoordinates {
    float x;
    float y;
    float z;
};

struct JointAngles {
    float theta1;
    float theta2;
    float theta3;
};

class Delta_Robot {
public:
    Delta_Robot(float base_length, float upper_arm_length, float lower_arm_length);

    // Forward Kinematics: Calculates end-effector position based on joint angles
    CartesianCoordinates forwardKinematics(float theta1, float theta2, float theta3);

    // Inverse Kinematics: Calculates joint angles based on desired end-effector position
    JointAngles inverseKinematics(float x, float y, float z);

private:
    float base_length;
    float upper_arm_length;
    float lower_arm_length;

    float calculateZ(float theta);
};

#endif // DELTA_ROBOT_H
