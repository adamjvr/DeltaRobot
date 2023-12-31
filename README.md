# DeltaRobot
Computes forward and inverse kinematics for delta robots

# Delta Robot Class

The `Delta_Robot` class represents a parallel kinematic manipulator, also known as a delta robot. This class provides methods for both forward and inverse kinematics calculations, allowing you to determine the end-effector position based on joint angles and vice versa.

## Class Definition

### Delta_Robot

#### Constructor

```cpp
Delta_Robot(float base_length, float upper_arm_length, float lower_arm_length);
```
**Parameters:**
- `base_length`: Length of the base triangle side.
- `upper_arm_length`: Length of the upper arm.
- `lower_arm_length`: Length of the lower arm.

## Methods 

### Forward Kinematics
```cpp
CartesianCoordinates forwardKinematics(float theta1, float theta2, float theta3);
```

**Parameters:**
- `theta1`: Joint angle for the first arm.
- `theta2`: Joint angle for the second arm.
- `theta3`: Joint angle for the third arm.

**Return Type:** 
- `CartesianCoordinates` structure.

### Inverse Kinematics

```cpp
JointAngles inverseKinematics(float x, float y, float z);
```

**Parameters:**
- `x`: Desired x-coordinate of the end-effector.
- `y`: Desired y-coordinate of the end-effector.
- `z`: Desired z-coordinate of the end-effector.

**Return Type:** 
- `JointAngles` structure.

## Structs
**CartesianCoordinates**

```cpp
struct CartesianCoordinates {
    float x;
    float y;
    float z;
};
```

**Members:**
- `x`: X-coordinate of a point in Cartesian space.
- `y`: Y-coordinate of a point in Cartesian space.
- `z`: Z-coordinate of a point in Cartesian space.

**JointAngles**
```cpp
struct JointAngles {
    float theta1;
    float theta2;
    float theta3;
};
```

**Members:**
- `theta1`: Joint angle for the first arm.
- `theta2`: Joint angle for the second arm.
- `theta3`: Joint angle for the third arm.

## Usage

- Create an instance of the Delta_Robot class with the desired dimensions.
- Use the forwardKinematics method to calculate the end-effector position based on joint angles.
- Use the inverseKinematics method to calculate joint angles based on the desired end-effector position.

```cpp
Delta_Robot delta(100.0, 200.0, 300.0); // Replace with actual dimensions

// Example: Forward Kinematics
CartesianCoordinates result = delta.forwardKinematics(0.1, 0.2, 0.3);

// Example: Inverse Kinematics
JointAngles angles = delta.inverseKinematics(150.0, 0.0, 250.0);
```
## Contributing

Feel free to contribute to the development of this library. If you find any issues or have suggestions for improvements, please create an issue or submit a pull request.

## License

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.