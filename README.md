# ROBOT ARM

“A robot arm is a mechanical device designed to mimic the movements of a human arm. It is typically used in industries such as manufacturing, assembly, and automation. The robot arm consists of multiple joints and segments that allow it to move and manipulate objects with precision and accuracy. With the advancements in robotics technology, robot arms have become increasingly versatile and capable of performing complex tasks. They can be programmed to perform repetitive actions, handle delicate objects, and even work in hazardous environments. Robot arms have revolutionized industries by increasing productivity, efficiency, and safety in various applications.”

# 1. COMPONENTS

## 1.1 KEY COMPONENTS

- 
    1. **WeMos D1 Mini** - **MICROCONTROLLER:** Responsible for receiving sensor data, performing calculations, and sending control signals to servos. (**₹400)**
    2. **Micro Servo Motors** - x2 - **Motors** (**₹90 * 2 = ₹180)**
    3. **High Torque Servo Motors** - x4 - **Motors:** Actuate the robot arm's joints, controlling their motion and positioning. (**Approaching ₹2000)**
    4. **PWM Board** - I2C Address(Closed = 1, Open = 0), **extension board** used to drive 6 servos.
    Translates digital signals from the microcontroller into pulse-width modulation (PWM) commands, which are understood by the servo motors. (**Approaching ₹600)**
    

### 1.2 SENSORS

- 
    1. **Accelerometer + Gyro (GY-521):**  Two sensors in the upper and lower arm measure acceleration and orientation, providing data for joint angle calculations. (**₹600)**
    2. **Compass (GY-511):** A compass at the wrist measures the arm's yaw, aiding in overall orientation determination. (**₹400)**
    3. **Linear Hall Effect Sensor:** A proximity sensor near the thumb provides feedback on gripper position. (**₹200)**

## 1.3 OTHERS

- 
    1. **Ball bearings:** Reduce friction and allow smooth joint movement. (**₹200 for 10pcs)**
    2. **Nuts & bolts:** Secure the robot arm's structure and components. (**Approx** **₹100)**
    3. **PCB Boards:** Provide electrical connections and support for components. 
    (**₹90 for 2 pcs * 2 = ₹180)**
    4. **Extension cables:** Extend the reach of sensor and servo connections. 
    (Used normal jumper wires)
    5. **Thick 5V Power Supply (₹300)**

**(TOTAL EXPENSES:** 5-6k for above components + 10k for 3D printed parts = **15k for project)**

# 2. MECHANISM

### 2.1 **Sensor Functionality:**

- 
    1. **Three-Axis Accelerometers(for SH, UA, LA):** Measure the linear acceleration of the arm's segments, helping determine their orientation relative to gravity. (G1 GY521)
    2. **Magnetometer (Compass for WR):** Measures the Earth's magnetic field, providing the arm's yaw (rotation around the vertical axis). (GY-511)
    3. **Gripper Sensor(TH):** Detects proximity to a magnet, indicating gripper position and object interaction. (Linear Hall Effect)

### 2.2 **Sensor Data Processing:**

- 
    1. **Orientation Calculation:** Accelerometer readings are used to calculate the orientation of the arm's segments, relative the gravity vectors. These determine the angle of servo motors.
    
        
        Since regular movement also creates acceleration, 
        the trajectory of the robot arm will differ at higher movement speeds.
        
    2. **Yaw Correction:** Compass measurements are combined with accelerometer data to determine the arm's absolute yaw.
    
    The measured position on the start up of the micro controller is measured and taken as the reference forward direction later. The compass vector is taken at the wrist and applied to a servo which rotates the complete arm.
    
    3. **Matrix Transformations:**  Sensor orientations are transformed using matrices to account for arm geometry and coordinate systems.
    
        
        Since the orientation of the wrist sensor needs to be subtracted from the compass vector to get the absolute world orientation, all transformations from shoulder to wrist are back transformed using matrices. This happens in the *calculateAngles* method of the sketch.
        
        ```cpp
        //Calculate the absolute compass vector
        
        float a1 = -atan2(-sensors[1][1], -sensors[1][0]);
        Matrix r0 = Matrix::rotation(-a1, 0, 1, 0); 
        
        float a2 = atan2(sensors[2][1], sensors[2][2]);
        Matrix r1 = Matrix::rotation(angles[4], 1, 0, 0);
        
        Vector v = r0 * r1 * sensors[3];
        v[2] = 0;
        v.normalize();
        compass0 = v;
        ```
        
![Untitled](https://github.com/dangopea/RobotArm/assets/67912172/06112908-8dc3-4228-b25d-316458d29f44)
![Untitled2](https://github.com/dangopea/RobotArm/assets/67912172/fff9dcf6-7fa9-406d-b44d-6418bcd8c8f7)


### 2.3 **Servo Control & Movement Execution:**

1. **Servo Angle Calculation:** Servo angles are calculated based on sensor data and desired arm positions.
2. **PWM Signal Generation:** The microcontroller generates PWM signals corresponding to the calculated servo angles.
3. **PWM Board Control:** The PWM board translates PWM signals into servo-compatible commands, controlling servo movements.

<aside>
💡 **To achieve precise positioning**
The system employs *feedback control,* continuously monitoring the arm's actual position and adjusting servo commands as necessary. This ensures that the arm reaches the desired position accurately and efficiently.

</aside>
# 3. WIRING

Almost all communication is done using I²C. The I²C is a bit limited in signal strength, so you might want to add additional pull-up resistors to SDA and SCL. 

To be able to address two GY-521 sensors at once, they need to have two different addresses.

- **G1 GY-521:** GND to AD0
- **G2 GY-521:** VCC to AD0

All sensor boards are powered by 5V since they have their own 3.3V regulators. 
The GY-511 board also has an 3.3V pin which changes the pin order.. so, be careful to connect the right pin to 5V if building an adapter board.

The power supply that's connected to the PWM board screw connectors should be able to apply several amps since quite some forces can occur with 6 active servos. Your USB will be overloaded for sure if you don't use an additional supply.

## 3.1 Microcontroller (WeMos D1 Mini)

- 
    
    
    | Pins | Connected to |
    | --- | --- |
    | 1. D1 | 1. SCL pin of:
    - G1 GY-521
    - G2 GY-521
    - G1 GY-511
    2. SCL 2 pin of PWM Servo Driver |
    | 2. D2 | 1. SDA pin of:
    - G1 GY-521
    - G2 GY-521
    - G1 GY-511
    2. SDA 2 pin of PWM Servo Driver |
    | 3. GND | 1. All GNDs of Sensors
    2. GND 2 pin of PWM Servo Driver |
    | 4. 5V | 1. All VCCs of Sensors
    2. VCC 2 pin of PWM Servo Driver |
    | 5. A0 | OUT/ANG pin of Linear Hall Effect |

## 3.2 Upper Arm (G1 GY-521)

- 
    
    
    | Pins | Connected to |
    | --- | --- |
    | 1. SCL | D1 of WeMos |
    | 2. SDA | D2 of WeMos |
    | 3. GND | GND of WeMos |
    | 4. VCC | 5V of WeMos |

## 3.3 Lower Arm (G2 GY-521)

- 
    
    
    | Pins | Connected to |
    | --- | --- |
    | 1. SCL | D1 of WeMos |
    | 2. SDA | D2 of WeMos |
    | 3. GND | GND of WeMos |
    | 4. VCC | 5V of WeMos |

## 3.4 Wrist (G1 GY-511)

- 
    
    
    | Pins | Connected to |
    | --- | --- |
    | 1. SCL | D1 of WeMos |
    | 2. SDA | D2 of WeMos |
    | 3. GND | GND of WeMos |
    | 4. VCC | 5V of WeMos |

## 3.5 Thumb (Linear Hall Effect Sensor)

- 
    
    
    | Pins | Connected to |
    | --- | --- |
    | 1. GND | GND of WeMos |
    | 2. VCC | 5V of WeMos |
    | 3. OUT | A0 of WeMos |

## 3.6 PWM-SERVO-DRIVER

- 
    
    
    | Pins | Connected to |
    | --- | --- |
    | 1. V+ & GND | Thick 5V Adapter |
    | 2. SCL 2 | D1 of WeMos |
    | 3. SDA 2 | D2 of WeMos |
    | 4. GND 2 | GND of WeMos |
    | 5. VCC 2 | 5V of WeMos |
    | 6. PWM, VCC & GND | All connected to 6 Servo Motors: |

![Schematic](https://github.com/dangopea/RobotArm/assets/67912172/ecf3d5e8-ed7b-4cfd-918c-b50be865d0e0)


# 4. SOLDERING & FIXING

1. Do this for 3 sensors + 1 microcontroller
    1. G1 GY-521: Solder headers in place
    2. G2 GY-521: Solder headers in place
    3. G1 GY-511: Solder headers in place
    4. WeMos D1 R2 Mini - Solder headers in place

1. GY521 x2 - 
    1. AD0 to VCC
    2. AD0 to GND
    
<img width="1212" alt="Screenshot 2566-11-13 at 5 10 34 pm" src="https://github.com/dangopea/RobotArm/assets/67912172/69e89a84-10f7-4c91-b9c8-adea690cf275">

