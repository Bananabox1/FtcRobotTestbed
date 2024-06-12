## Whitefield Robotics Team - Programming Tutorial
### Goals:
1.	Understand hardware platform and software infrastructure involved
2.	Understand an OP mode, how they are written, how they are downloaded to the bot, and how they are executed
3.	Understand the FTC SDK/JDK and the libraries available
4.	Understand the structure of the existing robotics code
5.	Understand what online documentation and other resources exist at First for programmers
   
### Outline:
1. Setup hardware and install necessary software
    1. Install Driver Station (DS) app on Android phone
    2. Install Rev hardware client on laptop
        - Use to initially configure Control Hub and update firmware
    3. Connect Driver Station app to Control Hub
        -	Connect Android phone to Wi-fi provided by Control Hub
        -	Can also connect laptop to Control Hub Wi-fi and access internal website at 
    4. Install Android Studio on laptop
    5. Fork FIRST-Tech-Challenge/FtcRobotController repository on Github to a team Testbed repository
    6. Clone the Testbed repository to a folder on the laptop
    7. Open the Testbed repository with Android Studio
2. Build and Test Testbed
    1. Mount hardware to platform and connect
        - Control Hub
        - Battery
        - Servo
          [Click Here](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/configuring_servo/configuring-servo.html)
        - Camera
          [Click Here](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/configuring_external_webcam/configuring-external-webcam.html)
        - Xbox Controller (thru DS app)
        - Expansion possibilities:
            - DC Motor
            - Color Sensor
            - Distance Sensor
            - Digital Touch Sensor
    2. Configure Hardware in DS app
        - What alternatives are there to DS app for configuration
    3. Write “Hello World” oOp mode 
        - Basic linear op mode from Scratch
        - Moves servo
        - Instructions:
          [Click Here](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html)
    4. Build and run op mode from DS app
3. Learn Op Mode Development
    1. Explore op mode samples
       - Automated and manual op modes
       - Linear and iterative op modes
       - Concept op modes for servo
       - Concept TensorFlow object detection
    2. Write Op Mode(s) that exercises TestBed hardware
        - Both automated and manual op modes
        - Interfaces with servo and camera (and other hardware)
        - Responds to DS app/Xbox controller in manual mode
4. Learn Video Libraries
    1. Controlling and calibrating camera(s)
    2. VisionPortal
        - Enables simultaneous AprilTag and TFOD processors
        - Provides augmented display over HDMI port
        - Allows for camera controls and multiple cameras
    3. AprilTag
    4. TensorFlow Lite
    5. Vuforia(?)
    6. EasyOpenCV(?)
5. Learn Other Libraries and Scaffolding
    1. Roadrunner(?)
6. Learn Structure of Existing Code
7. Inventory Resources for Learning Programming
    - [Programming Tutorial Contents](https://ftc-docs.firstinspires.org/en/latest/programming_resources/index.html)

### Notes:
1. 2024-2025 FTC
      1. Theme: "Into the deep"
      2. Reveal: September 7th
 
