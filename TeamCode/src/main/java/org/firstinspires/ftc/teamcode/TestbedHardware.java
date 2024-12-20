package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class TestbedHardware {

    // Instance variables

    // For accessing the calling OpMode for hardware initialization, etc.
    private LinearOpMode myOpMode;

    // Hardware components on TestBed
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;  //  Motors for Mecanum drive

    private Servo servoTest; // Test servo
    private TouchSensor touchSensorTest; // Test touch sensor
    private DistanceSensor distanceSensorTest; // Test distance sensor

    // Encoders (deadwheels) for odometry - not present on testbed but needed for sample code
    private DcMotorEx encoderRight, encoderLeft, encoderAux;

    // Vision portal and AprilTag processor
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection processor */
    //private VisionProcessor colorDetect; // other vision processors as needed, e.g., custom VisionProcessor class

    // Values for Motion functions
    // ***** Adjustable Parameters *****
    private final double DEADWHEEL_CM_PER_TICK = 0.65; // for odometry calculations
    private final double LENGTH =1.4; //length B in cm

    // Odometry counters
    private int currentRightPosition;
    private int currentLeftPosition;
    private int currentAuxPosition;

    // Current robot vector (x, y, and Θ in field coordinates)
    //private Pose2D currentPosition;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public TestbedHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {

        // Define Mecanum drivetrain hardware instance variables
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftfront_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightfront_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "leftback_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightback_drive");

        // Set drive motor directions based motor/wheel installation
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define other hardware instant variables
        servoTest = myOpMode.hardwareMap.get(Servo.class, "servo_test");
        touchSensorTest = myOpMode.hardwareMap.get(TouchSensor.class, "touch_sensor");
        distanceSensorTest = myOpMode.hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // Initialize other hardware
        servoTest.setPosition(0.5); // Initially center the servo position
        servoTest.scaleRange(0, 1); // Set scale range based on any hardware installation limitations, alternatively could set a minimum and/or maximum position value

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    // Basic movement methods for four-motor Mecanum drive train
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void move(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Get servo position
     */
    public double getServoPosition() {
        return servoTest.getPosition();
    }

    /**
     * Set servo position
     */
    public void setServoPosition(double pos) {
        servoTest.setPosition(pos); // This could, e.g., be scaled here
    }

    /**
     * Get the distance sensor reading
     */
    public double getDistance() {
        return distanceSensorTest.getDistance(DistanceUnit.MM);
    }

    /**
     * Get the touch sensor state
     */
    public boolean isTouchSensorPressed() {
        return touchSensorTest.isPressed();
    }

    // Methods for camera and AprilTag initialization, configuration, and processing

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag() {

        // Build the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(1469.86, 1469.86, 612.825,366.546)

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initAprilTag().
    */
    public void setCameraExposure(int exposureMS, int gain) {

        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                myOpMode.sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpMode.sleep(20);
        }
    }

    /**
     * Return AprilTag detections (wrapper function)
     */
    public List<AprilTagDetection> getAprilTagDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Return current encoder values from motors
     */
    public int[] getCurrentEncoderValues() {
        int[] encVals = new int[4];

        encVals[0] = leftFrontDrive.getCurrentPosition();
        encVals[1] = rightFrontDrive.getCurrentPosition();
        encVals[2] = leftBackDrive.getCurrentPosition();
        encVals[3] = rightBackDrive.getCurrentPosition();

        return encVals;
    }

    // Working code for autonomous navigation
    public void moveToFieldPosition(double x, double y) {

    }

    public void setHeading(double theta) {

    }

}


