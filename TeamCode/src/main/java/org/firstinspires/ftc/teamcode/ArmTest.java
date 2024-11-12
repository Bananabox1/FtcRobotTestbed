package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode is for testing and calibrating the arm rotation motor, viper-slide extension motor, and
 * gripper servo on the 2024-2025 INTO THE DEEP competition robot.
 *
 * This OpMode is teleop and in the "Test" group.
 *
 * NOTE: This OpMode contains a servo zeroing feature.  When the X button is pressed, the claw
 * servo will reset to the zero position. This should be done BEFORE installing the right gripper
 * arm on the claw body to ensure that the limits in the RobotHardware class are correct.
 */

@TeleOp(name="Test and Calibrate Arm and Claw", group="Test")
//@Disabled
public class ArmTest extends OpMode
{
    // Set some limits for testing
    private static final double EXTENSION_MOTOR_MAX_POWER = 0.7;
    private static final int ARM_EXTENSION_LIMIT = 2938; // Encoder position at maximum viper-slide extension

    // Tolerances and proportional gain values for arm extension position controller. These need to be calibrated.
    static final int ARM_EXTENSION_DEADBAND = 25; // Deadband range for arm extension position in ticks (~1% of full extension)
    static final double ARM_EXTENSION_KP = 0.00333; // Proportional gain for arm extension position error

    private static final double CLAW_SERVO_RANGE_MIN = 0.0;
    private static final double CLAW_SERVO_RANGE_MAX = 0.6;
    private static final double CLAW_OPEN_POSITION = 0.8;
    private static final double CLAW_CLOSED_POSITION = 0.0;

    // Declare OpMode members.
    private DcMotorEx rotationMotor;
    private DcMotorEx extensionMotor;
    private Servo gripperServo;

    // Store last gamepad state
    private final Gamepad lastGamepadState = new Gamepad();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware objects
        // NOTE: On the TestBed, use the zero motor interface ("leftfront_drive") and the zero
        // servo interface ("servo_test") so we don't have to change the hardware configuration
        // from the Mecanum drive.
        extensionMotor = hardwareMap.get(DcMotorEx.class, "leftfront_drive");

        // configure motor settings
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extensionMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // NOTE: On the TestBed, use the one motor interface ("rightfront_drive") for the rotation
        // motore so we don't have to change the hardware configuration from the Mecanum drive.
        rotationMotor = hardwareMap.get(DcMotorEx.class, "rightfront_drive");
        rotationMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rotationMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        gripperServo = hardwareMap.get(Servo.class, "servo_test");

        // set a range for the servo and set the initial position
        gripperServo.scaleRange(CLAW_SERVO_RANGE_MIN, CLAW_SERVO_RANGE_MAX);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // move the servo to zero position if X button is pressed
        // NOTE: used for if the A button is pressed
        if (gamepad1.x) {
            gripperServo.setPosition(0.0);
        }
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

        // reset the encoder count to zero
        // NOTE: the viper slide should be fully retracted before "Start" is pressed
        extensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //Thread.sleep(100);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // set the initial position of the claw
        gripperServo.setPosition(CLAW_CLOSED_POSITION);

        // Tell the driver that the OpMode is running.
        telemetry.addData("Status", "Running...");
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // Right trigger controls the claw servo
        double servoPosition;
        if (gamepad1.right_trigger > 0.4) {
            servoPosition = CLAW_OPEN_POSITION;
        } else {
            servoPosition = CLAW_CLOSED_POSITION;
        }
        //servoPosition = Math.abs(gamepad1.right_stick_x);
        gripperServo.setPosition(servoPosition);

        // if the arm extension motor is busy extending arm, don't allow the driver to control it
        if (!extensionMotor.isBusy()) {

            // Reset the encoder if the A button is pressed
            if (gamepad1.a) {
                extensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }

            // If a particular position for the arm extension is desired, set it using the
            // setArmExtension method
            if(gamepad1.left_bumper && !lastGamepadState.left_bumper ||
                    gamepad1.right_bumper && !lastGamepadState.right_bumper ||
                    gamepad1.dpad_up && !lastGamepadState.dpad_up ||
                    gamepad1.dpad_down && !lastGamepadState.dpad_down) {

                int pos = extensionMotor.getCurrentPosition();
                // bumpers move to the limits, dpad up and down move the arm extension by 300 ticks
                if(gamepad1.left_bumper)
                    pos = 0;
                else if(gamepad1.right_bumper)
                    pos = ARM_EXTENSION_LIMIT;
                else if(gamepad1.dpad_up)
                    pos += 300;
                else if(gamepad1.dpad_down)
                    pos -= 300;

                // run the motor to the desired position
                extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extensionMotor.setTargetPosition(clip(pos, 0, ARM_EXTENSION_LIMIT));
                extensionMotor.setPower(EXTENSION_MOTOR_MAX_POWER);
            }

            // otherwise, get the arm movement from the left stick
            else {

                // Left stick Y controls the arm extension.
                double motorPower = -gamepad1.left_stick_y;
                //extensionMotor.setPower(motorPower * EXTENSION_MOTOR_MAX_POWER);

                // Use values from gamepad to set extension motor power applying a proportional gain to
                // the power as the position approaches the limits.
                double currentPosition = extensionMotor.getCurrentPosition();
                double error;
                if (motorPower < 0)
                    error = currentPosition - 0;
                else
                    error = ARM_EXTENSION_LIMIT - currentPosition;

                // reset the motor mode to run using encoder
                extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                if (Math.abs(error) > ARM_EXTENSION_DEADBAND)
                    extensionMotor.setPower(motorPower * clip(ARM_EXTENSION_KP * error, -1.0, 1.0) * EXTENSION_MOTOR_MAX_POWER);
                else
                    extensionMotor.setPower(0.0);
            }

            // Right stick Y controls the arm rotation.
            rotationMotor.setPower(gamepad1.right_stick_y);

            // store the gamepad state for the next loop
            lastGamepadState.copy(gamepad1);

        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running...");
        telemetry.addData("Encoder", "Position: " + extensionMotor.getCurrentPosition());
        telemetry.addData("Motor", "Power %.2f", extensionMotor.getPower());
        telemetry.addData("Servo", "Position %.2f", gripperServo.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // stop any arm extension motor movement
        extensionMotor.setPower(0.0);

        // Tell the driver that the OpMode has stopped.
        telemetry.addData("Status", "Stopped");
    }
}