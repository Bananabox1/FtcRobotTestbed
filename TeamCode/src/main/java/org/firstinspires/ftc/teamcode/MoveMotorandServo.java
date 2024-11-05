package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Move motor and servo on testbed", group="Test")
//@Disabled
public class MoveMotorandServo extends OpMode
{
    // Set some limits for testing
    private static final double MOTOR_MAX_POWER = .4;

    private static final double SERVO_OPEN_POSITION = 0.35;
    private static final double SERVO_CLOSED_POSITION = 0.05;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor;
    private Servo servo;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor = hardwareMap.get(DcMotor.class, "leftfront_drive");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);

        servo = hardwareMap.get(Servo.class, "servo_test");
        servo.setPosition(SERVO_CLOSED_POSITION);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // Left stick X controls the motor. Right stick Y controls the servo
        double motorPower = -gamepad1.left_stick_x;
        double servoPosition;
        if (gamepad1.right_trigger > 0.1)
            servoPosition = SERVO_OPEN_POSITION;
        else if (gamepad1.left_trigger > 0.1)
            servoPosition = servoPosition = SERVO_CLOSED_POSITION;
        else
            servoPosition = -1;

        // Send calculated power to wheels
        motor.setPower(motorPower*MOTOR_MAX_POWER);
        if (servoPosition != -1)
            servo.setPosition(servoPosition);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor", "Power %.2f", motorPower);
        telemetry.addData("Servo", "Position %.2f", servoPosition);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        motor.setPower(0.0);
    }

}
