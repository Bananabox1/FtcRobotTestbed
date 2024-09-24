package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HelloWorld_Old extends LinearOpMode {

    private Servo servoTest;

    private double pos;

    @Override
    public void runOpMode() {

        servoTest = hardwareMap.get(Servo.class, "servoTest");

        pos = 0.5; // Initial position

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        servoTest.setPosition(pos);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double input = gamepad1.left_stick_x;

            if(input > 0.5) {
                pos += 0.05;
                if(pos > 1.0)
                    pos = 1.0;
            }

            if(input < -0.5) {
                pos -= 0.05;
                if(pos < 0.0)
                    pos = 0.0;
            }

            servoTest.setPosition(pos);

            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}