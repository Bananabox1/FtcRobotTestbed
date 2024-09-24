package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

<<<<<<< HEAD
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class HelloWorldOpMode extends LinearOpMode {
    private double pos;
    private Servo servoTest;
    private VisionPortal myVisionPortal;
    @Override
    public void runOpMode() {
        servoTest = hardwareMap.get(Servo.class, "servoTest");
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"));
=======
@TeleOp
public class HelloWorld_Old extends LinearOpMode {

    private Servo servoTest;

    private double pos;

    @Override
    public void runOpMode() {

        servoTest = hardwareMap.get(Servo.class, "servoTest");

        pos = 0.5; // Initial position

>>>>>>> origin/master
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

<<<<<<< HEAD
        pos = 0.5; // initial position
        servoTest.setPosition(pos);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            double input = gamepad1.left_stick_x;
            if (input > 0.5)
                pos += 0.05;
            if (pos > 1.0)
                pos = 1.0;
            if (input < -0.5)
                pos -= 0.05;
            if (pos < 0.0)
                pos = 0.0;
            servoTest.setPosition(pos);


        }
    }
}
=======
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
>>>>>>> origin/master
