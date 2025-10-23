package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "intake outtake test")
public class intakeOuttakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize drive motors
        ;
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Initialize outtake motors and servo
        DcMotorEx leftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "leftOuttakeMotor");
        DcMotorEx rightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "rightOuttakeMotor");
        Servo transferServo = hardwareMap.get(Servo.class, "transferServo");

        // Reverse directions of motors
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftOuttakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Set run modes

        leftOuttakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightOuttakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servo position
        transferServo.setPosition(0.0); // starting position (flap down)

        waitForStart();

        while (opModeIsActive()) {
            // Driving controls
            double drive = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double rotate = gamepad1.left_stick_x;

            // Intake controls
            double intake = gamepad1.left_trigger;
            if (intake > 0) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }

            double outtake = gamepad1.right_trigger;
            if (outtake > 0) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            // Outtake (flywheel) control
            if (gamepad1.a) { //sets power for the close position
                leftOuttakeMotor.setPower(0.32);
                rightOuttakeMotor.setPower(0.32);
            } else if (gamepad1.y) { //stops the motors
                leftOuttakeMotor.setPower(0.0);
                rightOuttakeMotor.setPower(0.0);
            } else if (gamepad1.b) { //sets motor power for the far position
                leftOuttakeMotor.setPower(0.4); //tune based on how powerful motor is
                rightOuttakeMotor.setPower(0.4); //tune based on how powerful motor is
            }

            // Transfer servo control (brief push)
            if (gamepad1.x) {
                transferServo.setPosition(0.5); // push up
                sleep(500); // wait half a second
                transferServo.setPosition(0.0); // return down
            }





            telemetry.addData("Outtake Power (L/R)", "%4.2f / %4.2f", leftOuttakeMotor.getPower(), rightOuttakeMotor.getPower());
           // telemetry.addData("Transfer Servo Position", transferServo.getPosition());
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.update();
        }
    }
}
