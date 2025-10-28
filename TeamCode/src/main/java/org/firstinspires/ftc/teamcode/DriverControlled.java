package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Driver Controlled")
public class DriverControlled extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize drive motors
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        // Initialize outtake motors and servo
        DcMotorEx leftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "leftOuttakeMotor");
        DcMotorEx rightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "rightOuttakeMotor");
        Servo transferServo = hardwareMap.get(Servo.class, "transferServo");

        // Reverse directions of motors
        //backRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightOuttakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
       // frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set run modes
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftOuttakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightOuttakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            if (gamepad1.a) { //sets power for the far position
                leftOuttakeMotor.setPower(1.0);
                rightOuttakeMotor.setPower(1.0);
            } else if (gamepad1.b) { //stops the motors
                leftOuttakeMotor.setPower(0.0);
                rightOuttakeMotor.setPower(0.0);
            } else if (gamepad1.y) { //sets motor power for the close position
                leftOuttakeMotor.setPower(0.5); //tune based on how powerful motor is
                rightOuttakeMotor.setPower(0.5); //tune based on how powerful motor is
            }

            // Transfer servo control (brief push)
            if (gamepad1.x) {
                transferServo.setPosition(1.0); // push up
                sleep(500); // wait half a second
                transferServo.setPosition(0.0); // return down
            }

            // ----- Calculate drive power -----
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize power
            double maxPower = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Apply drive power
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            telemetry.addData("Outtake Power (L/R)", "%4.2f / %4.2f", leftOuttakeMotor.getPower(), rightOuttakeMotor.getPower());
            telemetry.addData("Transfer Servo Position", transferServo.getPosition());
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.update();
        }
    }
}
