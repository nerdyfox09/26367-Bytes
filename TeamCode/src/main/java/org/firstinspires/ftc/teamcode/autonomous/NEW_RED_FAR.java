package org.firstinspires.ftc.teamcode.autonomous;

import android.app.Notification;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BYTES_CONFIG;
import org.firstinspires.ftc.teamcode.Bytes_Robot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

import java.lang.Math;

@Autonomous (name = "NEW Red Far Auto")
public class NEW_RED_FAR extends LinearOpMode {

    Bytes_Robot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Bytes_Robot(hardwareMap, telemetry);

        // Mirror across field centerline (y changes sign)
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));

        myRobot.localizer = new ThreeDeadWheelLocalizer(
                hardwareMap,
                BYTES_CONFIG.PARAMS_DRIVETRAIN_IN_PER_TICK,
                beginPose);

        myRobot.driveMode = Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE;

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        myRobot.leftOuttakeMotor.setVelocity(775);
        myRobot.rightOuttakeMotor.setVelocity(775);

        myRobot.leftTransferServo.setDirection(Servo.Direction.FORWARD);
        myRobot.rightTransferServo.setDirection(Servo.Direction.REVERSE);

        myRobot.leftTransferServo.setPosition(.75);
        myRobot.rightTransferServo.setPosition(.75);

        sleep(500);

        myRobot.intakeMotor.setPower(0.4);

        sleep(8000);

        myRobot.leftOuttakeMotor.setVelocity(0);
        myRobot.rightOuttakeMotor.setVelocity(0);

        myRobot.leftTransferServo.setDirection(Servo.Direction.FORWARD);
        myRobot.rightTransferServo.setDirection(Servo.Direction.REVERSE);

        myRobot.leftTransferServo.setPosition(.5);
        myRobot.rightTransferServo.setPosition(.5);

        myRobot.intakeMotor.setPower(0);

        myRobot.frontLeft.setPower(0.75);
        myRobot.backLeft.setPower(0.75);
        myRobot.frontRight.setPower(-0.75);
        myRobot.backRight.setPower(-0.75);

        sleep(2000);

        myRobot.frontLeft.setPower(0);
        myRobot.backLeft.setPower(0);
        myRobot.frontRight.setPower(0);
        myRobot.backRight.setPower(0);

    }
}
