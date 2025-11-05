package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "Blue Far Auto")
public class BlueFarAuto extends LinearOpMode {

    Bytes_Robot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Bytes_Robot(hardwareMap);

        Pose2d beginPose = new Pose2d(50, -15, Math.toRadians(35));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // Run outtake to shoot
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(1150);
                            myRobot.rightOuttakeMotor.setVelocity(1150);
                        })

                        .waitSeconds(1)

                        // Run intake to shoot
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))

                        // Stop Outtake
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(0.0);
                            myRobot.rightOuttakeMotor.setVelocity(0.0);
                        })

                        // Stop intake
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))

                        .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(90)), Math.toRadians(180)) // pick up more balls

                        // Run intake at new location
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))

                        .strafeTo(new Vector2d(36,-38))
                        .splineToLinearHeading(new Pose2d(50, -15, Math.toRadians(35)), Math.toRadians(20)) // drive back to line

                        // Stop intake now that balls are collected
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))

                        // Spin up outtake flywheels
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(1150);
                            myRobot.rightOuttakeMotor.setVelocity(1150);
                        })

                        // Start intake to shoot
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))

                        // Stop outtake
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(0.0);
                            myRobot.rightOuttakeMotor.setVelocity(0.0);
                        })

                        // Stop intake
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))

                        .build());



        // store pose for use in later TeleOp modes
        BYTES_GLOBAL_Storage.currentPose = drive.localizer.getPose();
    }
}