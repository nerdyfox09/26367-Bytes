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

        Pose2d beginPose = new Pose2d(60, -20, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .splineToLinearHeading(new Pose2d(55, -5, Math.toRadians(30)), Math.toRadians(0)) // pick up more balls

                        // Run outtake to shoot
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(1000);
                            myRobot.rightOuttakeMotor.setVelocity(1000);
                        })

                        .waitSeconds(1)

                        // Run intake to shoot
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))

                        .waitSeconds(3)

                        // Stop Outtake
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(0.0);
                            myRobot.rightOuttakeMotor.setVelocity(0.0);
                        })

                        // Stop intake
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))

                        .splineToLinearHeading(new Pose2d(33, -30, Math.toRadians(270)), Math.toRadians(180)) // pick up more balls

                        // Run intake at new location
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))

                        .strafeTo(new Vector2d(36,-48))

                        .waitSeconds(1)

                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))

                        .splineToLinearHeading(new Pose2d(58, -13, Math.toRadians(30)), Math.toRadians(20)) // drive back to line

                        // Spin up outtake flywheels
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(1000);
                            myRobot.rightOuttakeMotor.setVelocity(1000);
                        })

                        // Start intake to shoot
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))

                        .waitSeconds(3)

                        // Stop outtake
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(0.0);
                            myRobot.rightOuttakeMotor.setVelocity(0.0);
                        })

                        // Stop intake
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))

                        .lineToX(30)

                        .build());



        // store pose for use in later TeleOp modes
        BYTES_GLOBAL_Storage.currentPose = drive.localizer.getPose();
    }
}