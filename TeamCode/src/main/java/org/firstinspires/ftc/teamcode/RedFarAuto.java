package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Far Auto")
public class RedFarAuto extends LinearOpMode {

    Bytes_Robot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Bytes_Robot(hardwareMap);

        // Mirror across field centerline (y changes sign)
        Pose2d beginPose = new Pose2d(65, 22, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .splineToLinearHeading(new Pose2d(60, 15, Math.toRadians(30)), Math.toRadians(0)) // pick up more balls
                        // Spin up outtake
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

                        // Mirror the pickup path across Y-axis (negative Y becomes positive)
                        .splineToLinearHeading(new Pose2d(37, 30, Math.toRadians(90)), Math.toRadians(0)) // pick up more balls

                        // Run intake at new location
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))

                        .strafeTo(new Vector2d(36, 48))

                        .waitSeconds(1)

                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))

                        // Return mirrored path
                        .splineToLinearHeading(new Pose2d(58, 13, Math.toRadians(-42)), Math.toRadians(-20))

                        // Spin up outtake flywheels
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(1000);
                            myRobot.rightOuttakeMotor.setVelocity(1000);
                        })

                        .waitSeconds(1)

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

                        .build()
        );

        // Save pose for TeleOp
        BYTES_GLOBAL_Storage.currentPose = drive.localizer.getPose();
    }
}
