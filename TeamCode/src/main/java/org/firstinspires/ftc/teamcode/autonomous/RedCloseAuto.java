package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BYTES_GLOBAL_Storage;
import org.firstinspires.ftc.teamcode.Bytes_Robot;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Red Close Auto")
public class RedCloseAuto extends LinearOpMode {

    Bytes_Robot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Bytes_Robot(hardwareMap, telemetry);

        // Mirror across field centerline (y changes sign)
        Pose2d beginPose = new Pose2d(-62, 36, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(315)), Math.toRadians(-20))
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(750);
                            myRobot.rightOuttakeMotor.setVelocity(750);
                        })

                        .waitSeconds(1)

                        // Run intake to shoot
                        .stopAndAdd(() -> {
                            myRobot.intakeMotor.setPower(1);
                            myRobot.rightTransferServo.setPosition(0.75);
                            myRobot.leftTransferServo.setPosition(0.75);

                        })

                        .waitSeconds(3)

                        // Stop Outtake
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(0.0);
                            myRobot.rightOuttakeMotor.setVelocity(0.0);
                        })

                        // Stop intake
                        .stopAndAdd(() -> {
                            myRobot.intakeMotor.setPower(0);
                            myRobot.rightTransferServo.setPosition(0.5);
                            myRobot.leftTransferServo.setPosition(0.5);

                        })
                        .splineToLinearHeading(new Pose2d(-11, 30, Math.toRadians(90)), Math.toRadians(0))
                        // Run intake to shoot
                        .stopAndAdd(() -> {
                            myRobot.intakeMotor.setPower(1);
                            myRobot.rightTransferServo.setPosition(0.75);
                            myRobot.leftTransferServo.setPosition(0.75);

                        })

                        .strafeTo(new Vector2d(-11, 52))
                        .stopAndAdd(() -> {
                            myRobot.intakeMotor.setPower(0);
                            myRobot.rightTransferServo.setPosition(0.5);
                            myRobot.leftTransferServo.setPosition(0.5);

                        })
                        .splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(315)), Math.toRadians(45))
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(750);
                            myRobot.rightOuttakeMotor.setVelocity(750);
                        })

                        .waitSeconds(1)

                        // Run intake to shoot
                        .stopAndAdd(() -> {
                            myRobot.intakeMotor.setPower(1);
                            myRobot.rightTransferServo.setPosition(0.75);
                            myRobot.leftTransferServo.setPosition(0.75);

                        })

                        .waitSeconds(3)

                        // Stop Outtake
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(0.0);
                            myRobot.rightOuttakeMotor.setVelocity(0.0);

                                myRobot.intakeMotor.setPower(0);
                                myRobot.rightTransferServo.setPosition(0.5);
                                myRobot.leftTransferServo.setPosition(0.5);


                        })
                        .strafeTo(new Vector2d(20, 0))
                        .build()
        );

        // Save pose for TeleOp
        BYTES_GLOBAL_Storage.currentPose = drive.localizer.getPose();
    }
}
