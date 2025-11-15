package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Close Auto")
public class BlueCloseAuto extends LinearOpMode {

    Bytes_Robot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Bytes_Robot(hardwareMap);

        // Mirrored across y = 0 (blue side)
        Pose2d beginPose = new Pose2d(-62, -36, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(-3, -5, Math.toRadians(45)), Math.toRadians(20))
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(900);
                            myRobot.rightOuttakeMotor.setVelocity(900);
                        })
                        .waitSeconds(1)
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))
                        .waitSeconds(3)
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(0.0);
                            myRobot.rightOuttakeMotor.setVelocity(0.0);
                        })
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))
                        .splineToLinearHeading(new Pose2d(-11, -30, Math.toRadians(-90)), Math.toRadians(0))
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))
                        .strafeTo(new Vector2d(-11, -52))
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(0.0))
                        .splineToLinearHeading(new Pose2d(-3, -53, Math.toRadians(45)), Math.toRadians(-45))
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(900);
                            myRobot.rightOuttakeMotor.setVelocity(900);
                        })
                        .waitSeconds(1)
                        .stopAndAdd(() -> myRobot.intakeMotor.setPower(1.0))
                        .waitSeconds(3)
                        .stopAndAdd(() -> {
                            myRobot.leftOuttakeMotor.setVelocity(0.0);
                            myRobot.rightOuttakeMotor.setVelocity(0.0);
                        })
                        .strafeTo(new Vector2d(20, 0))
                        .build()
        );

        BYTES_GLOBAL_Storage.currentPose = drive.localizer.getPose();
    }
}
