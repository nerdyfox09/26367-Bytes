package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BYTES_GLOBAL_Storage;
import org.firstinspires.ftc.teamcode.Bytes_Robot;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous (name = "Drive Out Auto")
public class DriveOutAuto extends LinearOpMode {

    Bytes_Robot myRobot;

    @Override
    public void runOpMode() throws InterruptedException {
        myRobot = new Bytes_Robot(hardwareMap, telemetry);

        Pose2d beginPose = new Pose2d(60, -15,Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToX(30)
                        .build());



        // store pose for use in later TeleOp modes
        BYTES_GLOBAL_Storage.currentPose = drive.localizer.getPose();
    }
}