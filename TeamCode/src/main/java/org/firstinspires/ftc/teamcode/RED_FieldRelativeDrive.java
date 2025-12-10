/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Bytes_PID_Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.Math;
import java.util.Locale;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@Config
@TeleOp(name = "RED Field Relative Drive", group = "Bytes")
public class RED_FieldRelativeDrive extends OpMode {

    Bytes_Robot myRobot;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagDetection desiredTag = null;
    private AprilTagDetection lastTag = null;
    private double lastTagTime = 0;

    private static final int DESIRED_TAG_ID = BYTES_CONFIG.GAME_DECODE_TARGETS_RED_ID;  // ID 24 = red target aprilTag

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet;

    @Override
    public void init() {
        // initialize myRobot
        myRobot = new Bytes_Robot(hardwareMap, telemetry);
        myRobot.initCamera();

        // Pull initial pose from GlobalStorage. If no AutoOp has set the pose, it will
        // default to 0,0,0
        Pose2d initTelePose = BYTES_GLOBAL_Storage.currentPose;
        telemetry.addData("Initial TeleOp Pose: ", initTelePose);
        telemetry.addData("Drive Mode: ", myRobot.driveMode);
        telemetry.update();

        myRobot.localizer = new ThreeDeadWheelLocalizer(
                hardwareMap,
                BYTES_CONFIG.PARAMS_DRIVETRAIN_IN_PER_TICK,
                initTelePose);
    }

    @Override
    public void loop() {
        this.packet = new TelemetryPacket();

        telemetry.addLine("Dpad Up: reset Yaw");
        telemetry.addLine("B: AIM short-range");
        telemetry.addLine("Y: AIM long-range");
        telemetry.addLine("X: ");
        telemetry.addLine("A: ");
        telemetry.addLine("Left Bumper: field-relative");
        telemetry.addLine("Right Bumper: robot-relative");
        telemetry.addLine("Left Trigger: Reverse intake");
        telemetry.addLine("Right Trigger: Intake");

        telemetry.addData("Time: ", System.currentTimeMillis()/1000);

        // check if desiredTag is in sight
        myRobot.webSight.update();
        desiredTag = null;
        desiredTag = myRobot.webSight.getTagBySpecificId(DESIRED_TAG_ID);
        if (desiredTag != null) {
            lastTag = desiredTag;
            lastTagTime = System.currentTimeMillis();

            telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.elevation));
        } else {
            telemetry.addLine("*** TAG NOT FOUND :( ***");
        }

        // ***************************************************************
        // *****                                                    ******
        // ***** SETTING UP GAMEPAD 1 - all functions work together ******
        // *****      --has sequencing--                            ******
        // ***************************************************************

        // If you press any dpad button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.dpad_up) {
            myRobot.resetYaw(myRobot.localizer);
        }

        if (gamepad1.dpad_down) {
            myRobot.resetYaw(myRobot.localizer);
        }

        if (gamepad1.dpad_left) {
            myRobot.resetYaw(myRobot.localizer);
        }

        if (gamepad1.dpad_right) {
            myRobot.resetYaw(myRobot.localizer);
        }

        // If you press x, set drive mode to FIELD_RELATIVE
        if (gamepad1.x) {
            myRobot.driveMode = Bytes_Robot.DRIVE_MODE.FIELD_RELATIVE;
        }

        // If you press b, set drive mode to ROBOT_RELATIVE
        if (gamepad1.b) {
            myRobot.driveMode = Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE;
        }

        //when the trigger is held down, the intake and transfer servos spin
        if (gamepad1.right_trigger > 0) {

            myRobot.leftTransferServo.setDirection(Servo.Direction.FORWARD);
            myRobot.rightTransferServo.setDirection(Servo.Direction.REVERSE);

            myRobot.leftTransferServo.setPosition(.75);
            myRobot.rightTransferServo.setPosition(.75);

            myRobot.intakeMotor.setPower(BYTES_CONFIG.HW_INTAKE_MOTORS_SPEED);

        }

        //when the trigger is let go of, the intake and transfer servos stop
         else if (gamepad1.right_trigger <= 0){
            myRobot.leftTransferServo.setPosition(.5);
            myRobot.rightTransferServo.setPosition(.5);

            myRobot.intakeMotor.setPower(0);
        }

         //while the bumper is pressed, the
        if (gamepad1.right_bumper) {
            myRobot.leftTransferServo.setDirection(Servo.Direction.REVERSE);
            myRobot.rightTransferServo.setDirection(Servo.Direction.FORWARD);

            myRobot.leftTransferServo.setPosition(.75);
            myRobot.rightTransferServo.setPosition(.75);

            myRobot.intakeMotor.setPower(-BYTES_CONFIG.HW_INTAKE_MOTORS_SPEED);

        }
        else if (!gamepad1.right_bumper){
            myRobot.leftTransferServo.setPosition(.5);
            myRobot.rightTransferServo.setPosition(.5);

            myRobot.intakeMotor.setPower(0);
        }

        //while the left trigger is held down, the outtake motors will spin up and the servos and
        //intake will also spin up to shoot balls for a long distance
        while (gamepad1.left_trigger > 0) {
            myRobot.leftOuttakeMotor.setVelocity(BYTES_CONFIG.HW_OUTTAKE_MOTORS_LEFT_VELOCITY_LONG);
            myRobot.rightOuttakeMotor.setVelocity(BYTES_CONFIG.HW_OUTTAKE_MOTORS_RIGHT_VELOCITY_LONG);

            myRobot.leftTransferServo.setDirection(Servo.Direction.FORWARD);
            myRobot.rightTransferServo.setDirection(Servo.Direction.REVERSE);

            myRobot.leftTransferServo.setPosition(.75);
            myRobot.rightTransferServo.setPosition(.75);

            myRobot.intakeMotor.setPower(BYTES_CONFIG.HW_INTAKE_MOTORS_SPEED);
        }

        //while the left bumper is held down, the outtake motors will spin up and the servos and
        //intake will also spin up to shoot balls for a short distance
        while(gamepad1.left_bumper){
            myRobot.leftOuttakeMotor.setVelocity(BYTES_CONFIG.HW_OUTTAKE_MOTORS_LEFT_VELOCITY_SHORT);
            myRobot.rightOuttakeMotor.setVelocity(BYTES_CONFIG.HW_OUTTAKE_MOTORS_RIGHT_VELOCITY_SHORT);

            myRobot.leftTransferServo.setDirection(Servo.Direction.FORWARD);
            myRobot.rightTransferServo.setDirection(Servo.Direction.REVERSE);

            myRobot.leftTransferServo.setPosition(.75);
            myRobot.rightTransferServo.setPosition(.75);

            myRobot.intakeMotor.setPower(BYTES_CONFIG.HW_INTAKE_MOTORS_SPEED);

        }

        if (gamepad1.a){
            // aim to target with LONG controllers and targets
            autoAimToTarget(BYTES_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_RANGE,
                    BYTES_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_BEARING,
                    myRobot.pidCtrlLongDrive,
                    myRobot.pidCtrlLongTurn);

            // clean up aimShort in case we just finished that
            myRobot.pidCtrlShortTurn.resetController();
            myRobot.pidCtrlShortDrive.resetController();

        } else if (gamepad1.y){
            // aim to target with SHORT controllers and targets
            autoAimToTarget(BYTES_CONFIG.PARAMS_CTRL_AIM_SHORT_TARGET_RANGE,
                    BYTES_CONFIG.PARAMS_CTRL_AIM_SHORT_TARGET_BEARING,
                    myRobot.pidCtrlShortDrive,
                    myRobot.pidCtrlShortTurn);

            // clean up aimLong in case we just finished that
            myRobot.pidCtrlLongTurn.resetController();
            myRobot.pidCtrlLongDrive.resetController();

        } else if (!gamepad1.y && !gamepad1.a) {
            // no auto aim in progress, just drive with joystick
            // first we clean up all autoAim controllers
            myRobot.pidCtrlLongDrive.resetController();
            myRobot.pidCtrlLongTurn.resetController();
            myRobot.pidCtrlShortDrive.resetController();
            myRobot.pidCtrlShortTurn.resetController();

            telemetry.addData("Drive Mode: ", myRobot.driveMode);
            telemetry.addData("Current Heading: ", Math.toDegrees(myRobot.localizer.getPose().heading.log()));

            telemetry.addData("drive Args(fwd):", -gamepad1.right_stick_y);
            telemetry.addData("drive Args(right):", gamepad1.right_stick_x);
            telemetry.addData("drive Args (rot):", gamepad1.left_stick_x);

            if (myRobot.driveMode == Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE) {
                myRobot.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            } else {
                myRobot.driveFieldRelative(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            }

            myRobot.updatePoseEstimate();
        }

        // *******************************************************************
        // *****                                                        ******
        // ***** SETTING UP GAMEPAD 2 - all functions run separately    ******
        // *****                                                        ******
        // *******************************************************************

        // If you press dpad up or down, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad2.dpad_up) {
            myRobot.resetYaw(myRobot.localizer);
        }

        if (gamepad2.dpad_down) {
            myRobot.resetYaw(myRobot.localizer);
        }

        //if you press dpad left, then the drive mode goes to field relative
        if (gamepad2.dpad_left) {
            myRobot.driveMode = Bytes_Robot.DRIVE_MODE.FIELD_RELATIVE;        }

        //if you press dpad right, then the drive mode goes to robot relative
        if (gamepad2.dpad_right) {
            myRobot.driveMode = Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE;        }

        // If you press x, spin up the flywheels to a long shot velocity
        if (gamepad2.x) {
            myRobot.leftOuttakeMotor.setVelocity(BYTES_CONFIG.HW_OUTTAKE_MOTORS_LEFT_VELOCITY_LONG);
            myRobot.rightOuttakeMotor.setVelocity(BYTES_CONFIG.HW_OUTTAKE_MOTORS_RIGHT_VELOCITY_LONG);
        }

        // If you press b, spin up the flywheels to a short shot velocity
        else if (gamepad2.b) {
            myRobot.leftOuttakeMotor.setVelocity(BYTES_CONFIG.HW_OUTTAKE_MOTORS_LEFT_VELOCITY_SHORT);
            myRobot.rightOuttakeMotor.setVelocity(BYTES_CONFIG.HW_OUTTAKE_MOTORS_RIGHT_VELOCITY_SHORT);
        }
        else if (!gamepad1.x && !gamepad1.b) {
            myRobot.leftOuttakeMotor.setVelocity(0);
            myRobot.rightOuttakeMotor.setVelocity(0);
        }

        //when the right trigger is held down, the intake spins
        if (gamepad2.right_trigger > 0) {
            myRobot.intakeMotor.setPower(BYTES_CONFIG.HW_INTAKE_MOTORS_SPEED);
        }

        //when the trigger is let go of, the intake stops
        else if (gamepad2.right_trigger <= 0){
            myRobot.intakeMotor.setPower(0);
        }

        //while the right bumper is pressed, the intake spins backwards
        if (gamepad2.right_bumper) {
           myRobot.intakeMotor.setPower(-BYTES_CONFIG.HW_INTAKE_MOTORS_SPEED);

        }
        else if (!gamepad2.right_bumper){
            myRobot.intakeMotor.setPower(0);
        }

        //while the left trigger is held down, the transfer servos will spin
        while (gamepad2.left_trigger > 0) {
            myRobot.leftTransferServo.setDirection(Servo.Direction.FORWARD);
            myRobot.rightTransferServo.setDirection(Servo.Direction.REVERSE);

            myRobot.leftTransferServo.setPosition(.75);
            myRobot.rightTransferServo.setPosition(.75);
        }

        //while the left bumper is held down, the transfer servos will spin backwards
        while(gamepad2.left_bumper){
            myRobot.leftTransferServo.setDirection(Servo.Direction.REVERSE);
            myRobot.rightTransferServo.setDirection(Servo.Direction.FORWARD);

            myRobot.leftTransferServo.setPosition(.75);
            myRobot.rightTransferServo.setPosition(.75);

        }

        if (gamepad1.a){
            // aim to target with LONG controllers and targets
            autoAimToTarget(BYTES_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_RANGE,
                    BYTES_CONFIG.PARAMS_CTRL_AIM_LONG_TARGET_BEARING,
                    myRobot.pidCtrlLongDrive,
                    myRobot.pidCtrlLongTurn);

            // clean up aimShort in case we just finished that
            myRobot.pidCtrlShortTurn.resetController();
            myRobot.pidCtrlShortDrive.resetController();

        } else if (gamepad1.y){
            // aim to target with SHORT controllers and targets
            autoAimToTarget(BYTES_CONFIG.PARAMS_CTRL_AIM_SHORT_TARGET_RANGE,
                    BYTES_CONFIG.PARAMS_CTRL_AIM_SHORT_TARGET_BEARING,
                    myRobot.pidCtrlShortDrive,
                    myRobot.pidCtrlShortTurn);

            // clean up aimLong in case we just finished that
            myRobot.pidCtrlLongTurn.resetController();
            myRobot.pidCtrlLongDrive.resetController();

        } else if (!gamepad1.y && !gamepad1.a) {
            // no auto aim in progress, just drive with joystick
            // first we clean up all autoAim controllers
            myRobot.pidCtrlLongDrive.resetController();
            myRobot.pidCtrlLongTurn.resetController();
            myRobot.pidCtrlShortDrive.resetController();
            myRobot.pidCtrlShortTurn.resetController();

            telemetry.addData("Drive Mode: ", myRobot.driveMode);
            telemetry.addData("Current Heading: ", Math.toDegrees(myRobot.localizer.getPose().heading.log()));

            telemetry.addData("drive Args(fwd):", -gamepad1.right_stick_y);
            telemetry.addData("drive Args(right):", gamepad1.right_stick_x);
            telemetry.addData("drive Args (rot):", gamepad1.left_stick_x);

            if (myRobot.driveMode == Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE) {
                myRobot.drive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            } else {
                myRobot.driveFieldRelative(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            }

            myRobot.updatePoseEstimate();
        }

        telemetry.update();
        dashboard.sendTelemetryPacket(this.packet);
    }

    private void autoAimToTarget(double targetRange,
                                 double targetBearing,
                                 Bytes_PID_Controller pidCtrlRange,
                                 Bytes_PID_Controller pidCtrlBearing) {


        AprilTagDetection activeTag = this.desiredTag;
        boolean usingStaleData = false;
        double timeElapsedSinceGoodTag = System.currentTimeMillis() - this.lastTagTime;
        double maxDetectionPersistence = BYTES_CONFIG.HW_SENSORS_CAMERA_MAX_DETECTION_PERSISTENCE;
        double decayFactor = 1.0; // no effect if set to 1.0

        // Use lastTag if desiredTag is null AND lastTag is recent
        // We'll trust lastTag for up to 50ms
        if (activeTag == null && this.lastTag != null &&
                (timeElapsedSinceGoodTag < maxDetectionPersistence )) {
            activeTag = this.lastTag;
            telemetry.addLine("*** Using LAST SEEN TAG DATA ***");
            usingStaleData = true;
        }

        if (activeTag != null) {

            // The reported bearing value is our current angular position relative to the tag
            double currentBearing = activeTag.ftcPose.bearing;

            // output telemetry to driver station
            telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", activeTag.id, activeTag.metadata.name));
            telemetry.addLine(String.format(Locale.US,"R/B %6.1f %6.1f  (inch, deg)", activeTag.ftcPose.range, activeTag.ftcPose.bearing));

            // PID to target bearing
            double turnPower = pidCtrlBearing.getNewPower(currentBearing, targetBearing, true, packet);

            // The reported range value is our current y position relative to the tag
            double currentRange = activeTag.ftcPose.range;

            // PID to target range
            double drivePower = pidCtrlRange.getNewPower(currentRange, targetRange, true, packet);

            // if using staleDetection, apply decay factor
            if (usingStaleData) {
                // set decay factor to go from 1.0 when timeElapsed = 0 down to
                //   0.0 when timeElapsed = maxPersistence allowed
                decayFactor = Math.max(0.0, 1.0-(timeElapsedSinceGoodTag/maxDetectionPersistence));
                drivePower = drivePower * decayFactor;
                turnPower = turnPower * decayFactor;

                // telemetry
                packet.put("decayFactor", decayFactor);
                packet.put("drivePowerDecayed", drivePower);
                packet.put("turnPowerDecayed", turnPower);
            }

            // update motor powers (we flip sign on turn power to turn against the error)
            myRobot.drive(-drivePower, 0, -turnPower);

            myRobot.updatePoseEstimate();
        }
        else {
            // If we have NO tag (neither desired nor last), the robot should stop
            myRobot.drive(0, 0, 0);
            telemetry.addLine("****** AIM ERROR: No aprilTag found! :( ******");
        }
    }
}