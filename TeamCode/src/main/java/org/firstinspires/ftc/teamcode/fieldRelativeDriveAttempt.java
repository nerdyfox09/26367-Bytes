
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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import java.lang.Math;

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
@TeleOp(name = "Field Relative Mecanum Drive")
public class fieldRelativeDriveAttempt extends OpMode {

    Bytes_Robot myRobot;

    @Override
    public void init() {
        // initialize myRobot
        myRobot = new Bytes_Robot(hardwareMap);

        // Pull initial pose from GlobalStorage. If no AutoOp has set the pose, it will
        // default to 0,0,0
        Pose2d initTelePose = BYTES_GLOBAL_Storage.currentPose;
        telemetry.addData("Initial TeleOp Pose: ", initTelePose);
        telemetry.addData("Drive Mode: ", myRobot.driveMode);
        telemetry.update();

        myRobot.localizer = new ThreeDeadWheelLocalizer(
                hardwareMap,
                BYTES_CONFIG.PARAMS.DRIVETRAIN.inPerTick,
                initTelePose);
    }

    @Override
    public void loop() {
        telemetry.addLine("A: reset Yaw");
        telemetry.addLine("Left Bumper: field-relative");
        telemetry.addLine("Right Bumper: robot-relative");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            // imu.resetYaw();

            // call private method in this class
            resetYaw(myRobot.localizer);
        }
        // If you press the left bumper, set drive mode to FIELD_RELATIVE
        if (gamepad1.left_bumper) {
            myRobot.driveMode = Bytes_Robot.DRIVE_MODE.FIELD_RELATIVE;
        }

        // If you press the right bumper, set drive mode to ROBOT_RELATIVE
        if (gamepad1.right_bumper) {
            myRobot.driveMode = Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE;
        }

        telemetry.addData("Drive Mode: ", myRobot.driveMode);
        telemetry.addData("Current Heading: ", Math.toDegrees(myRobot.localizer.getPose().heading.log()));

        telemetry.addData("drive Args(fwd):", -gamepad1.left_stick_y);
        telemetry.addData("drive Args(right):", gamepad1.left_stick_x);
        telemetry.addData("drive Args (rot):", gamepad1.right_stick_x);

        if (myRobot.driveMode == Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        updatePoseEstimate();
        telemetry.update();
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                myRobot.localizer.getPose().heading.log());
        //  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        myRobot.frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        myRobot.frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        myRobot.backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        myRobot.backRight.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void resetYaw(Localizer localizer) {
        Pose2d currentPose = localizer.getPose();

        // set new pose with heading of 0, but same x, y
        Pose2d newPose = new Pose2d(currentPose.position, 0);
        localizer.setPose(newPose);
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = myRobot.localizer.update();

        return vel;
    }

}
