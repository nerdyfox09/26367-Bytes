package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Encoder Test")
public class EncoderTest extends OpMode {

    private Bytes_Robot myRobot;
    public void init() {
        // Initialize robot and pose
        myRobot = new Bytes_Robot(hardwareMap);
    }

    @Override
    public void loop() {
        double targetVelocity = 0;

        // Check if the 'A' button on gamepad 1 is being held
        if (gamepad1.a) {
            // Set the target velocity to your test value
            targetVelocity = 1000;
        } else {
            // Stop the motors if 'A' is not pressed
            targetVelocity = 0;
        }

        // Send the velocity command to the motors
        myRobot.leftOuttakeMotor.setVelocity(targetVelocity);
        myRobot.rightOuttakeMotor.setVelocity(targetVelocity);

        // --- TELEMETRY ---
        // This is the core of the test.
        // We output the target velocity and the *current* encoder position.
        telemetry.addData("--- Motor Test ---", "");
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData(">", " ");
        telemetry.addData("Left Encoder", myRobot.leftOuttakeMotor.getCurrentPosition());
        telemetry.addData("Right Encoder", myRobot.rightOuttakeMotor.getCurrentPosition());
        telemetry.addData(">", " ");
        telemetry.addData("Left Velocity (Read)", myRobot.leftOuttakeMotor.getVelocity());
        telemetry.addData("Right Velocity (Read)", myRobot.rightOuttakeMotor.getVelocity());

        // Update the telemetry on the Driver Station
        telemetry.update();

    // Stop motors when the OpMode is stopped
        myRobot.leftOuttakeMotor.setVelocity(0);
        myRobot.rightOuttakeMotor.setVelocity(0);
}
    }

