package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Relative Drive")
public class FieldRelativeDrive extends OpMode {

    private Bytes_Robot myRobot;

    private DcMotorEx intakeMotor;
    private DcMotorEx leftOuttakeMotor;
    private DcMotorEx rightOuttakeMotor;

    @Override
    public void init() {
        // Initialize robot and pose
        myRobot = new Bytes_Robot(hardwareMap);
        Pose2d initTelePose = BYTES_GLOBAL_Storage.currentPose;
        telemetry.addData("Initial TeleOp Pose: ", initTelePose);
        telemetry.update();

        myRobot.localizer = new ThreeDeadWheelLocalizer(
                hardwareMap,
                BYTES_CONFIG.PARAMS.DRIVETRAIN.inPerTick,
                initTelePose
        );

        // Initialize intake/outtake hardware
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftOuttakeMotor = hardwareMap.get(DcMotorEx.class, "leftOuttakeMotor");
        rightOuttakeMotor = hardwareMap.get(DcMotorEx.class, "rightOuttakeMotor");

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightOuttakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftOuttakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOuttakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftOuttakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightOuttakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        myRobot.driveMode = Bytes_Robot.DRIVE_MODE.FIELD_RELATIVE;
    }

    @Override
    public void loop() {
        telemetry.addLine("D-pad up: reset yaw");
        telemetry.addLine("Left bumper: field-relative");
        telemetry.addLine("Right bumper: robot-relative");

        // --- Drive mode switching ---
        if (gamepad1.left_bumper) myRobot.driveMode = Bytes_Robot.DRIVE_MODE.FIELD_RELATIVE;
        if (gamepad1.right_bumper) myRobot.driveMode = Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE;
        if (gamepad1.dpad_up) resetYaw(myRobot.localizer);

        // --- Driving ---
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (myRobot.driveMode == Bytes_Robot.DRIVE_MODE.ROBOT_RELATIVE) {
            drive(forward, right, rotate);
        } else {
            driveFieldRelative(forward, right, rotate);
        }

        updatePoseEstimate();

        // --- Intake control ---
        if (gamepad1.left_trigger > 0) {
            intakeMotor.setPower(-1.0);
        } else if (gamepad1.right_trigger > 0) {
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        // --- Outtake (flywheel) control ---
        if (gamepad1.a) { // far shot
            leftOuttakeMotor.setVelocity(1150);
            rightOuttakeMotor.setVelocity(1150);
        } else if (gamepad1.b) { // stop
            leftOuttakeMotor.setVelocity(0.0);
            rightOuttakeMotor.setVelocity(0.0);
        } else if (gamepad1.y) { // close shot
            leftOuttakeMotor.setVelocity(3);
            rightOuttakeMotor.setVelocity(3);
        }

        // --- Telemetry ---
        telemetry.addData("Drive Mode", myRobot.driveMode);
        telemetry.addData("Heading (deg)", Math.toDegrees(myRobot.localizer.getPose().heading.log()));
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Outtake L/R", "%4.2f / %4.2f",
                leftOuttakeMotor.getPower(), rightOuttakeMotor.getPower());
        telemetry.update();
    }

    // --- Helper methods ---

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                myRobot.localizer.getPose().heading.log());

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));

        myRobot.frontLeft.setPower(fl / max);
        myRobot.frontRight.setPower(fr / max);
        myRobot.backLeft.setPower(bl / max);
        myRobot.backRight.setPower(br / max);
    }

    private void resetYaw(Localizer localizer) {
        Pose2d currentPose = localizer.getPose();
        Pose2d newPose = new Pose2d(currentPose.position, 0);
        localizer.setPose(newPose);
    }

    private PoseVelocity2d updatePoseEstimate() {
        return myRobot.localizer.update();
    }
}
