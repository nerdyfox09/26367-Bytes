
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Bytes_Robot {

    // This declares the four motors needed
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx intakeMotor;
    public DcMotorEx rightOuttakeMotor;
    public DcMotorEx leftOuttakeMotor;

    // RoadRunner localizer for odometry
    public ThreeDeadWheelLocalizer localizer;

    public enum DRIVE_MODE {
        ROBOT_RELATIVE,
        FIELD_RELATIVE;
    }

    public DRIVE_MODE driveMode = DRIVE_MODE.FIELD_RELATIVE;

    // This declares the IMU needed to get the current direction the robot is facing
    //IMU imu;

    public Bytes_Robot(HardwareMap hardwareMap) {

        // initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.BACK_LEFT);
        backRight = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HARDWARE.DRIVETRAIN.MOTORS.BACK_RIGHT);

        leftOuttakeMotor = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HARDWARE.OUTTAKE.MOTORS.LEFT);
        rightOuttakeMotor = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HARDWARE.OUTTAKE.MOTORS.RIGHT);

        intakeMotor = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HARDWARE.INTAKE.MOTORS.PRIMARY);

        // set brake mode on for firm stopping behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We set some motors in reverse which is needed for drive trains where some motors
        // are running in the opposite direction
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightOuttakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Set run modes
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftOuttakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOuttakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
