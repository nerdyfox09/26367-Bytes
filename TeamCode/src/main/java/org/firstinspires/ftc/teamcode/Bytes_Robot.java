
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Bytes_Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Bytes_PID_Controller;


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
    public MecanumDrive drive;

    // Webcam
    public CameraName webCamName;
    public Bytes_Camera webSight;

    // PID turn controller
    public Bytes_PID_Controller pidCtrlLongTurn;
    public Bytes_PID_Controller pidCtrlShortTurn;

    // PID drive controller
    public Bytes_PID_Controller pidCtrlLongDrive;
    public Bytes_PID_Controller pidCtrlShortDrive;


    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public enum DRIVE_MODE {
        ROBOT_RELATIVE,
        FIELD_RELATIVE
    }

    public DRIVE_MODE driveMode = DRIVE_MODE.FIELD_RELATIVE;

    // This declares the IMU needed to get the current direction the robot is facing
    //IMU imu;

    public Bytes_Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        // show the current FOXY_CONFIG mode
        String strConfigMode = BYTES_CONFIG.MODE_DEBUG ? "DEBUG" : "COMPETITION";
        if (BYTES_CONFIG.MODE_DEBUG) {
            telemetry.addLine("********* WARNING *********");
            telemetry.addLine("********* WARNING *********");
        }
        telemetry.addLine("*** Config Mode is: " + strConfigMode + " ***");

        // initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HW_DRIVE_MOTORS_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HW_DRIVE_MOTORS_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HW_DRIVE_MOTORS_BACK_LEFT);
        backRight = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HW_DRIVE_MOTORS_BACK_RIGHT);

        leftOuttakeMotor = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HW_OUTTAKE_MOTORS_LEFT);
        rightOuttakeMotor = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HW_OUTTAKE_MOTORS_RIGHT);

        intakeMotor = hardwareMap.get(DcMotorEx.class, BYTES_CONFIG.HW_INTAKE_MOTORS_PRIMARY);

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

        // get camera
        webCamName = hardwareMap.get(CameraName.class, BYTES_CONFIG.HW_SENSORS_CAMERA_NAME);

        // initialize PID Controllers
        pidCtrlLongTurn = new Bytes_PID_Controller(BYTES_CONFIG.PARAMS_CTRL_AIM_LONG_TURN);
        pidCtrlLongDrive = new Bytes_PID_Controller(BYTES_CONFIG.PARAMS_CTRL_AIM_LONG_DRIVE);
        pidCtrlShortTurn = new Bytes_PID_Controller(BYTES_CONFIG.PARAMS_CTRL_AIM_SHORT_TURN);
        pidCtrlShortDrive = new Bytes_PID_Controller(BYTES_CONFIG.PARAMS_CTRL_AIM_SHORT_DRIVE);

        // store hwMap and telemetry
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void initCamera() {
        this.webSight = new Bytes_Camera();
        this.webSight.init(this.webCamName, this.hardwareMap, this.telemetry);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = BYTES_CONFIG.PARAMS_DRIVETRAIN_MAX_POWER;
        double maxSpeed = BYTES_CONFIG.PARAMS_DRIVETRAIN_MAX_SPEED;  // make this slower for outreaches

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
        this.frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        this.frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        this.backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        this.backRight.setPower(maxSpeed * (backRightPower / maxPower));
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                this.localizer.getPose().heading.log());
        //  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void resetYaw(Localizer localizer) {
        Pose2d currentPose = localizer.getPose();

        // set new pose with heading of 0, but same x, y
        Pose2d newPose = new Pose2d(currentPose.position, 0);
        localizer.setPose(newPose);
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = this.localizer.update();

        return vel;
    }
}
