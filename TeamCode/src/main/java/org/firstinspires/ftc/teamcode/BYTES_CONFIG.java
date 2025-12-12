package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BYTES_CONFIG {
    // debug or competition mode
    public static boolean MODE_DEBUG = false; // set to 'false' for COMPETITION

    // target April Tags for DECODE season 2025-2026
    public static int GAME_DECODE_TARGETS_RED_ID = 24;
    public static int GAME_DECODE_TARGETS_BLUE_ID = 20;

    // drive motors
    public static String HW_DRIVE_MOTORS_FRONT_LEFT = "motorFrontLeft";
    public static String HW_DRIVE_MOTORS_FRONT_RIGHT = "motorFrontRight";
    public static String HW_DRIVE_MOTORS_BACK_LEFT = "motorBackLeft";
    public static String HW_DRIVE_MOTORS_BACK_RIGHT = "motorBackRight";

    // drive encoders (deadwheels)
    public static String HW_DRIVE_ENCODERS_PARALLEL_LEFT = "motorFrontLeft";
    public static String HW_DRIVE_ENCODERS_PARALLEL_RIGHT = "motorBackRight";
    public static String HW_DRIVE_ENCODERS_PERPENDICULAR = "motorBackLeft";

    // intake motor
    public static String HW_INTAKE_MOTORS_PRIMARY = "intakeMotor";
    public static double HW_INTAKE_MOTORS_SPEED = 0.5;

    public static String HW_TRANSFER_SERVOS_RIGHT = "rightTransferServo";
    public static String HW_TRANSFER_SERVOS_LEFT = "leftTransferServo";

    // outtake motors
    public static String HW_OUTTAKE_MOTORS_LEFT = "leftOuttakeMotor";
    public static String HW_OUTTAKE_MOTORS_RIGHT = "rightOuttakeMotor";

    public static double HW_OUTTAKE_MOTORS_LEFT_VELOCITY_SHORT = 750;

    public static double HW_OUTTAKE_MOTORS_RIGHT_VELOCITY_SHORT = 750;

    public static double HW_OUTTAKE_MOTORS_SHORT_LOWER_VELOCITY = 860;
    public static double HW_OUTTAKE_MOTORS_SHORT_UPPER_VELOCITY = 890;


    // long shoot velocity: 875
    // long range: 140in
    // long bearing: 17.6 degrees
    public static double HW_OUTTAKE_MOTORS_LEFT_VELOCITY_LONG = 875;

    public static double HW_OUTTAKE_MOTORS_RIGHT_VELOCITY_LONG = 875;

    public static double HW_OUTTAKE_MOTORS_LONG_LOWER_VELOCITY = 860;
    public static double HW_OUTTAKE_MOTORS_LONG_UPPER_VELOCITY = 890;


    // sensors - gyro
    public static String HW_SENSORS_IMU_PRIMARY = "imu";

    // sensors - camera specs
    public static final CameraSpecs WEB_SIGHT = new CameraSpecs(
            "webSight", 4.0, 225, 941.178, 941.178, 484.54, 291.576 );

    public static final CameraSpecs FOX_RAY_VISION = new CameraSpecs(
            "foxRayVision", 4.0, 225, 827.558, 827.558, 348.813, 266.441 );

    public static final CameraSpecs FTC_DEFAULTS = new CameraSpecs(
            "ftcDefaults", 4.0, 225, 822.317, 822.317, 319.495, 242.502 );

    // sensors - camera currently installed on robot
    public static CameraSpecs ACTIVE_CAMERA = WEB_SIGHT;
    public static double HW_SENSORS_CAMERA_MAX_DETECTION_PERSISTENCE = 200; // in milliSec - how long to allow stale TagDetection to be valid

    public static double PARAMS_DRIVETRAIN_IN_PER_TICK = 0.00198432166862; // deadwheel measurement with forwardPushTest
    public static double PARAMS_DRIVETRAIN_LATERAL_IN_PER_TICK = 0.001492295191183421; // deadwheel measurement with lateralRampLogger
    public static double PARAMS_DRIVETRAIN_TRACK_WIDTH_TICKS = 7182.769805858175; // deadwheel measurement with angularRampLogger

    // feed forward parameters (in tick units)
    public static double PARAMS_DRIVETRAIN_kV = 0.0003752091615679999; // determined with forwardRampLogger
    public static double PARAMS_DRIVETRAIN_kS = 0.8900557107069424; // determined with forwardRampLogger
    public static double PARAMS_DRIVETRAIN_kA = 0.0001; // experimentally tuned with ManualFeedForwardTuner

    // path profile parameters (in inches)
    public static double PARAMS_DRIVETRAIN_MAX_WHEEL_VEL = 50;
    public static double PARAMS_DRIVETRAIN_MIN_PROFILE_ACCEL = -30;
    public static double PARAMS_DRIVETRAIN_MAX_PROFILE_ACCEL = 50;

    // turn profile parameters (in radians)
    public static double PARAMS_DRIVETRAIN_MAX_ANG_VEL = Math.PI; // shared with path
    public static double PARAMS_DRIVETRAIN_MAX_ANG_ACCEL = Math.PI;

    // path controller gains
    public static double PARAMS_DRIVETRAIN_AXIAL_GAIN = 14;
    public static double PARAMS_DRIVETRAIN_LATERAL_GAIN = 10;
    public static double PARAMS_DRIVETRAIN_HEADING_GAIN = 11; // shared with turn

    public static double PARAMS_DRIVETRAIN_AXIAL_VEL_GAIN = 0.0;
    public static double PARAMS_DRIVETRAIN_LATERAL_VEL_GAIN = 0.0;
    public static double PARAMS_DRIVETRAIN_HEADING_VEL_GAIN = 0.0; // shared with turn

    // THROTTLE CONTROL
    public static double PARAMS_DRIVETRAIN_MAX_POWER = 1.0; // decimal percentage
    public static double PARAMS_DRIVETRAIN_MAX_SPEED = 1.0; // multiplier for MAX_POWER

    // deadwheels
    public static double PARAMS_DEADWHEELS_PAR_0_TICKS = -3204.4471985017008; // y-pos of parallel encoder (in tick units) // measured with angularRampLogger
    public static double PARAMS_DEADWHEELS_PAR_1_TICKS = 3002.359352997628; // y-pos of parallel encoder (in tick units) // measured with angularRampLogger
    public static double PARAMS_DEADWHEELS_PERP_TICKS = -3278.8175652843615; // x-pos of perpendicular encoder (in tick units) // measured with angularRampLogger

    // controller - aim long - bearing
    public static PidConstants PARAMS_CTRL_AIM_LONG_TURN = new PidConstants(
            // limit integral sum to prevent runaway power when motors are maxed (should be around 1.0 / kI)
            // set maxDerivative to filter noise from camera sensor (initial value: 180 deg/s * 120%)
            // set kF to minimum power required to turn chassis (start with RR kS value)
            // alpha tunes low-pass filter on measurements - low value is more smoothing, but slower response
            0.03, 0, 0.0005, 0.09, 180, 200, 1, 0.8);
    public static double PARAMS_CTRL_AIM_LONG_TARGET_BEARING = 14.6; // degrees

    // controller - aim long - range
    public static PidConstants PARAMS_CTRL_AIM_LONG_DRIVE = new PidConstants(
            // limit integral sum to prevent runaway power when motors are maxed (should be around 1.0 / kI)
            // set maxDerivative to filter noise from camera sensor (initial value: 180 deg/s * 120%)
            // set kF to minimum power required to move chassis (start with RR kS value)
            // alpha tunes low-pass filter on measurements - high value is less smoothing, but faster response
            0.035, 0, 0.001, 0.087, 10, 200, 1, 0.65);
    public static double PARAMS_CTRL_AIM_LONG_TARGET_RANGE = 141.2; // inches

    // controller - aim short - bearing
    public static PidConstants PARAMS_CTRL_AIM_SHORT_TURN = new PidConstants(
            // limit integral sum to prevent runaway power when motors are maxed (should be around 1.0 / kI)
            // set maxDerivative to filter noise from camera sensor (initial value: 180 deg/s * 120%)
            // set kF to minimum power required to turn chassis (start with RR kS value)
            // alpha tunes low-pass filter on measurements - low value is more smoothing, but slower response
            0.04, 0.00015, 0.0005, 0.033, 180, 216, 1, 0.65);
    public static double PARAMS_CTRL_AIM_SHORT_TARGET_BEARING = 14.4; // degrees

    // controller - aim short - range
    public static PidConstants PARAMS_CTRL_AIM_SHORT_DRIVE = new PidConstants(
            // limit integral sum to prevent runaway power when motors are maxed (should be around 1.0 / kI)
            // set maxDerivative to filter noise from camera sensor (initial value: 180 deg/s * 120%)
            // set kF to minimum power required to move chassis (start with RR kS value)
            // alpha tunes low-pass filter on measurements - high value is less smoothing, but faster response
            0.06, 0, 0.0001, 0.00005, 10, 1000, 1, 0.8);
    public static double PARAMS_CTRL_AIM_SHORT_TARGET_RANGE = 88.4; // inches

    // *******************************************************
    // *****                                            ******
    // ***** NOTHING BELOW HERE SHOULD NEED TO BE TUNED ******
    // *****                                            ******
    // *******************************************************
    // sensors - configurable camera specs
    public static String HW_SENSORS_CAMERA_NAME = ACTIVE_CAMERA.NAME;
    public static double HW_SENSORS_CAMERA_EXPOSURE = ACTIVE_CAMERA.EXPOSURE;
    public static double HW_SENSORS_CAMERA_GAIN = ACTIVE_CAMERA.GAIN;
    public static double HW_SENSORS_CAMERA_FX = ACTIVE_CAMERA.FX;
    public static double HW_SENSORS_CAMERA_FY = ACTIVE_CAMERA.FY;
    public static double HW_SENSORS_CAMERA_CX = ACTIVE_CAMERA.CX;
    public static double HW_SENSORS_CAMERA_CY = ACTIVE_CAMERA.CY;

    public static class CameraSpecs {
        public String NAME;
        public double EXPOSURE;
        public double GAIN;
        public double FX;
        public double FY;
        public double CX;
        public double CY;

        public CameraSpecs(String name, double exposure, double gain, double fx, double fy, double cx, double cy) {
            this.NAME = name;
            this.EXPOSURE = exposure;
            this.GAIN = gain;
            this.FX = fx;
            this.FY = fy;
            this.CX = cx;
            this.CY = cy;
        }
    }

    public static class PidConstants {
        public double kP;
        public double kI;
        public double kD;
        public double kF;
        public double maxIntegralSum;
        public double maxDerivative;
        public double deadBand;
        public double alpha;

        public PidConstants(double kP, double kI, double kD, double kF, double maxIntegralSum, double maxDerivative, double deadBand, double alpha) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.maxIntegralSum = maxIntegralSum;
            this.maxDerivative = maxDerivative;
            this.deadBand = deadBand;
            this.alpha = alpha;
        }
    }
}