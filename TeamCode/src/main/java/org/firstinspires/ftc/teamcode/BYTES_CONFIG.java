package org.firstinspires.ftc.teamcode;

public class BYTES_CONFIG {
    // drive motors
    interface HARDWARE {
        interface DRIVETRAIN {
            interface MOTORS {
                String FRONT_LEFT = "motorFrontLeft";
                String FRONT_RIGHT = "motorFrontRight";
                String BACK_LEFT = "motorBackLeft";
                String BACK_RIGHT = "motorBackRight";
            }
            interface Encoders {
                String PARALLEL_LEFT = "motorFrontLeft";
                String PARALLEL_RIGHT = "motorBackRight";
                String PERPENDICULAR = "motorBackLeft";
            }
        }
        interface INTAKE {
            interface MOTORS {
                String PRIMARY = "intakeMotor";
            }
        }
        interface OUTTAKE {
            interface MOTORS {
                String LEFT = "leftOuttakeMotor";
                String RIGHT = "rightOuttakeMotor";
            }
        }

        interface TRANSFER {
            interface SERVOS {
                String PRIMARY = "transferServo";
            }
        }

        interface Sensors {
            interface IMU {
                String PRIMARY = "imu";
            }
        }


    }
    interface PARAMS {
        interface DRIVETRAIN {
            double IN_PER_TICK = 0.0019575404;
            double MAX_POWER = 1.0; // decimal percentage
            double MAX_SPEED = 1.0; // multiplier for MAX_POWER

        }
    }
}
