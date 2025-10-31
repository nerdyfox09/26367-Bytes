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

        interface Sensors {
            interface IMU {
                String PRIMARY = "imu";
            }
        }


    }
    interface PARAMS {
        interface DRIVETRAIN {
            public double inPerTick = 0.00198432166862;
            public double lateralInPerTick = 0.001492295191183421;
            public double trackWidthTicks = 7182.769805858175;

            // feedforward parameters (in tick units)
            public double kS = 0.8900557107069424;
            public double kV = 0.0003752091615679999;
            public double kA = 0.0001;

            // path profile parameters (in inches)
            public double maxWheelVel = 50;
            public double minProfileAccel = -30;
            public double maxProfileAccel = 50;

            // turn profile parameters (in radians)
            public double maxAngVel = Math.PI; // shared with path
            public double maxAngAccel = Math.PI;

            // path controller gains
            public double axialGain = 14.0;
            public double lateralGain = 10.0;
            public double headingGain = 11.0; // shared with turn

            public double axialVelGain = 0.0;
            public double lateralVelGain = 0.0;
            public double headingVelGain = 0.0; // shared with turn
        }

            // THROTTLE CONTROL
            double MAX_POWER = 1.0; // decimal percentage
            double MAX_SPEED = 1.0; // multiplier for MAX_POWER
        }
        interface DEADWHEELS {
            public double PAR_0_TICKS = -3204.4471985017008; // y position of the first parallel encoder (in tick units)
            public double PAR_1_TICKS = 3002.359352997628; // y position of the second parallel encoder (in tick units)
            public double PERP_Ticks = -3278.8175652843615; // x position of the perpendicular encoder (in tick units)


        }
    }

