package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * This OpMode is for tuning the P-gain of a flywheel to make it stop faster.
 *
 * HOW TO USE:
 * 1. Deploy and run this OpMode.
 * 2. The telemetry will show the current P, I, D, and F values.
 * 3. Hold 'A' to spin the flywheels up to TEST_VELOCITY.
 * 4. Release 'A' to command the flywheels to stop (setVelocity(0)).
 * 5. OBSERVE how long it takes to stop.
 *
 * 6. Press D-PAD UP to increase the P gain.
 * 7. Press D-PAD DOWN to decrease the P gain.
 * 8. Press 'Y' to send the new P value (and default I,D,F) to the motor controllers.
 * 9. Repeat steps 3-5.
 *
 * 10. Find the 'P' value that stops the motor quickly without violent
 * oscillation or a loud "BUZZ" (which means it's over-correcting).
 *
 * 11. Once you find your magic 'P' value, write it down! You will use this
 * value in your main robot code.
 */
@TeleOp(name = "Flywheel PID Tuner", group = "Tests")
public class FlywheelPIDTuner extends LinearOpMode {

    Bytes_Robot myRobot;

    // --- YOUR SETTINGS ---
    private static final double TEST_VELOCITY = 1150;
    // Increment for tuning P. Start with a larger value.
    // If 0.5 is too much, try 0.1 or 0.05.
    private static final double P_INCREMENT = 0.5;

    // We will get the default F from the controller.
    // We will set I and D to 0 for velocity control.
    private double currentP;
    private double currentI = 0.0;
    private double currentD = 0.0;
    private double currentF; // Will be read from the motor

    // Variables for button edge detection
    private boolean yWasPressed = false;
    private boolean yJustPressed = false;
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;

    @Override
    public void runOpMode() {

        // --- HARDWARE MAPPING ---
        try {
            myRobot = new Bytes_Robot(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Error", "Hardware class not found or init failed.");
            telemetry.addData("Message", e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        // --- MOTOR CONFIGURATION ---
        // IMPORTANT: Set your directions correctly!
        myRobot.leftOuttakeMotor.setDirection(DcMotor.Direction.FORWARD);
        myRobot.rightOuttakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set BRAKE mode (good for when OpMode stops)
        myRobot.leftOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myRobot.rightOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and set mode
        myRobot.leftOuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.rightOuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.leftOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myRobot.rightOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- GET DEFAULT PIDF ---
        // This is the most important step! We need the default F-gain.
        PIDFCoefficients defaultPIDF = myRobot.leftOuttakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Use the controller's default P and F as our starting point
        currentP = defaultPIDF.p;
        currentF = defaultPIDF.f;
        // We will force I and D to 0, as they are bad for flywheel velocity.
        currentI = 0.0;
        currentD = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start.");
        telemetry.addData(">", " ");
        telemetry.addData("Default P", defaultPIDF.p);
        telemetry.addData("Default I", defaultPIDF.i);
        telemetry.addData("Default D", defaultPIDF.d);
        telemetry.addData("Default F", defaultPIDF.f);
        telemetry.addData(">", "We will use P=%.3f, I=0, D=0, F=%.3f", currentP, currentF);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Button Edge Detection Logic ---
            yJustPressed = false; // Reset flag each loop
            if (gamepad1.y && !yWasPressed) {
                yJustPressed = true;
            }
            yWasPressed = gamepad1.y;

            if (gamepad1.dpad_up && !dpadUpWasPressed) {
                currentP += P_INCREMENT;
            }
            dpadUpWasPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownWasPressed) {
                if (currentP >= P_INCREMENT) { // Prevent going below 0
                    currentP -= P_INCREMENT;
                } else {
                    currentP = 0;
                }
            }
            dpadDownWasPressed = gamepad1.dpad_down;


            // --- Apply New PIDF ---
            if (yJustPressed) {
                PIDFCoefficients newPIDF = new PIDFCoefficients(currentP, currentI, currentD, currentF);
                myRobot.leftOuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
                myRobot.rightOuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

                // Add a blocking message to confirm it was sent
                telemetry.clearAll();
                telemetry.addData("!!!", "PIDF VALUES APPLIED!");
                telemetry.addData("P", "%.3f", currentP);
                telemetry.addData("I", "%.3f", currentI);
                telemetry.addData("D", "%.3f", currentD);
                telemetry.addData("F", "%.3f", currentF);
                telemetry.update();
                sleep(500); // Pause for 0.5s to show this message
            }

            // --- Motor Control ---
            if (gamepad1.a) {
                // Run flywheels at test velocity
                myRobot.leftOuttakeMotor.setVelocity(TEST_VELOCITY);
                myRobot.rightOuttakeMotor.setVelocity(TEST_VELOCITY);
            } else {
                // Command stop
                // --- NEW THEORY ---
                // We will command a NEGATIVE velocity.
                // This forces the PID controller to see a massive error
                // (e.g., Target: -100, Actual: 1150, Error: -1250).
                // A high P-gain will multiply this huge negative error
                // and apply MAXIMUM reverse power, braking the motor.
                myRobot.rightOuttakeMotor.setVelocity(-100);
                myRobot.leftOuttakeMotor.setVelocity(-100);
            }

            // --- TELEMETRY ---
            telemetry.addData("--- CONTROLS ---", "");
            telemetry.addData("Hold (A)", "Run Flywheels");
            telemetry.addData("Release (A)", "STOP Flywheels");
            telemetry.addData("D-Pad Up/Down", "Change P Gain");
            telemetry.addData("Press (Y)", "APPLY New PIDF");
            telemetry.addData(">", " ");
            telemetry.addData("--- LIVE VALUES ---", "");
            telemetry.addData("CURRENT P (Targeting)", "%.3f", currentP);
            telemetry.addData("CURRENT I", "%.3f", currentI);
            telemetry.addData("CURRENT D", "%.3f", currentD);
            telemetry.addData("CURRENT F", "%.3f", currentF);
            telemetry.addData(">", " ");
            telemetry.addData("Left Velocity (Actual)", "%.1f", myRobot.leftOuttakeMotor.getVelocity());
            telemetry.addData("Right Velocity (Actual)", "%.1f", myRobot.rightOuttakeMotor.getVelocity());

            // Show the PIDF *actually* on the motor
            PIDFCoefficients motorPIDF = myRobot.leftOuttakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Motor P (Actual)", "%.3f", motorPIDF.p);

            telemetry.update();
        }

        // Stop motors when OpMode exits
        myRobot.leftOuttakeMotor.setVelocity(0);
        myRobot.rightOuttakeMotor.setVelocity(0);
    }
}