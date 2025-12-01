package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * This OpMode is for tuning the D-gain of a flywheel.
 *
 * It tests the theory that K_d can be used to dampen (smooth)
 * a very aggressive stop.
 *
 * HOW TO USE:
 * 1. Deploy and run this OpMode.
 * 2. The telemetry will show the current P, I, D, and F values.
 * 3. Hold 'A' to spin the flywheels up to TEST_VELOCITY.
 * 4. Release 'A' to command the aggressive stop (setVelocity(-100)).
 * 5. OBSERVE the stop. Is it fast? Does it "bang" or "shudder" at 0?
 *
 * 6. Press D-PAD UP to increase the D gain.
 * 7. Press D-PAD DOWN to decrease the D gain.
 * 8. Press 'Y' to send the new D value (and default P,I,F) to the motor controllers.
 * 9. Repeat steps 3-5.
 *
 * 10. Find the 'D' value that makes the aggressive stop smooth,
 * without making it too slow.
 *
 * 11. Once you find your magic 'D' value, write it down!
 */
@TeleOp(name = "Flywheel D-Gain Tuner", group = "Tests")
public class FlywheelPIDTuner extends LinearOpMode {

    Bytes_Robot myRobot;

    // --- YOUR SETTINGS ---
    private static final double TEST_VELOCITY = 1150;
    // Increment for tuning D. D-gains are usually sensitive.
    // Try 0.1 first. If it's too slow, try 0.05 or 0.01.
    private static final double D_INCREMENT = 0.1;

    // We will get the default P and F from the controller.
    // We will set I to 0.
    // We will TUNE D.
    private double currentP; // Will be read from the motor
    private double currentI = 0.0;
    private double currentD = 0.0; // This is the value we will tune
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
            myRobot = new Bytes_Robot(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Error", "Hardware class not found or init failed.");
            telemetry.addData("Message", e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        // --- MOTOR CONFIGURATION ---
        myRobot.leftOuttakeMotor.setDirection(DcMotor.Direction.FORWARD);
        myRobot.rightOuttakeMotor.setDirection(DcMotor.Direction.REVERSE);

        myRobot.leftOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myRobot.rightOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myRobot.leftOuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.rightOuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRobot.leftOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myRobot.rightOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- GET DEFAULT PIDF ---
        PIDFCoefficients defaultPIDF = myRobot.leftOuttakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Use the controller's default P and F as our starting point
        currentP = defaultPIDF.p;
        currentF = defaultPIDF.f;
        // We will force I to 0, and start D at 0.
        currentI = 0.0;
        currentD = 0.0; // Start D at 0

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start.");
        telemetry.addData(">", " ");
        telemetry.addData("Default P", defaultPIDF.p);
        telemetry.addData("Default I", defaultPIDF.i);
        telemetry.addData("Default D", defaultPIDF.d);
        telemetry.addData("Default F", defaultPIDF.f);
        telemetry.addData(">", "We will use P=%.3f, I=0, D=TUNED, F=%.3f", currentP, currentF);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Button Edge Detection Logic ---
            yJustPressed = false; // Reset flag each loop
            if (gamepad1.y && !yWasPressed) {
                yJustPressed = true;
            }
            yWasPressed = gamepad1.y;

            // --- MODIFIED: Tune D-Gain ---
            if (gamepad1.dpad_up && !dpadUpWasPressed) {
                currentD += D_INCREMENT; // Increase D
            }
            dpadUpWasPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownWasPressed) {
                if (currentD >= D_INCREMENT) { // Prevent going below 0
                    currentD -= D_INCREMENT; // Decrease D
                } else {
                    currentD = 0;
                }
            }
            dpadDownWasPressed = gamepad1.dpad_down;


            // --- Apply New PIDF ---
            if (yJustPressed) {
                // We send the *default* P, 0 for I, the *new tuned* D, and *default* F
                PIDFCoefficients newPIDF = new PIDFCoefficients(currentP, currentI, currentD, currentF);
                myRobot.leftOuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
                myRobot.rightOuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

                // Add a blocking message to confirm it was sent
                telemetry.clearAll();
                telemetry.addData("!!!", "PIDF VALUES APPLIED!");
                telemetry.addData("P", "%.3f (default)", currentP);
                telemetry.addData("I", "%.3f", currentI);
                telemetry.addData("D", "%.3f (TUNED)", currentD);
                telemetry.addData("F", "%.3f (default)", currentF);
                telemetry.update();
                sleep(500); // Pause for 0.5s to show this message
            }

            // --- Motor Control ---
            if (gamepad1.a) {
                // Run flywheels at test velocity
                myRobot.leftOuttakeMotor.setVelocity(TEST_VELOCITY);
                myRobot.rightOuttakeMotor.setVelocity(TEST_VELOCITY);
            } else {
                // Command aggressive stop
                // The large error (Target: -100, Actual: 1150)
                // will be multiplied by P to create strong reverse power.
                // The D-gain will react to this fast change
                // and apply an opposing (damping) force.
                myRobot.rightOuttakeMotor.setVelocity(-100);
                myRobot.leftOuttakeMotor.setVelocity(-100);
            }

            // --- TELEMETRY ---
            telemetry.addData("--- CONTROLS ---", "");
            telemetry.addData("Hold (A)", "Run Flywheels");
            telemetry.addData("Release (A)", "Aggressive STOP");
            telemetry.addData("D-Pad Up/Down", "Change D Gain");
            telemetry.addData("Press (Y)", "APPLY New PIDF");
            telemetry.addData(">", " ");
            telemetry.addData("--- LIVE VALUES ---", "");
            telemetry.addData("CURRENT P (default)", "%.3f", currentP);
            telemetry.addData("CURRENT I", "%.3f", currentI);
            telemetry.addData("CURRENT D (Targeting)", "%.3f", currentD);
            telemetry.addData("CURRENT F (default)", "%.3f", currentF);
            telemetry.addData(">", " ");
            telemetry.addData("Left Velocity (Actual)", "%.1f", myRobot.leftOuttakeMotor.getVelocity());
            telemetry.addData("Right Velocity (Actual)", "%.1f", myRobot.rightOuttakeMotor.getVelocity());

            // Show the PIDF *actually* on the motor
            PIDFCoefficients motorPIDF = myRobot.leftOuttakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Motor D (Actual)", "%.3f", motorPIDF.d);

            telemetry.update();
        }

        // Stop motors when OpMode exits
        myRobot.leftOuttakeMotor.setVelocity(0);
        myRobot.rightOuttakeMotor.setVelocity(0);
    }
}