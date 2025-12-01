package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.BYTES_CONFIG;

public class Bytes_PID_Controller {

    // PID INFO:
    //
    //   P - responds to current error
    //   I - responds to accumulated past error (eliminates steady state error)
    //   D - responds to rate of change of error (dampens oscillation and provides braking)
    //
    //   motor_power_P = kP * current_error (tip: if overshooting target, kP is too high. if aims too slowly, too low)
    //   motor_power_I = kI * integral_sum (tip: set max limit to prevent infinite growth when motors at max power)
    //   motor_power_D = kD * error_derivative (tip: highly sensitive to sensor noise. may need a low-pass filter)
    //
    //   Tuning:
    //     1. Tune kP - set kI and kD to 0. Increase kP from 0.001 until system oscillates. Then back off 30%
    //     2. Tune kD - Increase kD from 0.0001 until system overshoots. THen back off.
    //     3. Tune kI - (if needed) Increase kI from 0.00001 until static error at end is gone.
    //
    //    Other Parameters needing to be set in Config:
    //      Deadband        - tolerance for error you can live with
    //      MaxIntegralSum  - prevent runaway power when motors are maxed
    //      MaxDerivative   - cheap way to filter noise from sensor
    //

    private BYTES_CONFIG.PidConstants pidConstants;

    private double prevError;
    private double integralSum;
    private double cycleTime;
    private double lastTime;
    private double filteredValue;

    public Bytes_PID_Controller (BYTES_CONFIG.PidConstants pidConstants) {
        this.pidConstants = pidConstants;
        this.prevError = 0.0;
        this.integralSum = 0.0;
        this.cycleTime = 0.0;
        this.lastTime = System.currentTimeMillis();
        this.filteredValue = Double.NaN;
    }

    // given a current reading and a target reading, return the power value to apply
    public double getNewPower (double currentMeasurement, double targetMeasurement, boolean useFilter, TelemetryPacket packet) {

        double newPower = 0;

        packet.put("rawMeasurement", currentMeasurement);

        // filter currentMeasurement if requested to dampen noise from sensor
        if (useFilter) {
            currentMeasurement = filter(currentMeasurement);
        }

        packet.put("filteredMeasurement", currentMeasurement);

        // calculate cycle time
        long currentTime = System.currentTimeMillis();

        // let's work in seconds
        this.cycleTime = (currentTime - this.lastTime) / 1000.0;
        this.lastTime = currentTime; // store currentTime for next loop iteration

        // get error
        double currentError = currentMeasurement - targetMeasurement;

        // check if we are in range
        if (Math.abs(currentError) > this.pidConstants.deadBand) {

            // get integral sum, capped at MAX config setting
            this.integralSum += currentError * this.cycleTime;
            this.integralSum = Math.max(-this.pidConstants.maxIntegralSum,
                    Math.min(this.pidConstants.maxIntegralSum, this.integralSum));

            // get error-derivative
            double errorDerivative = 0.0;
            if (this.cycleTime > 0) {
                errorDerivative = (currentError - this.prevError) / this.cycleTime;

                packet.put("rawDeriv", errorDerivative);

                // use limit to smooth the derivative and reduce noise spikes
                errorDerivative = Math.max(-pidConstants.maxDerivative,
                        Math.min(pidConstants.maxDerivative, errorDerivative));

                packet.put("clampedDeriv", errorDerivative);
            }

            // collect, p-i-d terms
            double pTerm = this.pidConstants.kP * currentError;
            double iTerm = this.pidConstants.kI * this.integralSum;
            double dTerm = this.pidConstants.kD * errorDerivative;

            packet.put("pTerm", pTerm);
            packet.put("iTerm", iTerm);
            packet.put("dTerm", dTerm);

            // update state variables
            this.prevError = currentError;

            // calculate new power
            newPower = pTerm + iTerm + dTerm;

            packet.put("prekF-power", newPower);

            // boost power if not large enough to get robot moving
            if (newPower > 0) {
                newPower = Math.max(newPower, this.pidConstants.kF);
            } else if (newPower < 0) {
                newPower = Math.min(newPower, -this.pidConstants.kF);
            }
        } else {
            // we reached our target - reset state machine values
            this.resetController();
        }

        packet.put("newPower", newPower);
        return newPower;
    }

    ////////////////////////////////////////////////////////////////////////////
    //
    // implement Low Pass Filters on Bearing and Range readings to reduce noise
    //
    ////////////////////////////////////////////////////////////////////////////
    public double filter (double rawReading) {

        // if first time through this set of readings
        if (Double.isNaN(this.filteredValue)) {
            this.filteredValue = rawReading;
        } else {
            this.filteredValue = (this.pidConstants.alpha * rawReading) + ((1.0 - this.pidConstants.alpha) * this.filteredValue);
        }

        return this.filteredValue;
    }

    public void resetController () {
        this.filteredValue = Double.NaN;
        this.prevError = 0;
        this.integralSum = 0;
        this.cycleTime = 0;
    }
}