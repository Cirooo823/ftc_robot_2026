package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageFlywheelController { // Retaining Warriors' class name

    // --- HARDWARE ---
    private DcMotorEx flywheel_Left;
    private DcMotorEx flywheel_Right;

    // --- VOLTAGE SENSOR ---
    private VoltageSensor batteryVoltageSensor;
    private final double NOMINAL_VOLTAGE = 13.0; // Define your nominal voltage. Tune kF/kP at this voltage.

    // --- HARDWARE CONSTANTS ---
    // !!! IMPORTANT: Verify these directions match your physical robot's wiring.
    private final DcMotorSimple.Direction LEFT_DIR = DcMotorSimple.Direction.FORWARD;  // Example
    private final DcMotorSimple.Direction RIGHT_DIR = DcMotorSimple.Direction.REVERSE; // Example
    private final double TICKS_PER_REVOLUTION = 28.0; // GoBilda 5203 1:1

    // --- PIDF COEFFICIENTS (Warriors' values, as starting point) ---
    public static double kF = 0.0004;
    public static double kP = 0.0001;
    public static double kI = 0.0; // Start at 0, tune if needed
    public static double kD = 0.0; // Start at 0, rarely needed for flywheels

    // --- PID State Variables (For separate PID loops) ---
    private double lastErrorL = 0;
    private double integralL = 0;
    private double lastErrorR = 0;
    private double integralR = 0;
    private ElapsedTime pidTimer = new ElapsedTime(); // A single timer for both is fine

    private double targetRPM = 0;
    private boolean flywheelOn = false;

    // --- Constructor ---
    public VoltageFlywheelController(HardwareMap hardwareMap) {
        flywheel_Left = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setDirection(LEFT_DIR);
        flywheel_Right.setDirection(RIGHT_DIR);

        flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Grab the robot's battery voltage sensor (first available)
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // pidTimer.reset(); // Initial reset can be here, or at start of update() loop
        // Current setup resets at start of update(), which is fine.
    }

    // --- Methods to Control the Flywheel ---
    public void setFlywheelTargetRPM(double rpm) {
        this.targetRPM = Math.max(0, rpm); // Ensure target RPM is never negative
    }

    public void turnFlywheelOn() {
        // Only turn on if a meaningful target is set (e.g., not 0 or very small)
        if (this.targetRPM > 100) { // Set a minimum RPM to consider "on"
            flywheelOn = true;
        }
    }

    public void turnFlywheelOff() {
        flywheelOn = false;
        targetRPM = 0;
        // setFlywheelPower(0, 0); // No need for this, update() handles it
        resetPIDState();
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    // --- Main Update Method: Call this in your OpMode's loop() ---
    public void update() {
        double targetTPS = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;

        // Reset the timer ONCE per update cycle for accurate dt for BOTH motors
        pidTimer.reset();

        if (flywheelOn && targetRPM > 0) {
            // Calculate for left motor, which will update this.lastErrorL and this.integralL
            double powerL = calculatePIDF(flywheel_Left, targetTPS, true);
            // Calculate for right motor, which will update this.lastErrorR and this.integralR
            double powerR = calculatePIDF(flywheel_Right, targetTPS, false);
            setFlywheelPower(powerL, powerR);
        } else {
            setFlywheelPower(0, 0);
            resetPIDState();
        }
    }

    private void setFlywheelPower(double powerL, double powerR) {
        flywheel_Left.setPower(powerL);
        flywheel_Right.setPower(powerR);
    }

    // Removed lastError and integralSum parameters
    private double calculatePIDF(DcMotorEx motor, double targetTPS, boolean isLeftMotor) {

        double dt = pidTimer.seconds(); // dt is now time since the START of the update cycle
        if (dt <= 0) dt = 1e-3;

        // Get the correct state variables for this motor (left or right)
        double currentLastError;
        double currentIntegralSum;

        if (isLeftMotor) {
            currentLastError = this.lastErrorL;
            currentIntegralSum = this.integralL;
        } else {
            currentLastError = this.lastErrorR;
            currentIntegralSum = this.integralR;
        }

        double currentTPS = -motor.getVelocity(); // Velocity sign consistent with your code
        double error = targetTPS - currentTPS;

        // --- Voltage Compensation ---
        double currentBatteryVoltage = (batteryVoltageSensor != null)
                ? batteryVoltageSensor.getVoltage()
                : NOMINAL_VOLTAGE;
        double voltageMultiplier = NOMINAL_VOLTAGE / Math.max(currentBatteryVoltage, 1.0);

        double feedforward = (targetTPS * kF) * voltageMultiplier;
        double proportional = error * kP;

        // --- IMPROVED INTEGRAL ANTI-WINDUP ---
        // Only accumulate integral if within a reasonable error range AND motor is not saturated
        // AND not hugely over target speed.
        if (Math.abs(error) > 50 && Math.abs(currentTPS) < targetTPS * 1.1) {
            currentIntegralSum += error * dt;
            // Anti-windup: Limit the integral sum directly, relative to kI
            // (e.g., allow integral term to contribute up to 0.5 power)
            double maxIntegralSum = 0.5 / (kI == 0 ? 1 : kI); // Avoid division by zero if kI is 0
            currentIntegralSum = Range.clip(currentIntegralSum, -maxIntegralSum, maxIntegralSum);
        } else {
            // Decay integral if error is too small or too large, or if PID is off
            currentIntegralSum *= 0.95; // Decay rate
        }
        double integralTerm = currentIntegralSum * kI;

        double derivative = (error - currentLastError) / dt;
        double derivativeTerm = derivative * kD;

        double totalPower = feedforward + proportional + integralTerm + derivativeTerm;

        // Store the updated state variables back into the class fields
        if (isLeftMotor) {
            this.lastErrorL = error;
            this.integralL = currentIntegralSum;
        } else {
            this.lastErrorR = error;
            this.integralR = currentIntegralSum;
        }

        return Range.clip(totalPower, 0.0, 1.0);
    }

    private void resetPIDState() {
        lastErrorL = 0;
        lastErrorR = 0;
        integralL = 0;
        integralR = 0;
        pidTimer.reset();
    }

    // --- Telemetry Helpers ---
    public double getCurrentRPM_Left() {
        return (-flywheel_Left.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }

    public double getCurrentRPM_Right() {
        return (-flywheel_Right.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getErrorRPM_Left() {
        return targetRPM - getCurrentRPM_Left();
    }

    public double getErrorRPM_Right() {
        return targetRPM - getCurrentRPM_Right();
    }

    // Optional: expose battery voltage for telemetry
    public double getBatteryVoltage() {
        return (batteryVoltageSensor != null) ? batteryVoltageSensor.getVoltage() : 0.0;
    }
    public double getCurrentRPM_Average() {
        // Get velocities from both motors
        double velL = -flywheel_Left.getVelocity(); // Note: negative sign matches your logic
        double velR = -flywheel_Right.getVelocity();

        // Average them (using absolute value just to be safe about direction)
        double averageTPS = (Math.abs(velL) + Math.abs(velR)) / 2.0;

        // Convert Ticks Per Second to RPM
        return (averageTPS / TICKS_PER_REVOLUTION) * 60.0;
    }
}
