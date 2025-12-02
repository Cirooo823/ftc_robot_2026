package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class VoltageFlywheelController {
    // --- HARDWARE ---
    private DcMotorEx flywheel_Left;
    private DcMotorEx flywheel_Right;

    // --- NEW: VOLTAGE SENSOR (for feedforward compensation) ---
    private VoltageSensor batteryVoltageSensor;
    // Tune this to whatever voltage you consider "nominal" when you tuned kF.
    // 12.0–13.0 is typical for an FTC 12V system fresh off the charger.
    private static final double NOMINAL_VOLTAGE = 13.0;

    // --- HARDWARE CONSTANTS ---
    private final DcMotorSimple.Direction LEFT_DIR  = DcMotorSimple.Direction.FORWARD;
    private final DcMotorSimple.Direction RIGHT_DIR = DcMotorSimple.Direction.REVERSE;
    private final double TICKS_PER_REVOLUTION = 28.0; // Correct for GoBilda 5203 1:1

    // --- PIDF COEFFICIENTS (your tuned values) ---
    public static double kF = 0.00036; // YOUR TUNED kF
    public static double kP = 0.00008;  // YOUR TUNED kP
    public static double kI = 0.0;      // Likely remains 0.0
    public static double kD = 0.0;      // Likely remains 0.0

    // --- PID State Variables ---
    private double lastErrorL = 0, lastErrorR = 0;
    private double integralL  = 0, integralR  = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // --- Flywheel State ---
    private double  targetRPM   = 0;   // Default to 0 RPM (off)
    private boolean flywheelOn  = false;

    // --- Constructor ---
    public VoltageFlywheelController(HardwareMap hardwareMap) {
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setDirection(LEFT_DIR);
        flywheel_Right.setDirection(RIGHT_DIR);

        flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Grab the robot's battery voltage sensor (first available)
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        pidTimer.reset();
    }

    // --- Methods to Control the Flywheel ---
    public void setFlywheelTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public void turnFlywheelOn() {
        flywheelOn = true;
        // When turning on, ensure a target is set. You might want a default.
        if (this.targetRPM == 0) this.targetRPM = 3000; // Example default
    }

    public void turnFlywheelOff() {
        flywheelOn = false;
        targetRPM = 0;
        setFlywheelPower(0, 0);
        resetPIDState();
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    // --- Main Update Method: Call this in your OpMode's loop() ---
    public void update() {
        double targetTPS = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;

        if (flywheelOn && targetRPM > 0) {
            double powerL = calculatePIDF(flywheel_Left,  targetTPS, lastErrorL, integralL, true);
            double powerR = calculatePIDF(flywheel_Right, targetTPS, lastErrorR, integralR, false);
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

    private double calculatePIDF(DcMotorEx motor,
                                 double targetTPS,
                                 double lastError,
                                 double integralSum,
                                 boolean isLeftMotor) {

        double dt = pidTimer.seconds();
        if (dt <= 0) dt = 1e-3;   // safety so we never divide by zero
        pidTimer.reset();

        // Velocity sign is kept consistent with your original code
        double currentTPS = -motor.getVelocity();
        double error = targetTPS - currentTPS;

        // --- NEW: voltage compensation for feedforward ---
        double currentBatteryVoltage = (batteryVoltageSensor != null)
                ? batteryVoltageSensor.getVoltage()
                : NOMINAL_VOLTAGE; // fallback if sensor is null
        double voltageMultiplier = NOMINAL_VOLTAGE / Math.max(currentBatteryVoltage, 1.0);

        // FF was: targetTPS * kF;
        // Now we scale for voltage so RPM stays more consistent across battery levels.
        double feedforward   = (targetTPS * kF) * voltageMultiplier;
        double proportional  = error * kP;

        // Integral term (same logic as your original)
        if (Math.abs(error) > 50 && Math.abs(currentTPS) < targetTPS * 1.1) {
            integralSum += error * dt;
        }
        double maxIntegralContribution = 0.15;
        double integralTerm = Range.clip(integralSum * kI,
                -maxIntegralContribution,
                maxIntegralContribution);

        double derivative    = (error - lastError) / dt;
        double derivativeTerm = derivative * kD;

        // Store state back into the appropriate side
        if (isLeftMotor) {
            this.lastErrorL = error;
            this.integralL  = integralSum;
        } else {
            this.lastErrorR = error;
            this.integralR  = integralSum;
        }

        double totalPower = feedforward + proportional + integralTerm + derivativeTerm;
        return Range.clip(totalPower, 0.0, 1.0);
    }

    private void resetPIDState() {
        lastErrorL = 0;
        lastErrorR = 0;
        integralL  = 0;
        integralR  = 0;
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
}
