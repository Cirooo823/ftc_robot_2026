package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class VoltageFlywheelController {

    // --- HARDWARE ---
    private final DcMotorEx flywheel_Left;
    private final DcMotorEx flywheel_Right;

    // --- VOLTAGE SENSOR ---
    private final VoltageSensor batteryVoltageSensor;
    private static final double NOMINAL_VOLTAGE = 13.0; // tune kF/kP at this nominal

    // --- HARDWARE CONSTANTS ---
    // Verify these match your robot.
    private static final DcMotorSimple.Direction LEFT_DIR  = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_DIR = DcMotorSimple.Direction.REVERSE;

    // IMPORTANT: verify encoder ticks per motor revolution for your exact motor/gearbox.
    private static final double TICKS_PER_REVOLUTION = 28.0;

    // --- PIDF COEFFICIENTS ---
    public static double kF = 0.000385;
    public static double kP = 0.00005;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // --- STATE (per motor) ---
    private double lastErrorL = 0.0, integralL = 0.0;
    private double lastErrorR = 0.0, integralR = 0.0;

    // Timing
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTimeSec = 0.0;

    private double targetRPM = 0.0;
    private boolean flywheelOn = false;

    // For debugging/telemetry
    private double lastPowerL = 0.0;
    private double lastPowerR = 0.0;

    public VoltageFlywheelController(HardwareMap hardwareMap) {
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setDirection(LEFT_DIR);
        flywheel_Right.setDirection(RIGHT_DIR);

        flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        loopTimer.reset();
        lastLoopTimeSec = loopTimer.seconds();
    }

    public void setFlywheelTargetRPM(double rpm) {
        this.targetRPM = Math.max(0.0, rpm);
    }

    public void turnFlywheelOn() {
        if (this.targetRPM > 100.0) {
            flywheelOn = true;
        }
    }

    public void turnFlywheelOff() {
        flywheelOn = false;
        targetRPM = 0.0;
        setFlywheelPower(0.0, 0.0);
        resetPIDState();
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public void update() {
        // dt = time since last update (in seconds)
        double now = loopTimer.seconds();
        double dt = now - lastLoopTimeSec;
        lastLoopTimeSec = now;

        if (dt <= 0) dt = 1e-3;
        // Optional: clamp dt to avoid huge jumps if you pause debugging
        dt = Range.clip(dt, 1e-3, 0.1);

        double targetTPS = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;

        if (flywheelOn && targetRPM > 0.0) {
            lastPowerL = calculatePIDF(flywheel_Left,  targetTPS, dt, true);
            lastPowerR = calculatePIDF(flywheel_Right, targetTPS, dt, false);
            setFlywheelPower(lastPowerL, lastPowerR);
        } else {
            lastPowerL = 0.0;
            lastPowerR = 0.0;
            setFlywheelPower(0.0, 0.0);
            resetPIDState();
        }
    }

    private void setFlywheelPower(double powerL, double powerR) {
        flywheel_Left.setPower(powerL);
        flywheel_Right.setPower(powerR);
    }

    private double calculatePIDF(DcMotorEx motor, double targetTPS, double dt, boolean isLeft) {

        double currentTPS = -motor.getVelocity();
        double error = targetTPS - currentTPS;

        double v = (batteryVoltageSensor != null) ? batteryVoltageSensor.getVoltage() : NOMINAL_VOLTAGE;
        v = Math.max(v, 1.0);

        // Voltage multiplier: increase commanded effort when voltage is low
        double vm = NOMINAL_VOLTAGE / v;

        // Pull correct state
        double lastErr = isLeft ? lastErrorL : lastErrorR;
        double integ   = isLeft ? integralL  : integralR;

        // Scale closed-loop gains with voltage as well (important!)
        double pGain = kP * vm;
        double iGain = kI * vm;
        double dGain = kD * vm;

        double feedforward = (targetTPS * kF) * vm;
        double proportional = error * pGain;

        // Integral (optional; you currently keep kI=0)
        if (iGain != 0.0) {
            // Basic anti-windup: integrate only when not saturated-ish and not wildly overspeed
            boolean notOverspeedCrazy = currentTPS < targetTPS * 1.1;
            if (Math.abs(error) > 50 && notOverspeedCrazy) {
                integ += error * dt;

                // Limit integral contribution to some fraction of power
                double maxIntegralPower = 0.3;
                double maxIntegralSum = maxIntegralPower / Math.max(iGain, 1e-9);
                integ = Range.clip(integ, -maxIntegralSum, maxIntegralSum);
            } else {
                integ *= 0.95;
            }
        } else {
            integ = 0.0;
        }
        double integralTerm = integ * iGain;

        double derivative = (error - lastErr) / dt;
        double derivativeTerm = derivative * dGain;

        double totalPower = feedforward + proportional + integralTerm + derivativeTerm;

        // Store back
        if (isLeft) {
            lastErrorL = error;
            integralL = integ;
        } else {
            lastErrorR = error;
            integralR = integ;
        }

        return Range.clip(totalPower, 0.0, 1.0);
    }

    private void resetPIDState() {
        lastErrorL = 0.0;
        lastErrorR = 0.0;
        integralL = 0.0;
        integralR = 0.0;
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

    public double getBatteryVoltage() {
        return (batteryVoltageSensor != null) ? batteryVoltageSensor.getVoltage() : 0.0;
    }

    public double getCurrentRPM_Average() {
        double velL = -flywheel_Left.getVelocity();
        double velR = -flywheel_Right.getVelocity();
        double averageTPS = (Math.abs(velL) + Math.abs(velR)) / 2.0;
        return (averageTPS / TICKS_PER_REVOLUTION) * 60.0;
    }

    // Optional: expose last computed powers for debugging
    public double getLastPowerL() { return lastPowerL; }
    public double getLastPowerR() { return lastPowerR; }
}
