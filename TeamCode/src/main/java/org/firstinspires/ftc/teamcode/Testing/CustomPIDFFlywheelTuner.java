package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Custom PIDF Flywheel Tuner", group="Calibration")
public class CustomPIDFFlywheelTuner extends LinearOpMode {

    private DcMotorEx flywheel_Left;
    private DcMotorEx flywheel_Right;

    // Timing: one dt for the entire loop (matches VoltageFlywheelController.update())
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTimeSec = 0.0;

    // Voltage sensor
    private VoltageSensor batteryVoltageSensor;
    private static final double NOMINAL_VOLTAGE = 13.0;

    // Motor specifics
    private static final double TICKS_PER_REVOLUTION = 28.0;

    private static final DcMotorSimple.Direction LEFT_MOTOR_FINAL_DIRECTION  = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction RIGHT_MOTOR_FINAL_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // Tuning variables
    public static double targetRPM = 3000;
    public static boolean flywheelOn = false;

    // PIDF coefficients (same “meaning” as VoltageFlywheelController)
    public static double kF = 1.0 / 2550.0;   // ~0.000392
    public static double kP = 0.00005;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // PID state (per motor)
    private double lastErrorL = 0.0, integralL = 0.0;
    private double lastErrorR = 0.0, integralR = 0.0;

    // Debug / last powers
    private double lastPowerL = 0.0, lastPowerR = 0.0;

    // Debounce variables
    private boolean debounceA, debounceUp, debounceDown;
    private boolean debounceFUp, debounceFDown, debouncePUp, debouncePDown;
    private boolean debounceIUp, debounceIDown, debounceDUp, debounceDDown;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setDirection(LEFT_MOTOR_FINAL_DIRECTION);
        flywheel_Right.setDirection(RIGHT_MOTOR_FINAL_DIRECTION);

        flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        telemetry.addLine("--- Custom PIDF Flywheel Tuner (VoltageFlywheelController-matched) ---");
        telemetry.addData("Left Motor Dir", LEFT_MOTOR_FINAL_DIRECTION.toString());
        telemetry.addData("Right Motor Dir", RIGHT_MOTOR_FINAL_DIRECTION.toString());
        telemetry.addData("Ticks Per Rev (Output)", TICKS_PER_REVOLUTION);
        telemetry.addLine("Gamepad1: A=Toggle ON/OFF, DpadU/D=TargetRPM ±100");
        telemetry.addLine("Gamepad2: DpadU/D=kF ±1e-6, RB/LB=kP ±1e-5");
        telemetry.addLine("Gamepad2: B/X=kI ±2e-7, Y/A=kD ±1e-6");
        telemetry.addLine("Press START to begin tuning.");
        telemetry.update();

        waitForStart();

        loopTimer.reset();
        lastLoopTimeSec = loopTimer.seconds();

        while (opModeIsActive()) {

            // 1) dt for this loop (matches your controller pattern)
            double now = loopTimer.seconds();
            double dt = now - lastLoopTimeSec;
            lastLoopTimeSec = now;
            if (dt <= 0) dt = 1e-3;
            dt = Range.clip(dt, 1e-3, 0.1);

            // 2) Handle tuning input
            handleGamepadInput();

            // 3) Convert target RPM -> target TPS
            double targetTPS = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;

            // 4) Battery voltage and vm (same as VoltageFlywheelController)
            double batteryV = (batteryVoltageSensor != null) ? batteryVoltageSensor.getVoltage() : NOMINAL_VOLTAGE;
            batteryV = Math.max(batteryV, 1.0);
            double vm = NOMINAL_VOLTAGE / batteryV;

            // 5) Run control
            if (flywheelOn) {
                lastPowerL = calculatePIDF(flywheel_Left,  targetTPS, dt, vm, true);
                lastPowerR = calculatePIDF(flywheel_Right, targetTPS, dt, vm, false);
                flywheel_Left.setPower(lastPowerL);
                flywheel_Right.setPower(lastPowerR);
            } else {
                flywheel_Left.setPower(0.0);
                flywheel_Right.setPower(0.0);
                lastPowerL = 0.0;
                lastPowerR = 0.0;
                resetPIDState();
            }

            // 6) Telemetry
            double currentTPS_L = -flywheel_Left.getVelocity();
            double currentRPM_L = (currentTPS_L / TICKS_PER_REVOLUTION) * 60.0;

            double currentTPS_R = -flywheel_Right.getVelocity();
            double currentRPM_R = (currentTPS_R / TICKS_PER_REVOLUTION) * 60.0;

            telemetry.addData("Flywheel State", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Actual RPM (L)", "%.0f", currentRPM_L);
            telemetry.addData("Actual RPM (R)", "%.0f", currentRPM_R);
            telemetry.addData("Error RPM (L)", "%.0f", targetRPM - currentRPM_L);
            telemetry.addData("Error RPM (R)", "%.0f", targetRPM - currentRPM_R);

            telemetry.addLine("--- Coefficients ---");
            telemetry.addData("kF", "%.6f", kF);
            telemetry.addData("kP", "%.8f", kP);
            telemetry.addData("kI", "%.10f", kI);
            telemetry.addData("kD", "%.8f", kD);

            telemetry.addLine("--- Voltage ---");
            telemetry.addData("Battery V", "%.2f", batteryV);
            telemetry.addData("vm (13.0/V)", "%.3f", vm);
            telemetry.addData("dt (s)", "%.4f", dt);

            telemetry.addLine("--- Output ---");
            telemetry.addData("Power L", "%.3f", lastPowerL);
            telemetry.addData("Power R", "%.3f", lastPowerR);

            telemetry.update();
        }

        flywheel_Left.setPower(0.0);
        flywheel_Right.setPower(0.0);
    }

    // ==================================================================
    // PIDF (VoltageFlywheelController-matched structure)
    // ==================================================================
    private double calculatePIDF(DcMotorEx motor,
                                 double targetTPS,
                                 double dt,
                                 double vm,
                                 boolean isLeftMotor) {

        // Current velocity (keep your sign convention)
        double currentTPS = -motor.getVelocity();
        double error = targetTPS - currentTPS;

        // Pull correct state
        double lastErr = isLeftMotor ? lastErrorL : lastErrorR;
        double integ   = isLeftMotor ? integralL  : integralR;

        // Voltage-scaled gains (matches controller)
        double pGain = kP * vm;
        double iGain = kI * vm;
        double dGain = kD * vm;

        // Feedforward voltage-compensated (matches controller)
        double feedforward = (targetTPS * kF) * vm;
        double proportional = error * pGain;

        // Integral w/ basic anti-windup (same idea as your controller)
        if (iGain != 0.0) {
            boolean notOverspeedCrazy = currentTPS < targetTPS * 1.1;
            if (Math.abs(error) > 50 && notOverspeedCrazy) {
                integ += error * dt;

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

        // Derivative
        double derivative = (error - lastErr) / dt;
        double derivativeTerm = derivative * dGain;

        double totalPower = feedforward + proportional + integralTerm + derivativeTerm;

        // Store back
        if (isLeftMotor) {
            lastErrorL = error;
            integralL = integ;
        } else {
            lastErrorR = error;
            integralR = integ;
        }

        // Flywheel only forward
        return Range.clip(totalPower, 0.0, 1.0);
    }

    private void resetPIDState() {
        lastErrorL = 0.0;
        lastErrorR = 0.0;
        integralL  = 0.0;
        integralR  = 0.0;
    }

    // ==================================================================
    // GAMEPAD INPUT HANDLING (Live tuning)
    // ==================================================================
    private void handleGamepadInput() {
        // Gamepad1 A: Toggle flywheel
        if (gamepad1.a && !debounceA) { flywheelOn = !flywheelOn; debounceA = true; }
        else if (!gamepad1.a) { debounceA = false; }

        // Gamepad1 Dpad Up/Down: Target RPM
        if (gamepad1.dpad_up && !debounceUp) {
            targetRPM += 100;
            debounceUp = true;
        } else if (!gamepad1.dpad_up) {
            debounceUp = false;
        }

        if (gamepad1.dpad_down && !debounceDown) {
            targetRPM -= 100;
            if (targetRPM < 0) targetRPM = 0;
            debounceDown = true;
        } else if (!gamepad1.dpad_down) {
            debounceDown = false;
        }

        // Gamepad2 Dpad Up/Down: kF
        if (gamepad2.dpad_up && !debounceFUp) {
            kF += 0.000001;
            debounceFUp = true;
        } else if (!gamepad2.dpad_up) {
            debounceFUp = false;
        }

        if (gamepad2.dpad_down && !debounceFDown) {
            kF -= 0.000001;
            if (kF < 0) kF = 0;
            debounceFDown = true;
        } else if (!gamepad2.dpad_down) {
            debounceFDown = false;
        }

        // Gamepad2 RB/LB: kP
        if (gamepad2.right_bumper && !debouncePUp) {
            kP += 0.00001;
            debouncePUp = true;
        } else if (!gamepad2.right_bumper) {
            debouncePUp = false;
        }

        if (gamepad2.left_bumper && !debouncePDown) {
            kP -= 0.00001;
            if (kP < 0) kP = 0;
            debouncePDown = true;
        } else if (!gamepad2.left_bumper) {
            debouncePDown = false;
        }

        // Gamepad2 B/X: kI (tiny steps)
        if (gamepad2.b && !debounceIUp) {
            kI += 0.0000002; // 2e-7
            debounceIUp = true;
        } else if (!gamepad2.b) {
            debounceIUp = false;
        }

        if (gamepad2.x && !debounceIDown) {
            kI -= 0.0000002; // 2e-7
            if (kI < 0) kI = 0;
            debounceIDown = true;
        } else if (!gamepad2.x) {
            debounceIDown = false;
        }

        // Gamepad2 Y/A: kD
        if (gamepad2.y && !debounceDUp) {
            kD += 0.000001; // 1e-6
            debounceDUp = true;
        } else if (!gamepad2.y) {
            debounceDUp = false;
        }

        if (gamepad2.a && !debounceDDown) {
            kD -= 0.000001; // 1e-6
            if (kD < 0) kD = 0;
            debounceDDown = true;
        } else if (!gamepad2.a) {
            debounceDDown = false;
        }
    }
}
