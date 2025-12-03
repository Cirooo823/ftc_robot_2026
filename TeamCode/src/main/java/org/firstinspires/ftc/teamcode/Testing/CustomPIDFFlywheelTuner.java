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
    private ElapsedTime pidTimer = new ElapsedTime(); // Timer to calculate dt for PID

    // --- NEW: VOLTAGE SENSOR FOR FEEDFORWARD COMPENSATION ---
    private VoltageSensor batteryVoltageSensor;
    // Set this to the voltage you tuned around / consider "normal"
    private static final double NOMINAL_VOLTAGE = 13.0;

    // --- MOTOR SPECIFICS (GoBilda 5203 1:1 Ratio) ---
    // Encoder PPR (Pulses Per Revolution) at Output Shaft: 28 PPR
    private final double TICKS_PER_REVOLUTION = 28.0;

    // --- HARDWARE CONSTANTS (Confirmed by diagnostics) ---
    private final DcMotorSimple.Direction LEFT_MOTOR_FINAL_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    private final DcMotorSimple.Direction RIGHT_MOTOR_FINAL_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // --- TUNING VARIABLES (Use these to adjust with Gamepad) ---
    public static double targetRPM = 3000; // Initial target RPM
    public static boolean flywheelOn = false; // Toggle state

    // --- CUSTOM PIDF COEFFICIENTS ---
    // Start values; you will tune these live.
    public static double kF = 1.0 / 2550.0; // ~0.000392
    public static double kP = 0.0000;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // --- PID State Variables (Internal to the controller) ---
    double lastErrorL = 0, lastErrorR = 0;
    double integralL = 0, integralR = 0;

    // --- Gamepad Debounce variables ---
    private boolean debounceA, debounceUp, debounceDown;
    private boolean debounceFUp, debounceFDown, debouncePUp, debouncePDown;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware Initialization ---
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setDirection(LEFT_MOTOR_FINAL_DIRECTION);
        flywheel_Right.setDirection(RIGHT_MOTOR_FINAL_DIRECTION);

        // RUN_WITHOUT_ENCODER because we’re doing our own PIDF in power space
        flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Voltage sensor (take first available)
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        telemetry.addLine("--- Custom PIDF Flywheel Tuner (Voltage-Comp) ---");
        telemetry.addData("Left Motor Dir", LEFT_MOTOR_FINAL_DIRECTION.toString());
        telemetry.addData("Right Motor Dir", RIGHT_MOTOR_FINAL_DIRECTION.toString());
        telemetry.addData("Ticks Per Rev (Output)", TICKS_PER_REVOLUTION);
        telemetry.addLine("Gamepad1: A=Toggle, DpadU/D=TargetRPM");
        telemetry.addLine("Gamepad2: DpadU/D = kF, RB/LB = kP");
        telemetry.addLine("Press START to begin tuning.");
        telemetry.update();

        waitForStart();
        pidTimer.reset(); // Start timer for dt calc

        while (opModeIsActive()) {
            // 1. Handle User Input for Tuning
            handleGamepadInput();

            // 2. Calculate Target Ticks Per Second
            double targetTPS = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;

            // 3. Run the Custom PIDF Loop
            if (flywheelOn) {
                double powerL = calculatePIDF(flywheel_Left,  targetTPS, lastErrorL, integralL, true);
                double powerR = calculatePIDF(flywheel_Right, targetTPS, lastErrorR, integralR, false);

                flywheel_Left.setPower(powerL);
                flywheel_Right.setPower(powerR);
            } else {
                flywheel_Left.setPower(0);
                flywheel_Right.setPower(0);
                resetPIDState();
            }

            // 4. Telemetry Updates
            double currentTPS_L = -flywheel_Left.getVelocity();
            double currentRPM_L = (currentTPS_L / TICKS_PER_REVOLUTION) * 60.0;
            double currentTPS_R = -flywheel_Right.getVelocity();
            double currentRPM_R = (currentTPS_R / TICKS_PER_REVOLUTION) * 60.0;

            double batteryV = (batteryVoltageSensor != null) ? batteryVoltageSensor.getVoltage() : 0.0;
            double voltageMultiplier = (batteryV > 1.0) ? NOMINAL_VOLTAGE / batteryV : 1.0;

            telemetry.addData("Flywheel State", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", "%.2f", targetRPM);
            telemetry.addData("Actual RPM (L)", "%.2f", currentRPM_L);
            telemetry.addData("Actual RPM (R)", "%.2f", currentRPM_R);
            telemetry.addData("Error RPM (L)", "%.2f", targetRPM - currentRPM_L);
            telemetry.addData("Error RPM (R)", "%.2f", targetRPM - currentRPM_R);
            telemetry.addLine("--- Coefficients ---");
            telemetry.addData("kF (Feedforward)", "%.6f", kF);
            telemetry.addData("kP (Proportional)", "%.6f", kP);
            // telemetry.addData("kI (Integral)", "%.6f", kI);
            // telemetry.addData("kD (Derivative)", "%.6f", kD);
            telemetry.addLine("--- Voltage ---");
            telemetry.addData("Battery V", "%.2f", batteryV);
            telemetry.addData("Voltage Mult", "%.3f", voltageMultiplier);
            telemetry.update();
        }

        // --- Stop motors on OpMode End ---
        flywheel_Left.setPower(0);
        flywheel_Right.setPower(0);
    }

    // ==================================================================
    // CUSTOM PIDF WITH VOLTAGE COMPENSATION
    // ==================================================================
    private double calculatePIDF(DcMotorEx motor,
                                 double targetTPS,
                                 double lastError,
                                 double integralSum,
                                 boolean isLeftMotor) {

        double dt = pidTimer.seconds();
        if (dt <= 0) dt = 1e-3;
        pidTimer.reset();

        // 1. Current velocity (negated to make "forward" positive)
        double currentTPS = -motor.getVelocity();

        // 2. Error
        double error = targetTPS - currentTPS;

        // 3. Feedforward with VOLTAGE COMPENSATION
        double batteryV = (batteryVoltageSensor != null) ? batteryVoltageSensor.getVoltage() : NOMINAL_VOLTAGE;
        double voltageMultiplier = NOMINAL_VOLTAGE / Math.max(batteryV, 1.0);
        double feedforward = (targetTPS * kF) * voltageMultiplier;

        // 4. Proportional term
        double proportional = error * kP;

        // 5. Integral term (optional, currently kI = 0, but logic kept)
        if (Math.abs(error) > 50 && Math.abs(currentTPS) < targetTPS * 1.1) {
            integralSum += error * dt;
        }
        double maxIntegralContribution = 0.15;
        double integralTerm = Range.clip(integralSum * kI,
                -maxIntegralContribution,
                maxIntegralContribution);

        // 6. Derivative term
        double derivative = (error - lastError) / dt;
        double derivativeTerm = derivative * kD;

        // Save PID state for this side
        if (isLeftMotor) {
            this.lastErrorL = error;
            this.integralL  = integralSum;
        } else {
            this.lastErrorR = error;
            this.integralR  = integralSum;
        }

        // 7. Total power
        double totalPower = feedforward + proportional + integralTerm + derivativeTerm;

        // 8. Clip to [0, 1] since flywheel only spins forward
        return Range.clip(totalPower, 0.0, 1.0);
    }

    // --- Reset PID State Variables when flywheel is turned off ---
    private void resetPIDState() {
        lastErrorL = 0;
        lastErrorR = 0;
        integralL  = 0;
        integralR  = 0;
        pidTimer.reset();
    }

    // ==================================================================
    // GAMEPAD INPUT HANDLING (For live tuning)
    // ==================================================================
    private void handleGamepadInput() {
        // Gamepad1 A: Toggle flywheel ON/OFF
        if (gamepad1.a && !debounceA) { flywheelOn = !flywheelOn; debounceA = true; }
        else if (!gamepad1.a) { debounceA = false; }

        // Gamepad1 Dpad Up/Down: Adjust Target RPM
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

        // Gamepad2 Dpad Up/Down: Adjust kF (Feedforward)
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

        // Gamepad2 Right/Left Bumper: Adjust kP (Proportional)
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

        // TODO: Add controls for kI and kD if you want to tune them later
    }
}
