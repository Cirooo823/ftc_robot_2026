package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="automatictele", group="TeleOp")
public class automatictele extends OpMode {
    // ===== DRIVE =====
    private DcMotorEx right_b, left_f, right_f, left_b;

    // ===== MECHANISMS =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor   intake;
    private CRServo   thirdStage;

    // ===== FLYWHEEL CONSTANTS (goBILDA 6000 RPM) =====
    private static final double TICKS_PER_REV     = 28.0;          // 5202 encoder
    private static final double MAX_RPM           = 4500.0;        // shooter cap, not full 6000
    private static final double MAX_TICKS_PER_SEC = (TICKS_PER_REV * MAX_RPM) / 60.0; // ~2100 tps

    // ===== OUTER VELOCITY LOOP TUNING =====
    // More conservative to reduce oscillation / overshoot
    private static final double OUTER_KP               = 0.015;
    private static final double RPM_FILTER_ALPHA       = 0.15;    // 0..1, lower = smoother
    private static final double RPM_TOLERANCE          = 15.0;    // +/- band where we don't correct
    private static final double MAX_DELTA_TPS_PER_LOOP = 60.0;    // max change in commandedTps each loop

    // Slight under-target bias so REV PIDF overshoot lands nearer real target
    private static final double TARGET_SCALE           = 0.88;

    // "Ready" band: 50–100 RPM BELOW target for LONG preset
    // shooterRPM - 100 <= filteredRpm <= shooterRPM - 50
    private static final double READY_BELOW_LOW   = 100.0;
    private static final double READY_BELOW_HIGH  = 50.0;
    private static final double READY_LATCH_TIME  = 1.0;   // seconds of auto-feed

    // Single-start target + step sizes
    private static final int DEFAULT_RPM   = 3300;  // default under MAX_RPM
    private static final int STEP_FINE     = 100;   // hold LB for fine steps
    private static final int STEP_NORMAL   = 300;   // default step (no bumper)
    private static final int STEP_COARSE   = 1000;  // hold RB for big steps

    // ===== PRESET RPMs =====
    private static final int PRESET_SHORT_RPM  = 3300; // gamepad2.X
    private static final int PRESET_MED_RPM    = 3600; // gamepad2.Y
    private static final int PRESET_LONG_RPM   = 3800; // gamepad2.B

    // ===== STATE =====
    private boolean flywheelOn = false;
    private boolean intakeOn   = false;

    // Edge detection (g1 intake toggles)
    private boolean prevA=false, prevX=false, prevB=false;
    private boolean prevDU=false, prevDD=false, prevDL=false, prevDR=false;

    // gamepad2 preset button edges
    private boolean prevX2=false, prevY2=false, prevB2=false;

    // Troubleshooting toggle
    private boolean useVelocityControl = true;  // Left-stick click toggles Velocity <-> Power
    private boolean prevLS = false;

    // Flywheel target (driver-adjustable) in RPM
    private double shooterRPM       = 2000.0; // will snap to DEFAULT_RPM on A=ON
    private double shooterTpsTarget = 0.0;

    // What we actually command after outer-loop clamp
    private double commandedTps     = 0.0;

    // Filtered RPM for smoother feedback
    private double filteredRpm      = 0.0;

    // Third stage (-1=left, 0=stop, +1=right)
    private int thirdDir = 0;

    // Automatic "ready to shoot" flags (for long preset)
    private boolean readyToShoot         = false;
    private boolean readyLatched         = false;
    private double  readyLatchedStartTime = 0.0;

    // Drive scale
    private double driverScale = 1.0;

    // last preset label for telemetry
    private String lastPreset = "—";

    @Override
    public void init() {
        // ---- Map drive ----
        left_f  = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b  = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Your working directions
        left_f.setDirection(DcMotor.Direction.REVERSE);
        right_b.setDirection(DcMotor.Direction.REVERSE);
        right_f.setDirection(DcMotor.Direction.REVERSE);

        right_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Mechanisms ----
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");
        intake         = hardwareMap.get(DcMotor.class, "intake");

        // Flywheel encoders for velocity control
        flywheel_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Coasting is typical for flywheels
        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Shared shaft via miter gears: send same positive velocity; flip one direction
        flywheel_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Third stage CR servo
        thirdStage = hardwareMap.get(CRServo.class, "thirdStage");
        thirdStage.setPower(0.0);

        // ===== Softer explicit PIDF for flywheel velocity =====
        double P = 8.0;
        double I = 0.0;
        double D = 2.0;
        double F = 8.0;
        flywheel_Left.setVelocityPIDFCoefficients(P, I, D, F);
        flywheel_Right.setVelocityPIDFCoefficients(P, I, D, F);
    }

    @Override
    public void loop() {
        drive();
        runFlywheel();
        runThirdStage();
        runIntake();

        telemetry.update(); // one update per loop
    }

    // ===================== DRIVE =====================
    private void drive(){
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;

        double y  = -gamepad1.left_stick_y;   // forward/back
        double x  = -gamepad1.right_stick_x;  // strafe
        double rx = -gamepad1.left_stick_x;   // rotate

        double ys  = y  * driverScale;
        double xs  = x  * driverScale;
        double rxs = rx * driverScale;

        double d = Math.max(Math.abs(ys) + Math.abs(xs) + Math.abs(rxs), 1.0);

        right_f.setPower(( ys + xs + rxs) / d);
        left_b.setPower( ( ys - xs + rxs) / d);
        left_f.setPower( ( ys - xs - rxs) / d);
        right_b.setPower(( ys + xs - rxs) / d);
    }

    // ===================== FLYWHEEL =====================
    private void runFlywheel() {
        // A toggles ON/OFF; always start at DEFAULT_RPM
        boolean a = gamepad2.a;
        if (a && !prevA) {
            flywheelOn = !flywheelOn;
            if (flywheelOn) {
                shooterRPM         = DEFAULT_RPM; // will clamp below
                lastPreset         = "Start(Default)";
                commandedTps       = 0.0;        // reset outer-loop command
                filteredRpm        = 0.0;        // reset filter
                readyLatched       = false;
                readyToShoot       = false;
            } else {
                shooterRPM         = 0.0;
                shooterTpsTarget   = 0.0;
                commandedTps       = 0.0;
                filteredRpm        = 0.0;
                readyLatched       = false;
                readyToShoot       = false;
            }
        }
        prevA = a;

        // ===== PRESETS: gamepad2.X / Y / B =====
        boolean x2 = gamepad2.x;
        if (x2 && !prevX2) {
            shooterRPM         = PRESET_SHORT_RPM;
            flywheelOn         = true;
            lastPreset         = "Short (X)";
            commandedTps       = 0.0;
            filteredRpm        = 0.0;
            readyLatched       = false;
            readyToShoot       = false;
        }
        prevX2 = x2;

        boolean y2 = gamepad2.y;
        if (y2 && !prevY2) {
            shooterRPM         = PRESET_MED_RPM;
            flywheelOn         = true;
            lastPreset         = "Medium (Y)";
            commandedTps       = 0.0;
            filteredRpm        = 0.0;
            readyLatched       = false;
            readyToShoot       = false;
        }
        prevY2 = y2;

        boolean b2 = gamepad2.b;
        if (b2 && !prevB2) {
            shooterRPM         = PRESET_LONG_RPM;
            flywheelOn         = true;
            lastPreset         = "Long (B)";
            commandedTps       = 0.0;
            filteredRpm        = 0.0;
            readyLatched       = false;
            readyToShoot       = false;
        }
        prevB2 = b2;

        // Decide step size (hold LB for fine, RB for coarse)
        int step = STEP_NORMAL;
        if (gamepad2.left_bumper)  step = STEP_FINE;
        if (gamepad2.right_bumper) step = STEP_COARSE;

        // D-pad Up/Down adjust RPM (rising edges)
        boolean du = gamepad2.dpad_up;
        if (flywheelOn && du && !prevDU) {
            shooterRPM   = clamp(shooterRPM + step, 0, MAX_RPM);
            lastPreset   = "—"; // manual override
            readyLatched = false;
            readyToShoot = false;
        }
        prevDU = du;

        boolean dd = gamepad2.dpad_down;
        if (flywheelOn && dd && !prevDD) {
            shooterRPM   = clamp(shooterRPM - step, 0, MAX_RPM);
            lastPreset   = "—"; // manual override
            readyLatched = false;
            readyToShoot = false;
        }
        prevDD = dd;

        // Toggle control mode with LEFT STICK BUTTON: Velocity <-> Power
        boolean ls = gamepad2.left_stick_button;
        if (ls && !prevLS) useVelocityControl = !useVelocityControl;
        prevLS = ls;
        telemetry.addData("FW Mode", useVelocityControl ? "Velocity" : "Power");

        // Clamp logical target RPM
        shooterRPM = clamp(shooterRPM, 0, MAX_RPM);

        // --- Read current velocity ---
        double leftVelTps  = flywheel_Left.getVelocity();
        double rightVelTps = flywheel_Right.getVelocity();
        double leftAbsTps  = Math.abs(leftVelTps);
        double rightAbsTps = Math.abs(rightVelTps);

        double rpmL = (leftAbsTps * 60.0) / TICKS_PER_REV;
        double rpmR = (rightAbsTps * 60.0) / TICKS_PER_REV;
        double avgRpm = 0.5 * (rpmL + rpmR);

        // Initialize filter on first valid reading after start
        if (filteredRpm == 0.0 && flywheelOn && (rpmL > 0 || rpmR > 0)) {
            filteredRpm = avgRpm;
        }

        // Low-pass filter the RPM to reduce noise / jitter
        if (flywheelOn) {
            filteredRpm = RPM_FILTER_ALPHA * avgRpm + (1.0 - RPM_FILTER_ALPHA) * filteredRpm;
        } else {
            filteredRpm = 0.0;
        }

        // RPM error between what you want and what you actually have (filtered)
        double rpmError = shooterRPM - filteredRpm;

        // Convert RPM error to tps error (same units as setVelocity)
        double tpsError = (rpmError / 60.0) * TICKS_PER_REV;

        // Compute theoretical tps corresponding to *scaled* target RPM
        double scaledTargetRPM = shooterRPM * TARGET_SCALE;
        double tpsTarget       = (scaledTargetRPM / 60.0) * TICKS_PER_REV;

        // --- Outer-loop clamp with deadband, ramp limit, and hard upper cap ---
        if (flywheelOn && useVelocityControl) {
            // Only correct if we're outside the tolerance band
            if (Math.abs(rpmError) > RPM_TOLERANCE) {
                double deltaTps = OUTER_KP * tpsError;

                // Limit change per loop to avoid big jumps
                if (deltaTps >  MAX_DELTA_TPS_PER_LOOP) deltaTps =  MAX_DELTA_TPS_PER_LOOP;
                if (deltaTps < -MAX_DELTA_TPS_PER_LOOP) deltaTps = -MAX_DELTA_TPS_PER_LOOP;

                commandedTps += deltaTps;
            }

            // Hard clamp:
            //  - never below 0
            //  - never above tpsTarget (scaled), so we don't drive too aggressively
            commandedTps = clamp(commandedTps, 0.0, tpsTarget);

            flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel_Left.setVelocity(commandedTps);
            flywheel_Right.setVelocity(commandedTps);
        } else if (flywheelOn && !useVelocityControl) {
            // Power mode (no outer-loop magic here)
            double power = clamp(shooterRPM / MAX_RPM, 0, 1);
            flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel_Left.setPower(power);
            flywheel_Right.setPower(power);
            telemetry.addData("Cmd power", "%.2f", power);
        } else {
            commandedTps = 0.0;
            flywheel_Left.setVelocity(0);
            flywheel_Right.setVelocity(0);
        }

        // ===== READY-TO-SHOOT LOGIC (for Long preset only) =====
        boolean longPresetActive = "Long (B)".equals(lastPreset);

        if (flywheelOn && useVelocityControl && longPresetActive) {
            // Are we in the 50–100 RPM below target band?
            boolean inReadyBand =
                    (filteredRpm >= shooterRPM - READY_BELOW_LOW) &&
                            (filteredRpm <= shooterRPM - READY_BELOW_HIGH);

            double now = getRuntime();

            if (inReadyBand && !readyLatched) {
                // Just entered ready band: latch for READY_LATCH_TIME seconds
                readyLatched = true;
                readyLatchedStartTime = now;
            }

            if (readyLatched && (now - readyLatchedStartTime <= READY_LATCH_TIME)) {
                readyToShoot = true;
            } else {
                readyLatched = false;
                readyToShoot = false;
            }
        } else {
            // Outside long preset / mode: clear ready state
            readyLatched = false;
            readyToShoot = false;
        }

        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Start RPM", DEFAULT_RPM);
        telemetry.addData("Step (held)", (gamepad1.left_bumper ? STEP_FINE : gamepad1.right_bumper ? STEP_COARSE : STEP_NORMAL));
        telemetry.addData("Preset Last", lastPreset);
        telemetry.addData("Target RPM (logical)", "%.0f / %.0f", shooterRPM, MAX_RPM);
        telemetry.addData("Scaled target RPM", "%.0f", scaledTargetRPM);
        telemetry.addData("Measured RPM (L,R,avg)", "%.0f, %.0f, %.0f", rpmL, rpmR, avgRpm);
        telemetry.addData("Filtered RPM", "%.0f", filteredRpm);
        telemetry.addData("Cmd tps (outer loop)", "%.0f / %.0f", commandedTps, tpsTarget);
        telemetry.addData("RPM error (filtered)", "%.0f", rpmError);
        telemetry.addData("ReadyLatched", readyLatched);
        telemetry.addData("ReadyToShoot (Long)", readyToShoot);
    }

    // ===================== THIRD STAGE (CR SERVO) =====================
    private void runThirdStage() {
        // If we're on LONG preset and auto ready-to-shoot is true,
        // override manual control and feed automatically.
        if (readyToShoot && "Long (B)".equals(lastPreset) && flywheelOn && useVelocityControl) {
            thirdStage.setPower(-1.0);  // REVERSED direction for auto feed
            telemetry.addData("ThirdStage", "AUTO FEED (READY)");
            telemetry.addData("ThirdStage Power", "%.2f", -1.0);
            return;
        }

        // If we're on long preset but NOT ready, force stop (no accidental spin)
        if ("Long (B)".equals(lastPreset) && flywheelOn && useVelocityControl) {
            thirdStage.setPower(0.0);
            telemetry.addData("ThirdStage", "AUTO HOLD (NOT READY)");
            telemetry.addData("ThirdStage Power", "%.2f", 0.0);
            return;
        }

        // ===== Manual control for other presets =====
        boolean dl = gamepad2.dpad_left;
        if (dl && !prevDL) thirdDir = (thirdDir == -1) ? 0 : -1;
        prevDL = dl;

        boolean dr = gamepad2.dpad_right;
        if (dr && !prevDR) thirdDir = (thirdDir == +1) ? 0 : +1;
        prevDR = dr;

        double power = (thirdDir == 0) ? 0.0 : (thirdDir < 0 ? -1.0 : 1.0);
        thirdStage.setPower(power);

        telemetry.addData("ThirdStage", thirdDir == 0 ? "STOP" : (thirdDir > 0 ? "RIGHT" : "LEFT"));
        telemetry.addData("ThirdStage Power", "%.2f", power);
    }

    // ===================== INTAKE =====================
    private void runIntake() {
        // X toggles intake reverse (-1), B holds intake forward (+1)  [gamepad1]
        boolean x = gamepad1.x;
        if (x && !prevX) intakeOn = !intakeOn;
        prevX = x;

        boolean b = gamepad1.b;

        if (b) {
            intake.setPower(-1.0);        // hold forward (your chosen direction)
        } else if (intakeOn) {
            intake.setPower(1.0);         // toggled reverse
        } else {
            intake.setPower(0.0);         // stop
        }
    }

    @Override
    public void stop() {
        // Stop drive
        right_f.setPower(0);
        left_f.setPower(0);
        right_b.setPower(0);
        left_b.setPower(0);

        // Stop mechanisms
        flywheel_Left.setVelocity(0);
        flywheel_Right.setVelocity(0);
        intake.setPower(0);
        thirdStage.setPower(0.0);
    }

    // ===== Helpers =====
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
