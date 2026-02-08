package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp with Adjustable RPM", group="TeleOp")
public class TeleOpAdjustable extends OpMode {

    private VoltageFlywheelController flywheelController;

    private DcMotorEx right_b, left_f, right_f, left_b;

    // ===== MECHANISMS =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotorEx intake; // now DcMotorEx for setVelocity()

    // ===== NEW BARRIER (SERVO) =====
    private Servo barrierServo;

    private static final double BARRIER_CLOSED_POS = 0.58;
    private static final double BARRIER_OPEN_POS   = 0;

    private boolean barrierOpen = false;

    private double driverScale = 1.0;

    // ===== STATE =====
    private boolean intakeOn   = false;

    // Edge detection (g1 intake toggles)
    private boolean prevA = false, prevX = false;
    private boolean prevDU = false, prevDD = false, prevDL = false, prevDR = false;

    // gamepad1 right stick button edge + preset state
    private boolean prevRSB1 = false;
    private boolean intakePreset250Enabled = false; // NEW: toggled by gamepad1 RSB

    // gamepad2 button edges
    private boolean prevX2  = false;
    private boolean prevB2  = false;
    private boolean prevLT2 = false;
    private boolean prevRT2 = false;
    private boolean prevLSB2 = false;

    // ===== PRESET RPMs =====
    private static final int PRESET_CLOSE = 3000; // gamepad2 left trigger
    private static final int PRESET_FAR = 2950; // gamepad2 right trigger

    // Fine adjust step and minimum
    private static final int RPM_STEP = 50;
    private static final int MIN_RPM  = 500;

    // ===== FLYWHEEL READY-TO-SHOOT RUMBLE (gamepad2) =====
    private static final double RPM_TOLERANCE = 50.0;
    private boolean flywheelAtSpeed       = false;
    private boolean flywheelReadyRumbled  = false;
    private double  lastTargetRPM         = 0.0;

    // ===================== INTAKE VELOCITY CONSTANTS =====================
    private static final double INTAKE_TICKS_PER_REV = 145.1;

    private static final double INTAKE_RPM_BARRIER_CLOSED = 1150.0;

    private static final double INTAKE_RPM_BARRIER_FAR  = 250.0; // when target is PRESET_MED_LONG_RPM
    private static final double INTAKE_RPM_BARRIER_CLOSE = 250.0;

    private static final double INTAKE_RPM_BARRIER_OPEN_DEFAULT = 250.0; // when target is PRESET_CLOSE

    // NEW: gamepad1 preset RPM (fixed)
    private static final double INTAKE_RPM_PRESET_G1_RSB = 250.0;

    @Override
    public void init() {
        // ---- Map drive ----
        left_f  = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b  = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

        left_f.setDirection(DcMotor.Direction.REVERSE);
        left_b.setDirection(DcMotor.Direction.FORWARD);
        right_f.setDirection(DcMotor.Direction.FORWARD);
        right_b.setDirection(DcMotor.Direction.FORWARD);

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
        intake         = hardwareMap.get(DcMotorEx.class, "intake");

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel_Right.setDirection(DcMotorSimple.Direction.FORWARD);

        // Use the voltage-compensated controller
        flywheelController = new VoltageFlywheelController(hardwareMap);

        // intake uses encoder for velocity control
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // If intake spins the wrong direction vs your old setPower(1.0), uncomment ONE of these:
        // intake.setDirection(DcMotorSimple.Direction.REVERSE);
        // intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // ===== NEW BARRIER SERVO INIT =====
        barrierServo = hardwareMap.get(Servo.class, "barrierServo");
        barrierServo.setPosition(BARRIER_CLOSED_POS);
        barrierOpen = false;

        telemetry.addLine("TeleOpAdjustable init complete.");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive();
        runFlywheel();
        runIntake();
        runBarrier();

        flywheelController.update();
        updateFlywheelReadyRumble();

        // ===== TELEMETRY =====
        telemetry.addLine();
        telemetry.addData("--- Flywheel ---", "");
        telemetry.addData("Flywheel State", flywheelController.isFlywheelOn() ? "ACTIVE" : "OFF");
        telemetry.addData("Target RPM", "%.0f", flywheelController.getTargetRPM());
        telemetry.addData("Actual RPM Left", "%.0f", flywheelController.getCurrentRPM_Left());
        telemetry.addData("Actual RPM Right", "%.0f", flywheelController.getCurrentRPM_Right());
        telemetry.addData("Battery Voltage", "%.2f", flywheelController.getBatteryVoltage());

        // Intake telemetry (current RPM)
        double intakeTicksPerSec = 0.0;
        double intakeRpm = 0.0;
        if (intake != null) {
            intakeTicksPerSec = intake.getVelocity(); // ticks/sec
            intakeRpm = (intakeTicksPerSec * 60.0) / INTAKE_TICKS_PER_REV;
        }

        telemetry.addLine();
        telemetry.addData("--- Intake ---", "");
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Intake Preset 250 (g1 RSB)", intakePreset250Enabled ? "ENABLED" : "OFF"); // NEW
        telemetry.addData("Intake RPM (current)", "%.0f", intakeRpm);
        telemetry.addData("Intake vel (tps)", "%.1f", intakeTicksPerSec);

        telemetry.addLine();
        telemetry.addData("--- Barrier ---", "");
        telemetry.addData("Barrier", barrierOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Barrier Pos", "%.3f", barrierOpen ? BARRIER_OPEN_POS : BARRIER_CLOSED_POS);
        telemetry.addData("Barrier Ctrl", "g2 dpad_left=CLOSE, dpad_right=OPEN, dpad_up=TOGGLE");

        telemetry.update();
    }

    // ===================== FLYWHEEL =====================
    private void runFlywheel() {
        boolean x2   = gamepad2.x;
        boolean y2   = gamepad2.y;
        boolean b2   = gamepad2.b;
        boolean lb2  = gamepad2.left_bumper;
        boolean rb2  = gamepad2.right_bumper;
        boolean a2   = gamepad2.a;
        boolean lsb2 = gamepad2.left_stick_button;

        double ltVal = gamepad2.left_trigger;
        double rtVal = gamepad2.right_trigger;
        boolean lt2  = ltVal > 0.5;
        boolean rt2  = rtVal > 0.5;


        if (lt2 && !prevLT2) setFlywheelPreset(PRESET_CLOSE);
        prevLT2 = lt2;

        if (rt2 && !prevRT2) setFlywheelPreset(PRESET_FAR);
        prevRT2 = rt2;

        if (lsb2 && !prevLSB2) flywheelController.turnFlywheelOff();
        prevLSB2 = lsb2;

        if (b2 && !prevB2 && flywheelController.isFlywheelOn()) {
            double newTarget = flywheelController.getTargetRPM() + RPM_STEP;
            flywheelController.setFlywheelTargetRPM(newTarget);
        }

        if (x2 && !prevX2 && flywheelController.isFlywheelOn()) {
            double newTarget = flywheelController.getTargetRPM() - RPM_STEP;
            if (newTarget < MIN_RPM) newTarget = MIN_RPM;
            flywheelController.setFlywheelTargetRPM(newTarget);
        }

        prevX2 = x2;
        prevB2 = b2;
    }

    private void setFlywheelPreset(int rpm) {
        flywheelController.setFlywheelTargetRPM(rpm);
        flywheelController.turnFlywheelOn();
    }

    // ===================== READY-TO-SHOOT RUMBLE LOGIC (gamepad2) =====================
    private void updateFlywheelReadyRumble() {
        double target = flywheelController.getTargetRPM();

        if (!flywheelController.isFlywheelOn() || target <= 0.0) {
            flywheelAtSpeed      = false;
            flywheelReadyRumbled = false;
            lastTargetRPM        = target;
            return;
        }

        if (target != lastTargetRPM) {
            flywheelAtSpeed      = false;
            flywheelReadyRumbled = false;
        }

        double errL = Math.abs(flywheelController.getErrorRPM_Left());
        double errR = Math.abs(flywheelController.getErrorRPM_Right());
        boolean atSpeedNow = (errL <= RPM_TOLERANCE) && (errR <= RPM_TOLERANCE);

        if (atSpeedNow && !flywheelAtSpeed && !flywheelReadyRumbled) {
            if (gamepad2 != null) gamepad2.rumbleBlips(1);
            flywheelReadyRumbled = true;
        }

        flywheelAtSpeed = atSpeedNow;
        lastTargetRPM   = target;
    }

    // ===================== DRIVE =====================
    private void drive() {
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;

        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double ys  = y  * driverScale;
        double xs  = x  * driverScale;
        double rxs = rx * driverScale;

        double d = Math.max(Math.abs(ys) + Math.abs(xs) + Math.abs(rxs), 1.0);

        right_f.setPower(( ys - xs - rxs) / d);
        left_b.setPower( ( ys - xs + rxs) / d);
        left_f.setPower( ( ys + xs + rxs) / d);
        right_b.setPower(( ys + xs - rxs) / d);
    }

    // ===================== INTAKE (VELOCITY CONTROL) =====================
    private void runIntake() {
        // g1 intake on/off toggle
        boolean x = gamepad1.x;
        if (x && !prevX) intakeOn = !intakeOn;
        prevX = x;

        // NEW: g1 right stick button toggles fixed 250 RPM preset
        boolean rsb1 = gamepad1.right_stick_button;
        if (rsb1 && !prevRSB1) {
            intakePreset250Enabled = !intakePreset250Enabled;
        }
        prevRSB1 = rsb1;

        boolean b = gamepad1.b;

        // Convert RPM -> ticks/sec because DcMotorEx.setVelocity() expects ticks/sec.
        double targetRpm = 0.0;

        // B overrides everything: hard reverse while held
        if (b) {
            targetRpm = -INTAKE_RPM_BARRIER_CLOSED; // -1150 RPM
        } else if (intakeOn) {

            // NEW: If preset is enabled, force intake RPM to 250 (regardless of barrier/flywheel preset logic)
            if (intakePreset250Enabled) {
                targetRpm = INTAKE_RPM_PRESET_G1_RSB; // 250 RPM
            } else {
                // Original logic:
                if (!barrierOpen) {
                    // Barrier closed: full intake speed
                    targetRpm = INTAKE_RPM_BARRIER_CLOSED;
                } else {
                    // Barrier open: choose based on the CURRENT flywheel target preset
                    double flywheelTarget = flywheelController.getTargetRPM();

                    if (Math.abs(flywheelTarget - PRESET_FAR) < 0.5) {
                        targetRpm = INTAKE_RPM_BARRIER_FAR;   // 250
                    } else if (Math.abs(flywheelTarget - PRESET_CLOSE) < 0.5) {
                        targetRpm = INTAKE_RPM_BARRIER_CLOSE;  // 200
                    } else {
                        targetRpm = INTAKE_RPM_BARRIER_OPEN_DEFAULT;    // 250 fallback
                    }
                }
            }

        } else {
            // off
            intake.setPower(0.0);
            return;
        }

        double ticksPerSecond = (targetRpm * INTAKE_TICKS_PER_REV) / 60.0;
        intake.setVelocity(ticksPerSecond);
    }

    // ===================== BARRIER (SERVO) =====================
    private void runBarrier() {
        if (barrierServo == null) return;

        boolean dl = gamepad2.dpad_left;
        boolean dr = gamepad2.dpad_right;
        boolean du = gamepad2.dpad_up;

        if (dl && !prevDL) {
            barrierServo.setPosition(BARRIER_CLOSED_POS);
            barrierOpen = false;
        }

        if (dr && !prevDR) {
            barrierServo.setPosition(BARRIER_OPEN_POS);
            barrierOpen = true;
        }

        if (du && !prevDU) {
            barrierOpen = !barrierOpen;
            barrierServo.setPosition(barrierOpen ? BARRIER_OPEN_POS : BARRIER_CLOSED_POS);
        }

        prevDL = dl;
        prevDR = dr;
        prevDU = du;
    }

    @Override
    public void stop() {
        right_f.setPower(0);
        left_f.setPower(0);
        right_b.setPower(0);
        left_b.setPower(0);

        flywheel_Left.setPower(0);
        flywheel_Right.setPower(0);
        intake.setPower(0);

        // safe state
        if (barrierServo != null) {
            barrierServo.setPosition(BARRIER_CLOSED_POS);
            barrierOpen = false;
        }

        if (gamepad1 != null) gamepad1.stopRumble();
        if (gamepad2 != null) gamepad2.stopRumble();
    }
}
