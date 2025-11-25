package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="setVelocityTesting", group="TeleOp")
public class setVelocityTesting extends OpMode {

    // ===== DRIVE =====
    private DcMotorEx right_b, left_f, right_f, left_b;

    // ===== MECHANISMS =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor   intake;
    private CRServo   thirdStage;

    // ===== FLYWHEEL CONSTANTS =====
    private static final double TICKS_PER_REV = 28.0;   // goBILDA 5202 encoder
    private static final double MAX_RPM       = 5000.0; // safety cap

    // Driver RPM settings
    private static final int DEFAULT_RPM = 3300;
    private static final int STEP_FINE   = 100;
    private static final int STEP_NORMAL = 250;
    private static final int STEP_COARSE = 800;

    // Preset RPMs (gamepad2)
    private static final int PRESET_RPM_X = 3300; // X
    private static final int PRESET_RPM_Y = 3500; // Y
    private static final int PRESET_RPM_B = 3800; // B

    // ===== STATE =====
    private boolean flywheelOn = false;
    private double  shooterRPM = DEFAULT_RPM;

    // edge detection
    private boolean prevA  = false;
    private boolean prevDU = false;
    private boolean prevDD = false;
    private boolean prevLB = false;
    private boolean prevRB = false;
    private boolean prevDL = false;
    private boolean prevDR = false;

    // gamepad2 preset edges
    private boolean prevX2 = false;
    private boolean prevY2 = false;
    private boolean prevB2 = false;

    // gamepad1 intake toggle edge
    private boolean prevX  = false;

    private boolean intakeOn = false;
    private int     thirdDir = 0;   // -1,0,+1

    private double driverScale = 1.0;

    @Override
    public void init() {
        // ---- DRIVE ----
        left_f  = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b  = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

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

        // ---- FLYWHEEL ----
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        // ==== YOUR MANUAL PIDF (Velocity) ====
        double P = 1.285;
        double I = 0.1285;
        double D = 0.0;
        double F = 12.85;
        flywheel_Left.setVelocityPIDFCoefficients(P, I, D, F);
        flywheel_Right.setVelocityPIDFCoefficients(P, I, D, F);

        // ---- INTAKE + THIRD STAGE ----
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        thirdStage = hardwareMap.get(CRServo.class, "thirdStage");
        thirdStage.setPower(0.0);

        telemetry.addLine("Flywheel PIDF Test TeleOp initialized");
    }

    @Override
    public void loop() {
        drive();
        runFlywheelSimple();
        runThirdStage();
        runIntake();
        telemetry.update();
    }

    // ===================== DRIVE =====================
    private void drive() {
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;

        double y  = -gamepad1.left_stick_y;
        double x  = -gamepad1.right_stick_x;
        double rx = -gamepad1.left_stick_x;

        double ys  = y  * driverScale;
        double xs  = x  * driverScale;
        double rxs = rx * driverScale;

        double d = Math.max(Math.abs(ys) + Math.abs(xs) + Math.abs(rxs), 1.0);

        right_f.setPower(( ys + xs + rxs) / d);
        left_b.setPower( ( ys - xs + rxs) / d);
        left_f.setPower( ( ys - xs - rxs) / d);
        right_b.setPower(( ys + xs - rxs) / d);
    }

    // ===================== SIMPLE FLYWHEEL VELOCITY CONTROL =====================
    private void runFlywheelSimple() {
        // A toggles flywheel ON/OFF
        boolean a = gamepad2.a;
        if (a && !prevA) {
            flywheelOn = !flywheelOn;
        }
        prevA = a;

        // === PRESETS: X/Y/B set fixed RPMs ===
        boolean x2 = gamepad2.x;
        if (x2 && !prevX2) {
            shooterRPM = PRESET_RPM_X;
        }
        prevX2 = x2;

        boolean y2 = gamepad2.y;
        if (y2 && !prevY2) {
            shooterRPM = PRESET_RPM_Y;
        }
        prevY2 = y2;

        boolean b2 = gamepad2.b;
        if (b2 && !prevB2) {
            shooterRPM = PRESET_RPM_B;
        }
        prevB2 = b2;

        // Choose RPM step with bumpers
        int step = STEP_NORMAL;
        boolean lb = gamepad2.left_bumper;
        boolean rb = gamepad2.right_bumper;
        if (lb && !prevLB) {
            // just edge tracking; step selection is instantaneous
        }
        if (rb && !prevRB) {
            // same here
        }
        prevLB = lb;
        prevRB = rb;

        if (lb) step = STEP_FINE;
        if (rb) step = STEP_COARSE;

        // D-pad up/down to adjust target RPM (on top of presets)
        boolean du = gamepad2.dpad_up;
        if (du && !prevDU) {
            shooterRPM += step;
        }
        prevDU = du;

        boolean dd = gamepad2.dpad_down;
        if (dd && !prevDD) {
            shooterRPM -= step;
        }
        prevDD = dd;

        // Clamp to safe range
        shooterRPM = clamp(shooterRPM, 0, MAX_RPM);

        if (flywheelOn) {
            double tpsTarget = (shooterRPM / 60.0) * TICKS_PER_REV;
            flywheel_Left.setVelocity(tpsTarget);
            flywheel_Right.setVelocity(tpsTarget);
        } else {
            flywheel_Left.setVelocity(0);
            flywheel_Right.setVelocity(0);
        }

        // Telemetry: measured RPM
        double leftTps  = Math.abs(flywheel_Left.getVelocity());
        double rightTps = Math.abs(flywheel_Right.getVelocity());
        double leftRpm  = (leftTps  * 60.0) / TICKS_PER_REV;
        double rightRpm = (rightTps * 60.0) / TICKS_PER_REV;
        double avgRpm   = 0.5 * (leftRpm + rightRpm);

        telemetry.addLine("=== Flywheel PIDF Test ===");
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f", shooterRPM);
        telemetry.addData("Measured RPM L/R/avg", "%.0f / %.0f / %.0f",
                leftRpm, rightRpm, avgRpm);
    }

    // ===================== THIRD STAGE (manual only) =====================
    private void runThirdStage() {
        boolean dl = gamepad2.dpad_left;
        if (dl && !prevDL) thirdDir = (thirdDir == -1) ? 0 : -1;
        prevDL = dl;

        boolean dr = gamepad2.dpad_right;
        if (dr && !prevDR) thirdDir = (thirdDir == +1) ? 0 : +1;
        prevDR = dr;

        double power = (thirdDir == 0) ? 0.0 : (thirdDir < 0 ? -1.0 : 1.0);
        thirdStage.setPower(power);

        telemetry.addData("ThirdStage", thirdDir == 0 ? "STOP" :
                (thirdDir > 0 ? "RIGHT" : "LEFT"));
        telemetry.addData("ThirdStage Power", "%.2f", power);
    }

    // ===================== INTAKE =====================
    private void runIntake() {
        boolean x = gamepad1.x;
        if (x && !prevX) intakeOn = !intakeOn;
        prevX = x;

        boolean b = gamepad1.b;

        if (b) {
            intake.setPower(-1.0);        // hold forward (your chosen direction)
        } else if (intakeOn) {
            intake.setPower(1.0);         // toggled reverse
        } else {
            intake.setPower(0.0);
        }

        telemetry.addData("IntakeOn(toggle X)", intakeOn);
        telemetry.addData("IntakeHeld(B)", b);
    }

    @Override
    public void stop() {
        right_f.setPower(0);
        left_f.setPower(0);
        right_b.setPower(0);
        left_b.setPower(0);

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
