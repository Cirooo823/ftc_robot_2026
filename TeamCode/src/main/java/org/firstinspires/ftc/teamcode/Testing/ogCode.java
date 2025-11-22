package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="ogCode", group="TeleOp")
public class ogCode extends OpMode {
    // ===== DRIVE =====
    private DcMotorEx right_b, left_f, right_f, left_b;

    // ===== MECHANISMS =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor   intake;
    private CRServo   thirdStage;

    // ===== FLYWHEEL CONSTANTS (goBILDA 6000 RPM) =====
    private static final double TICKS_PER_REV     = 28.0;                    // 6000 RPM bare motor encoder
    private static final double MAX_RPM           = 4500.0;
    private static final double MAX_TICKS_PER_SEC = (TICKS_PER_REV * MAX_RPM) / 60.0; // 2800 tps

    // Single-start target + step sizes
    private static final int DEFAULT_RPM   = 5000;  // NOTE: > MAX_RPM; will clamp to MAX_RPM on start
    private static final int STEP_FINE     = 100;   // hold LB for fine steps
    private static final int STEP_NORMAL   = 300;   // default step (no bumper)
    private static final int STEP_COARSE   = 1000;  // hold RB for big steps

    // ===== NEW: PRESET RPMs (replace with your tested values) =====
    private static final int PRESET_SHORT_RPM  = 3000; // gamepad2.X
    private static final int PRESET_MED_RPM    = 3700; // gamepad2.Y, measured should be about 3200
    private static final int PRESET_LONG_RPM   = 4500; // gamepad2.B measured shoudl be about 3800-3900

    // ===== STATE =====
    private boolean flywheelOn = false;
    private boolean intakeOn   = false;

    // Edge detection (g1 intake toggles)
    private boolean prevA=false, prevX=false, prevB=false;
    private boolean prevDU=false, prevDD=false, prevDL=false, prevDR=false;

    // ===== NEW: gamepad2 preset button edges =====
    private boolean prevX2=false, prevY2=false, prevB2=false;

    // Troubleshooting toggle
    private boolean useVelocityControl = true;  // Left-stick click toggles Velocity <-> Power
    private boolean prevLS = false;

    // Flywheel target (driver-adjustable)
    private double shooterRPM       = 2000.0; // will snap to DEFAULT_RPM on A=ON
    private double shooterTpsTarget = 0.0;

    // Third stage (-1=left, 0=stop, +1=right)
    private int thirdDir = 0;

    // Drive scale
    private double driverScale = 1.0;

    // ===== NEW: last preset label for telemetry =====
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

        // Leave PIDF defaults while determining targets (avoid kF saturation)
        // Example (optional later):
        // flywheel_Left.setVelocityPIDFCoefficients(18, 0, 10, 0);
        // flywheel_Right.setVelocityPIDFCoefficients(18, 0, 10, 0);
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
                shooterRPM = DEFAULT_RPM; // will clamp below
                lastPreset = "Start(Default)";
            }
        }
        prevA = a;

        // ===== PRESETS: gamepad2.X / Y / B =====
        boolean x2 = gamepad2.x;
        if (x2 && !prevX2) {
            shooterRPM = PRESET_SHORT_RPM;
            flywheelOn = true;           // ensure spin-up to preset
            lastPreset = "Short (X)";
        }
        prevX2 = x2;

        boolean y2 = gamepad2.y;
        if (y2 && !prevY2) {
            shooterRPM = PRESET_MED_RPM;
            flywheelOn = true;
            lastPreset = "Medium (Y)";
        }
        prevY2 = y2;

        boolean b2 = gamepad2.b;
        if (b2 && !prevB2) {
            shooterRPM = PRESET_LONG_RPM;
            flywheelOn = true;
            lastPreset = "Long (B)";
        }
        prevB2 = b2;

        // Decide step size (hold LB for fine, RB for coarse)
        int step = STEP_NORMAL;
        if (gamepad2.left_bumper)  step = STEP_FINE;
        if (gamepad2.right_bumper) step = STEP_COARSE;

        // D-pad Up/Down adjust RPM (rising edges)
        boolean du = gamepad2.dpad_up;
        if (flywheelOn && du && !prevDU) {
            shooterRPM = clamp(shooterRPM + step, 0, MAX_RPM);
            lastPreset = "—"; // manual override
        }
        prevDU = du;

        boolean dd = gamepad2.dpad_down;
        if (flywheelOn && dd && !prevDD) {
            shooterRPM = clamp(shooterRPM - step, 0, MAX_RPM);
            lastPreset = "—"; // manual override
        }
        prevDD = dd;

        // Toggle control mode with LEFT STICK BUTTON: Velocity <-> Power
        boolean ls = gamepad2.left_stick_button;
        if (ls && !prevLS) useVelocityControl = !useVelocityControl;
        prevLS = ls;
        telemetry.addData("FW Mode", useVelocityControl ? "Velocity" : "Power");

        // Compute target tps from RPM
        shooterRPM       = clamp(shooterRPM, 0, MAX_RPM);
        shooterTpsTarget = (shooterRPM / 60.0) * TICKS_PER_REV;
        double targetTps = Math.abs(shooterTpsTarget); // positive; directions handle spin

        // Command motors
        if (flywheelOn) {
            if (useVelocityControl) {
                flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flywheel_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flywheel_Left.setVelocity(targetTps);
                flywheel_Right.setVelocity(targetTps);
            } else {
                double power = clamp(shooterRPM / MAX_RPM, 0, 1);
                flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheel_Left.setPower(power);
                flywheel_Right.setPower(power);
                telemetry.addData("Cmd power", "%.2f", power);
            }
        } else {
            flywheel_Left.setVelocity(0);
            flywheel_Right.setVelocity(0);
        }

        // Readback / Telemetry
        double leftVel  = flywheel_Left.getVelocity();
        double rightVel = flywheel_Right.getVelocity();
        double leftAbs  = Math.abs(leftVel);
        double rightAbs = Math.abs(rightVel);
        double rpmL     = (leftAbs * 60.0) / TICKS_PER_REV;
        double rpmR     = (rightAbs * 60.0) / TICKS_PER_REV;

        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Start RPM", DEFAULT_RPM);
        telemetry.addData("Step (held)", (gamepad1.left_bumper ? STEP_FINE : gamepad1.right_bumper ? STEP_COARSE : STEP_NORMAL));
        telemetry.addData("Preset Last", lastPreset);
        telemetry.addData("Target RPM", "%.0f / %.0f", shooterRPM, MAX_RPM);
        telemetry.addData("Target tps", "%.0f / %.0f", targetTps, MAX_TICKS_PER_SEC);
        telemetry.addData("Measured tps (L,R)", "%.0f, %.0f", leftAbs, rightAbs);
        telemetry.addData("Measured RPM (L,R)", "%.0f, %.0f", rpmL, rpmR);
        telemetry.addData("Signs (L,R)", "%s, %s", (leftVel>=0?"+":"-"), (rightVel>=0?"+":"-"));
    }

    // ===================== THIRD STAGE (CR SERVO) =====================
    private void runThirdStage() {
        // dpad_left toggles LEFT spin on/off
        boolean dl = gamepad2.dpad_left;
        if (dl && !prevDL) thirdDir = (thirdDir == -1) ? 0 : -1;
        prevDL = dl;

        // dpad_right toggles RIGHT spin on/off
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
            intake.setPower(1.0);        // hold forward
        } else if (intakeOn) {
            intake.setPower(-1.0);       // toggled reverse
        } else {
            intake.setPower(0.0);        // stop
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