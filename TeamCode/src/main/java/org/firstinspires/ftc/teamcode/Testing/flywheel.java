package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name="FlywheelDebug", group="Testing")
public class flywheel extends OpMode {

    private DcMotorEx flywheel_Left, flywheel_Right;

    // goBILDA 6000 rpm bare motor encoder
    private static final double TICKS_PER_REV     = 28.0;
    private static final double MAX_RPM           = 6000.0;
    private static final double MAX_TICKS_PER_SEC = (TICKS_PER_REV * MAX_RPM) / 60.0; // 2800

    // Presets
    private static final int PRESET_FAR_RPM  = 4500;
    private static final int PRESET_NEAR_RPM = 3700;
    private static final int STEP_RPM        = 100;   // fine tuning with D-pad

    private boolean flywheelOn = false;
    private boolean prevA  = false;
    private boolean prevLB = false;
    private boolean prevRB = false;
    private boolean prevDU = false;
    private boolean prevDD = false;

    private double shooterRPM       = 0.0;
    private double shooterTpsTarget = 0.0;

    @Override
    public void init() {
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        // Use encoders for velocity
        flywheel_Left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel_Right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Coasting
        flywheel_Left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Shared shaft through miter gears -> same sign velocity, opposite direction
        flywheel_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Force reasonable PIDF so we are not using some ancient kF from another OpMode
        double kP = 15.0;
        double kI = 0.0;
        double kD = 3.0;
        double kF = 0.0;   // start with 0 feedforward; we can add later
        flywheel_Left.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        flywheel_Right.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        telemetry.addLine("FlywheelDebug ready. A=ON/OFF, RB=4500, LB=3700, Dpad +/-100");
    }

    @Override
    public void loop() {
        // --- Buttons ---
        boolean a  = gamepad1.a;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean du = gamepad1.dpad_up;
        boolean dd = gamepad1.dpad_down;

        // A toggles ON/OFF
        if (a && !prevA) {
            flywheelOn = !flywheelOn;
            if (flywheelOn && shooterRPM <= 0) {
                shooterRPM = PRESET_FAR_RPM; // default when turning on
            }
        }
        prevA = a;

        // LB = NEAR preset
        if (lb && !prevLB) {
            shooterRPM = PRESET_NEAR_RPM;
        }
        prevLB = lb;

        // RB = FAR preset
        if (rb && !prevRB) {
            shooterRPM = PRESET_FAR_RPM;
        }
        prevRB = rb;

        // D-pad fine tune
        if (flywheelOn && du && !prevDU) shooterRPM += STEP_RPM;
        if (flywheelOn && dd && !prevDD) shooterRPM -= STEP_RPM;
        prevDU = du;
        prevDD = dd;

        shooterRPM = clamp(shooterRPM, 0, MAX_RPM);
        shooterTpsTarget = (shooterRPM / 60.0) * TICKS_PER_REV;
        double targetTps = Math.abs(shooterTpsTarget);

        // --- Command motors ---
        if (flywheelOn && shooterRPM > 0) {
            flywheel_Left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flywheel_Right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            flywheel_Left.setVelocity(targetTps);
            flywheel_Right.setVelocity(targetTps);
        } else {
            flywheel_Left.setVelocity(0);
            flywheel_Right.setVelocity(0);
        }

        // --- Read back ---
        double leftVel  = flywheel_Left.getVelocity();   // tps (may be ±)
        double rightVel = flywheel_Right.getVelocity();
        double leftAbs  = Math.abs(leftVel);
        double rightAbs = Math.abs(rightVel);
        double rpmL     = (leftAbs * 60.0) / TICKS_PER_REV;
        double rpmR     = (rightAbs * 60.0) / TICKS_PER_REV;

        // Simple preset label
        String preset = "Custom";
        if (Math.abs(shooterRPM - PRESET_FAR_RPM) < 5)  preset = "FAR (4500)";
        if (Math.abs(shooterRPM - PRESET_NEAR_RPM) < 5) preset = "NEAR (3700)";

        // Ready check (±3% of target on left motor)
        double tol = 0.03 * shooterRPM;
        boolean ready = shooterRPM > 0 && Math.abs(rpmL - shooterRPM) <= tol;

        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Preset", preset);
        telemetry.addData("Target RPM", "%.0f / %.0f", shooterRPM, MAX_RPM);
        telemetry.addData("Target tps", "%.0f / %.0f", targetTps, MAX_TICKS_PER_SEC);
        telemetry.addData("Measured tps (L,R)", "%.0f, %.0f", leftAbs, rightAbs);
        telemetry.addData("Measured RPM (L,R)", "%.0f, %.0f", rpmL, rpmR);
        telemetry.addData("Signs (L,R)", "%s, %s", (leftVel>=0?"+":"-"), (rightVel>=0?"+":"-"));
        telemetry.addData("READY", ready ? "✅" : "⏳");
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}


