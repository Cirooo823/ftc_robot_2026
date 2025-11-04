package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="firstPrototypeRobot", group="TeleOp")
public class firstPrototypeRobot extends OpMode {
    private DcMotorEx right_b, left_f, right_f, left_b;

    // CHANGED: use DcMotorEx so we can call setVelocity()/getVelocity()
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor intake;

    private boolean flywheelOn = false;
    private boolean prevA = false;

    private double driverScale = 1.0;

    // === NEW: velocity control state ===
    // Replace these with your exact motor specs if different:
    // 5203 Series Yellow Jacket, 1:1 ratio
    private static final double TICKS_PER_REV = 560.0;   // goBILDA encoder: 28 * 20 = 560 ticks / output rev
    private static final double MAX_RPM       = 6000.0;  // per product spec (12V no-load)
    private static final double MAX_TICKS_PER_SEC = TICKS_PER_REV * (MAX_RPM / 60.0); // = 56,000 tps

    private double targetTicksPerSec = 0.0;                // current setpoint
    private static final double STEP_TPS = 0.05 * MAX_TICKS_PER_SEC; // 5% steps
    private boolean prevLB = false, prevRB = false;
    private boolean powerInitialized = false;              // ensure first A sets full speed once

    @Override
    public void init() {
        left_f  = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b  = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

        left_f.setDirection(DcMotor.Direction.REVERSE);
        right_b.setDirection(DcMotor.Direction.REVERSE);

        right_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mechanisms
        // CHANGED: map as DcMotorEx to use velocity functions
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        // CHANGED: velocity mode needs encoders; reset then RUN_USING_ENCODER
        flywheel_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Coasting is typical for flywheels
        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // intake stays as-is
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //region Loop
    @Override
    public void loop() {
        drive();
        runFlywheel();
        runIntake();
    }
    //endregion

    //region Class Methods
    private void drive(){
        driverScale = gamepad1.left_bumper ? 0.25 : 1.0;

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

    private void runFlywheel(){
        boolean aPressed = gamepad1.a;

        // Toggle ON/OFF on A (rising edge)
        if (aPressed && !prevA) {
            flywheelOn = !flywheelOn;
            // First time A turns it ON, force full speed once
            if (flywheelOn && !powerInitialized) {
                targetTicksPerSec = MAX_TICKS_PER_SEC;
                powerInitialized = true;
            }
        }
        prevA = aPressed;

        // Click-to-step velocity with LB/RB (rising edges)
        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLB) {
            targetTicksPerSec = clamp(targetTicksPerSec - STEP_TPS, 0.0, MAX_TICKS_PER_SEC);
        }
        prevLB = lb;

        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) {
            targetTicksPerSec = clamp(targetTicksPerSec + STEP_TPS, 0.0, MAX_TICKS_PER_SEC);
        }
        prevRB = rb;

        if (flywheelOn) {
            // Apply velocity setpoint (left positive, right negative for counter-rotation)
            flywheel_Left.setVelocity( targetTicksPerSec);
            flywheel_Right.setVelocity(-targetTicksPerSec);
        } else {
            // Stop
            flywheel_Left.setVelocity(0);
            flywheel_Right.setVelocity(0);
        }

        // Telemetry: target & measured velocity (ticks/sec) and % of max
        double leftVel  = flywheel_Left.getVelocity();
        double rightVel = flywheel_Right.getVelocity();

        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Target vel (tps)", "%.0f / %.0f (%.0f%%)",
                targetTicksPerSec, MAX_TICKS_PER_SEC, 100.0 * targetTicksPerSec / MAX_TICKS_PER_SEC);
        telemetry.addData("Measured (L,R) tps", "%.0f, %.0f", leftVel, rightVel);
        telemetry.update();
    }

    private void runIntake(){
        if (gamepad1.x) {
            intake.setPower(1.0);     // intake in
        } else if (gamepad1.b) {
            intake.setPower(-1.0);    // reverse
        } else {
            intake.setPower(0.0);     // stop
        }
    }
    //endregion

    @Override
    public void stop() {
        // Stop all drive motors
        right_f.setPower(0);
        left_f.setPower(0);
        right_b.setPower(0);
        left_b.setPower(0);
        flywheel_Left.setVelocity(0);
        flywheel_Right.setVelocity(0);
        intake.setPower(0);
    }

    // === Helpers ===
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
