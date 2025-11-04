package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="shootingTeleOp", group="TeleOp")
public class shootingTeleOp extends OpMode {
    // private DcMotorEx right_b, left_f, right_f, left_b;
    private DcMotor flywheel_Left, flywheel_Right;
    //private DcMotor intake;


    private boolean flywheelOn = false;
    private boolean prevA = false;

    private double driverScale = 1.0;

    // === Added for click-to-step power control ===
    private boolean prevLB = false;
    private boolean prevRB = false;
    private boolean powerInitialized = false; // ensures first A-click sets power to 1.0 only once
    private double  flywheelPowerMag = 1.00;     // 0.00..1.00, magnitude only
    private static final double FLYWHEEL_STEP = 0.05;

    @Override
    public void init() {
        // left_f  = hardwareMap.get(DcMotorEx.class, "leftFront");
        //right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        //left_b  = hardwareMap.get(DcMotorEx.class, "leftBack");
        // right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

        // left_f.setDirection(DcMotor.Direction.REVERSE);
        //  right_b.setDirection(DcMotor.Direction.REVERSE);

        // right_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // left_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // right_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //  left_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // right_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // left_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //   right_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //  left_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mechanisms
        flywheel_Left = hardwareMap.get(DcMotor.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotor.class, "flywheel_Right");
        flywheel_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        //  intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //  intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    //region Loop
    @Override
    public void loop() {
        //drive();
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

        // right_f.setPower(( ys + xs + rxs) / d);
        // left_b.setPower( ( ys - xs + rxs) / d);
        // left_f.setPower( ( ys - xs - rxs) / d);
        //  right_b.setPower(( ys + xs - rxs) / d);
    }

    private void runFlywheel(){
        boolean aPressed = gamepad1.a;

        // Detect rising edge (button pressed, not held)
        if (aPressed && !prevA) {
            flywheelOn = !flywheelOn; // toggle state

            // First time we ever turn ON via A, force full power
            if (flywheelOn && !powerInitialized) {
                flywheelPowerMag = 1.0;
                powerInitialized = true;
            }
        }
        prevA = aPressed;

        // === Added: step power with LB/RB (rising-edge) ===
        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLB) {
            flywheelPowerMag = clamp01(flywheelPowerMag - FLYWHEEL_STEP); // decrease by 0.05
        }
        prevLB = lb;

        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) {
            flywheelPowerMag = clamp01(flywheelPowerMag + FLYWHEEL_STEP); // increase by 0.05
        }
        prevRB = rb;

        // Apply to motors (left negative, right positive)
        double leftCmd  = flywheelOn ? -flywheelPowerMag : 0.0;
        double rightCmd = flywheelOn ?  flywheelPowerMag : 0.0;

        flywheel_Left.setPower(leftCmd);
        flywheel_Right.setPower(rightCmd);

        // Telemetry: report current set powers (-1..1)
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Power step", "%.2f per click", FLYWHEEL_STEP);
        telemetry.addData("Set magnitude (0..1)", "%.2f", flywheelPowerMag);
        telemetry.addData("Motor Power (L,R)", "%.2f, %.2f  (-1..1)", leftCmd, rightCmd);
    }

    private void runIntake(){
        if (gamepad1.x) {
            // intake.setPower(1.0);     // intake in
        } else if (gamepad1.b) {
            //   intake.setPower(-1.0);    // reverse
        } else {
            //   intake.setPower(0.0);     // stop
        }
    }
//endregion

    @Override
    public void stop() {
        // Stop all drive motors
        // right_f.setPower(0);
        // left_f.setPower(0);
        //  right_b.setPower(0);
        //  left_b.setPower(0);
        flywheel_Left.setPower(0);
        flywheel_Right.setPower(0);
        // intake.setPower(0);
    }

    // === Helper (added) ===
    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
