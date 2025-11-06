package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="firstPrototypeRobot", group="TeleOp")
public class firstPrototypeRobot extends OpMode {
    private DcMotorEx right_b, left_f, right_f, left_b;
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor intake;

    // NEW: third stage continuous rotation servo
    private CRServo thirdStage;

    private boolean flywheelOn = false;
    private boolean intakeOn = false;
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevB = false;

    // NEW: toggle state for thirdStage
    private boolean prevDL = false, prevDR = false;
    // -1 = spinning left, 0 = stop, +1 = spinning right
    private int thirdDir = 0;

    private double driverScale = 1.0;

    private static final double TICKS_PER_REV = 50.0;
    private static final double MAX_RPM = 5000.0;
    private static final double MAX_TICKS_PER_SEC = TICKS_PER_REV * (MAX_RPM / 60.0);

    private double shooterRPM = 2000.0;
    private double shooterTpsTarget = 0.0;

    @Override
    public void init() {
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

        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");
        intake = hardwareMap.get(DcMotor.class, "intake");

        flywheel_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // NEW: map the third stage CR servo
        thirdStage = hardwareMap.get(CRServo.class, "thirdStage");
        thirdStage.setPower(0.0);
    }

    @Override
    public void loop() {
        drive();
        runFlywheel();
        runThirdStage();   // NEW: handle third-stage CR servo
        runIntake();
    }

    private void drive(){
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

    private void runFlywheel(){
        boolean aPressed = gamepad1.a;

        if (aPressed && !prevA) {
            flywheelOn = !flywheelOn;
            if (flywheelOn) {
                double rpm = clamp(shooterRPM, 0.0, MAX_RPM);
                shooterTpsTarget = (rpm / 60.0) * TICKS_PER_REV;
            }
        }
        prevA = aPressed;

        if (flywheelOn) {
            flywheel_Left.setVelocity(shooterTpsTarget);
            flywheel_Right.setVelocity(-shooterTpsTarget);
        } else {
            flywheel_Left.setVelocity(0);
            flywheel_Right.setVelocity(0);
        }

        double leftVel  = flywheel_Left.getVelocity();
        double rightVel = flywheel_Right.getVelocity();

        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f / %.0f", shooterRPM, MAX_RPM);
        telemetry.addData("Target vel (tps)", "%.0f", shooterTpsTarget);
        telemetry.addData("Measured (L,R) tps", "%.0f, %.0f", leftVel, rightVel);
    }

    // NEW: toggle control for thirdStage CR servo using dpad left/right
    private void runThirdStage() {
        boolean dl = gamepad1.dpad_left;
        if (dl && !prevDL) {
            thirdDir = (thirdDir == -1) ? 0 : -1;
        }
        prevDL = dl;

        boolean dr = gamepad1.dpad_right;
        if (dr && !prevDR) {
            thirdDir = (thirdDir == +1) ? 0 : +1;
        }
        prevDR = dr;

        double power = (thirdDir == 0) ? 0.0 : (thirdDir < 0 ? -1.0 : 1.0);
        thirdStage.setPower(power);

        telemetry.addData("ThirdStage", thirdDir == 0 ? "STOP" : (thirdDir > 0 ? "RIGHT" : "LEFT"));
        telemetry.addData("ThirdStage Power", "%.2f", power);
        telemetry.update();
    }

    private void runIntake() {
        boolean xPressed = gamepad1.x;
        if (xPressed && !prevX) {
            intakeOn = !intakeOn;
        }
        prevX = xPressed;

        boolean bHeld = gamepad1.b;

        if (bHeld) {
            intake.setPower(1.0);
        } else if (intakeOn) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
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

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
