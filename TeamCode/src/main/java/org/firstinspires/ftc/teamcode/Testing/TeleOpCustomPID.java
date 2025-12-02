package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// IMPORT YOUR CUSTOM FLYWHEEL CONTROLLER CLASS


@TeleOp(name="TeleOpCustomPID", group="TeleOp")
public class TeleOpCustomPID extends OpMode {
    private FlywheelController flywheelController;

    private DcMotorEx right_b, left_f, right_f, left_b;

    // ===== MECHANISMS =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor intake;
    private CRServo thirdStage;

    private double driverScale = 1.0;

    // ===== STATE =====
    private boolean flywheelOn = false;
    private boolean intakeOn = false;

    // Edge detection (g1 intake toggles)
    private boolean prevA = false, prevX = false;
    private boolean prevDU = false, prevDD = false, prevDL = false, prevDR = false;

    // gamepad2 preset button edges
    private boolean prevA2 = false, prevX2 = false, prevY2 = false, prevB2 = false, prevLB2 = false, prevRB2 = false;

    private int thirdDir = 0;

    // ===== PRESET RPMs (keep your current values) =====
    private static final int PRESET_SHORT_RPM = 3300; // gamepad2.X
    private static final int PRESET_SHORT_MED_RPM = 3500; // gamepad2 lb
    private static final int PRESET_MED_RPM = 3700; // gamepad2.Y
    private static final int PRESET_MED_LONG_RPM = 3900; // gamepad 2 rb
    private static final int PRESET_LONG_RPM = 4100; // gamepad2.B


    @Override
    public void init() {
        // ---- Map drive ----
        left_f = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b = hardwareMap.get(DcMotorEx.class, "leftBack");
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

        // ---- Mechanisms ----
        flywheel_Left = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");
        intake = hardwareMap.get(DcMotor.class, "intake");

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelController = new FlywheelController(hardwareMap);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        thirdStage = hardwareMap.get(CRServo.class, "thirdStage");
        thirdStage.setPower(0.0);
    }

    @Override
    public void loop() {
        drive();
        runFlywheel();
        runThirdStage();
        runIntake();

        flywheelController.update();

        telemetry.addData("--- Flywheel ---", "");
        telemetry.addData("Flywheel State", flywheelController.isFlywheelOn() ? "ACTIVE" : "OFF");
        telemetry.addData("Target RPM", "%.0f", flywheelController.getTargetRPM());
        telemetry.addData("Actual RPM L", "%.0f", flywheelController.getCurrentRPM_Left());
        telemetry.addData("Actual RPM R", "%.0f", flywheelController.getCurrentRPM_Right());
        telemetry.addData("Error RPM L", "%.0f", flywheelController.getErrorRPM_Left());
        telemetry.addData("Error RPM R", "%.0f", flywheelController.getErrorRPM_Right());
        telemetry.update();
    }

    private void runFlywheel() {
        boolean x2 = gamepad2.x;
        if (x2 && !prevX2) {
            flywheelController.turnFlywheelOn();
            flywheelController.setFlywheelTargetRPM(PRESET_SHORT_RPM);
        }
        prevX2 = x2;

        boolean y2 = gamepad2.y;
        if (y2 &&!prevY2) {
            flywheelController.turnFlywheelOn();
            flywheelController.setFlywheelTargetRPM(PRESET_MED_RPM);
        }
        prevY2 = y2;

        boolean b2 = gamepad2.b;
        if (b2 && !prevB2){
            flywheelController.turnFlywheelOn();
            flywheelController.setFlywheelTargetRPM(PRESET_LONG_RPM);
        }
        prevB2 = b2;

        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2) {
            flywheelController.turnFlywheelOn();
            flywheelController.setFlywheelTargetRPM(PRESET_SHORT_MED_RPM);
        }
        prevLB2 = lb2;

        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2) {
            flywheelController.turnFlywheelOn();
            flywheelController.setFlywheelTargetRPM(PRESET_MED_LONG_RPM);
        }
        prevRB2 = rb2;

        boolean a2 = gamepad2.a;
        if (a2 && !prevA2) {
            flywheelController.turnFlywheelOff();
        }
        prevA2 = a2;
    }

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
}

