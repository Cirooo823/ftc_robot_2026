package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="TestingNewIntake", group="TeleOp")
public class testingNewIntake extends OpMode {
    private DcMotorEx right_b, left_f, right_f, left_b;

    // ===== MECHANISMS =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor intake;
    //   private CRServo   thirdStage;

    private double driverScale = 1.0;


    private boolean intakeOn   = false;

    // Edge detection (g1 intake toggles)
    private boolean prevA = false, prevX = false;
    private boolean prevDU = false, prevDD = false, prevDL = false, prevDR = false;

    // gamepad2 button edges
    private boolean prevA2  = false;
    private boolean prevX2  = false;
    private boolean prevY2  = false;
    private boolean prevB2  = false;
    private boolean prevLB2 = false;
    private boolean prevRB2 = false;
    private boolean prevLT2 = false;
    private boolean prevRT2 = false;
    private boolean prevLSB2 = false;

    private int thirdDir = 0;

    // ===== PRESET RPMs =====
    private static final int PRESET_STALL          = 500;  // A -> low stall preset
    private static final int PRESET_SHORT_MED_RPM  = 2900; // gamepad2 left trigger
    private static final int PRESET_MED_RPM        = 3400; // gamepad2 Y
    private static final int PRESET_MED_LONG_RPM   = 3250; // gamepad2 right bumper
    private static final int PRESET_LONG_RPM       = 3250; // gamepad2 right trigger

    // Fine adjust step and minimum
    private static final int RPM_STEP = 50;
    private static final int MIN_RPM  = 500;

    // ===== FLYWHEEL READY-TO-SHOOT RUMBLE (gamepad2) =====
    private static final double RPM_TOLERANCE = 50.0;
    private boolean flywheelAtSpeed       = false;
    private boolean flywheelReadyRumbled  = false;
    private double  lastTargetRPM         = 0.0;

    // ===== VISION =====
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private boolean           visionConfigured = false;

    // Last seen tag info (for telemetry continuity)
    private int    lastTagId     = -1;
    private double lastTagRange  = 0.0;
    private double lastYaw       = 0.0;
    private double lastBearing   = 0.0;

    // For current frame
    private boolean tagVisible       = false;
    private double  currentBearing   = 0.0;

    // ===== AIM RUMBLE (gamepad1) WHEN ALIGNED TO TAG =====
    // Bearing is how many degrees we must turn to point at the tag; 0 = directly at tag.
    private static final double AIM_BEARING_TOL_DEG = 1.5;  // tune this tolerance
    private boolean aimRumbleActive = false;

    // ===== DRIVE LOCK (ANTI-PUSH) =====
    // When gamepad1 left trigger is held, lock drive motors to current encoder positions
    private boolean driveLockActive = false;
    private static final double DRIVE_LOCK_HOLD_POWER = 0.30;  // tune: higher = stronger hold, more current

    private int lfLockPos, rfLockPos, lbLockPos, rbLockPos;


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

        intake         = hardwareMap.get(DcMotor.class,   "intake");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        drive();
        //     runFlywheel();
        //     runThirdStage();
        runIntake();

    }
    private void drive() {
        // Left trigger on gamepad1 engages drive lock

        // Normal mecanum drive
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

        double intakePower = 0.0;

        if (b) {
            intakePower = -1.0;  // hard reverse
        } else if (intakeOn) {
            // If flywheel is on, limit current draw
            //       if (flywheelController.isFlywheelOn()) {
            intakePower = 0.5;   // softer intake while shooting
        } else {
            intakePower = 1.0;   // full speed when just intaking
        }
        intake.setPower(intakePower);
    }

    @Override
    public void stop() {
        right_f.setPower(0);
        left_f.setPower(0);
        right_b.setPower(0);
        left_b.setPower(0);

        //     flywheel_Left.setVelocity(0);
        //      flywheel_Right.setVelocity(0);
        intake.setPower(0);
        //       thirdStage.setPower(0.0);
    }
}
