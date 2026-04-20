package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.NonEnslavedVoltageFlywheelController;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage; // Import your bridge file
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name="Tele with Odometry", group="TeleOp")
public class telewithodometry extends OpMode {
    private NonEnslavedVoltageFlywheelController flywheelController;
    private Follower follower;

    private DcMotorEx right_b, left_f, right_f, left_b;
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor intake;

    private Servo barrierServo;
    private static final double BARRIER_CLOSED_POS = 0.58;
    private static final double BARRIER_OPEN_POS = 0;
    private boolean barrierOpen = false;

    private double driverScale = 1.0;
    private boolean intakeOn = false;

    // Edge detection
    private boolean prevRSB1 = false;
    private boolean prevX2 = false, prevB2 = false, prevLSB2 = false, prevlb2 = false, prevrb2 = false;
    private boolean prevDU = false, prevDL = false, prevDR = false;

    // RPM Constants
    private static final int PRESET_CLOSE = 2500;
    private static final int PRESET_FAR = 2900;
    private static final int RPM_STEP = 50;
    private static final int MIN_RPM = 500;

    // Rumble / State
    private static final double RPM_TOLERANCE = 50.0;
    private boolean flywheelAtSpeed = false;
    private boolean flywheelReadyRumbled = false;
    private double lastTargetRPM = 0.0;

    @Override
    public void init() {
        // ---- Initialize Pedro Follower ----
        follower = new Follower(hardwareMap);
        follower.setStartingPose(PoseStorage.lastPose); // Pull from Auto sticky note

        // ---- Drive Train ----
        left_f = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

        left_f.setDirection(DcMotor.Direction.REVERSE);
        left_b.setDirection(DcMotor.Direction.FORWARD);
        right_f.setDirection(DcMotor.Direction.REVERSE);
        right_b.setDirection(DcMotor.Direction.FORWARD);

        // Drive Zero Power = BRAKE for precision
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
        flywheel_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel_Right.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelController = new NonEnslavedVoltageFlywheelController(hardwareMap);

        // ---- INTAKE (Float Mode to reduce heat) ----
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        barrierServo = hardwareMap.get(Servo.class, "barrierServo");
        barrierServo.setPosition(BARRIER_CLOSED_POS);

        telemetry.addLine("Ready: Odometry linked to Auto.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update(); // Keep the coordinate map updated

        drive();
        runFlywheel();
        runIntake();
        runBarrier();

        flywheelController.update();
        updateFlywheelReadyRumble();

        // TELEMETRY
        Pose p = follower.getPose();
        telemetry.addData("Position", "X:%.1f Y:%.1f", p.getX(), p.getY());
        telemetry.addData("Dist to Goal", "%.1f in", getDistToGoal());
        telemetry.addData("Target RPM", "%.0f", flywheelController.getTargetRPM());
        telemetry.addData("Actual RPM", "%.0f", flywheelController.getCurrentRPM_Average());
        telemetry.update();
    }

    private double getDistToGoal() {
        Pose p = follower.getPose();
        double dx = PoseStorage.goalX - p.getX();
        double dy = PoseStorage.goalY - p.getY();
        return Math.sqrt(dx*dx + dy*dy);
    }

    private double calculateAutoRPM(double distance) {
        // FORMULA: (Slope * distance) + Intercept
        // TUNE THESE: 12.5 is slope, 2000 is base RPM
        double rpm = (12.5 * distance) + 2000.0;
        return Math.max(MIN_RPM, Math.min(rpm, 3500));
    }

    private void runFlywheel() {
        // A Button = Auto Aim RPM
        if (gamepad2.a) {
            double dist = getDistToGoal();
            flywheelController.setFlywheelTargetRPM(calculateAutoRPM(dist));
            flywheelController.turnFlywheelOn();
        }

        // Presets
        if (gamepad2.left_bumper && !prevlb2) setFlywheelPreset(PRESET_CLOSE);
        if (gamepad2.right_bumper && !prevrb2) setFlywheelPreset(PRESET_FAR);
        if (gamepad2.left_stick_button && !prevLSB2) flywheelController.turnFlywheelOff();

        // Manual Tuning (+/- 50 RPM)
        if (gamepad2.b && !prevB2 && flywheelController.isFlywheelOn()) {
            flywheelController.setFlywheelTargetRPM(flywheelController.getTargetRPM() + RPM_STEP);
        }
        if (gamepad2.x && !prevX2 && flywheelController.isFlywheelOn()) {
            double newTarget = Math.max(flywheelController.getTargetRPM() - RPM_STEP, MIN_RPM);
            flywheelController.setFlywheelTargetRPM(newTarget);
        }

        prevX2 = gamepad2.x; prevB2 = gamepad2.b; prevlb2 = gamepad2.left_bumper;
        prevrb2 = gamepad2.right_bumper; prevLSB2 = gamepad2.left_stick_button;
    }

    private void runIntake() {
        // G1 B = Manual Reverse
        if (gamepad1.b) {
            intake.setPower(-1.0);
        }
        // G1 Left Trigger = 100% Power Forward
        else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(1.0);
        }
        else {
            intake.setPower(0.0);
        }
    }

    private void setFlywheelPreset(int rpm) {
        flywheelController.setFlywheelTargetRPM(rpm);
        flywheelController.turnFlywheelOn();
    }

    private void updateFlywheelReadyRumble() {
        double target = flywheelController.getTargetRPM();
        if (!flywheelController.isFlywheelOn() || target <= 0.0) {
            flywheelAtSpeed = false; flywheelReadyRumbled = false;
            return;
        }
        if (target != lastTargetRPM) {
            flywheelAtSpeed = false; flywheelReadyRumbled = false;
        }
        boolean atSpeedNow = Math.abs(target - flywheelController.getCurrentRPM_Average()) <= RPM_TOLERANCE;
        if (atSpeedNow && !flywheelAtSpeed && !flywheelReadyRumbled) {
            gamepad2.rumbleBlips(1);
            flywheelReadyRumbled = true;
        }
        flywheelAtSpeed = atSpeedNow;
        lastTargetRPM = target;
    }

    private void drive() {
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double ys = y * driverScale;
        double xs = x * driverScale;
        double rxs = rx * driverScale;
        double d = Math.max(Math.abs(ys) + Math.abs(xs) + Math.abs(rxs), 1.0);
        right_f.setPower(( ys - xs - rxs) / d);
        left_b.setPower( ( ys - xs + rxs) / d);
        left_f.setPower( ( ys + xs + rxs) / d);
        right_b.setPower(( ys + xs - rxs) / d);
    }

    private void runBarrier() {
        if (gamepad2.dpad_left && !prevDL) { barrierServo.setPosition(BARRIER_CLOSED_POS); barrierOpen = false; }
        if (gamepad2.dpad_right && !prevDR) { barrierServo.setPosition(BARRIER_OPEN_POS); barrierOpen = true; }
        if (gamepad2.dpad_up && !prevDU) {
            barrierOpen = !barrierOpen;
            barrierServo.setPosition(barrierOpen ? BARRIER_OPEN_POS : BARRIER_CLOSED_POS);
        }
        prevDL = gamepad2.dpad_left; prevDR = gamepad2.dpad_right; prevDU = gamepad2.dpad_up;
    }

    @Override
    public void stop() {
        flywheelController.turnFlywheelOff();
        intake.setPower(0);
    }
}