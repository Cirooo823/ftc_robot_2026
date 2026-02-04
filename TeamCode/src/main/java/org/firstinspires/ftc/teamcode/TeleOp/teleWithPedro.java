package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// ===== Pedro Pathing imports =====
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="teleWithPedro", group="TeleOp")
public class teleWithPedro extends OpMode {

    // ===================== PEDRO DRIVE =====================
    private Follower follower;

    // You said: start at (135, 9) heading pi (0,0 bottom-left red side)
    private static final Pose TELEOP_START_POSE = new Pose(135, 9, Math.PI);

    // Goals (your provided coordinates)
    private static final double BLUE_GOAL_X = 6.0;
    private static final double BLUE_GOAL_Y = 136.0;
    private static final double RED_GOAL_X  = 138.0;
    private static final double RED_GOAL_Y  = 136.0;

    // Shooter offset relative to robot center (inches)
    // You defined shooter at (0, -6) in robot frame where +forward is "north".
    // Here we interpret robot frame as: +forward along robot heading, +right to robot’s right.
    private static final double SHOOTER_OFFSET_RIGHT_IN   = 0.0;
    private static final double SHOOTER_OFFSET_FORWARD_IN = -6.0;

    // ======= TUNE ME: aim lock behavior =======
    private static final double AIM_TOL_DEG   = 1.5;   // "good enough" tolerance
    private static final double AIM_KP        = 3.0;   // proportional gain on heading error
    private static final double AIM_MAX_TURN  = 1.0;   // clamp turn command magnitude (0..1)
    private static final double MICRO_ADJUST_GAIN = 0.35; // right stick adds small trim while locked

    // Driver scaling (hold right trigger = slow)
    private double driverScale = 1.0;

    // Button edges
    private boolean prevLB1 = false;
    private boolean prevRB1 = false;

    // Drive state machine (no pathing now)
    private enum DriveState {
        MANUAL,
        AIM_LOCK_BLUE,
        AIM_LOCK_RED
    }
    private DriveState driveState = DriveState.MANUAL;

    // ===================== MECHANISMS =====================
    private VoltageFlywheelController flywheelController;
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotorEx intake;

    // ===== BARRIER (SERVO) =====
    private Servo barrierServo;
    private static final double BARRIER_CLOSED_POS = 0.6;
    private static final double BARRIER_OPEN_POS   = 0.0;
    private boolean barrierOpen = false;

    // ===== STATE =====
    private boolean intakeOn = false;
    private boolean prevX = false;

    // gamepad1 right stick button edge + preset state
    private boolean prevRSB1 = false;
    private boolean intakePreset250Enabled = false;

    // gamepad2 button edges
    private boolean prevX2  = false;
    private boolean prevB2  = false;
    private boolean prevLT2 = false;
    private boolean prevRT2 = false;
    private boolean prevLSB2 = false;

    // Barrier edges (g2)
    private boolean prevDU = false, prevDL = false, prevDR = false;

    // ===== FLYWHEEL PRESET RPMs =====
    private static final int PRESET_CLOSE = 3000;
    private static final int PRESET_FAR   = 2850;

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

    private static final double INTAKE_RPM_BARRIER_CLOSED_POSITION = 1150.0;

    private static final double INTAKE_RPM_BARRIER_FAR  = 230.0;
    private static final double INTAKE_RPM_BARRIER_CLOSE = 400.0;
    private static final double INTAKE_RPM_BARRIER_OPEN_DEFAULT = 400.0;

    private static final double INTAKE_RPM_PRESET_G1_RSB = 230.0;

    @Override
    public void init() {
        // ===================== PEDRO INIT =====================
        follower = Constants.createFollower(hardwareMap);

        // This sets the pose estimate Pedro starts with in TeleOp.
        // Pedro will then integrate Pinpoint odometry from your Constants.localizerConstants.
        follower.setStartingPose(TELEOP_START_POSE);
        follower.update(); // prime

        // ===================== MECHANISMS INIT =====================
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");
        intake         = hardwareMap.get(DcMotorEx.class, "intake");

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel_Right.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelController = new VoltageFlywheelController(hardwareMap);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        barrierServo = hardwareMap.get(Servo.class, "barrierServo");
        barrierServo.setPosition(BARRIER_CLOSED_POS);
        barrierOpen = false;

        telemetry.addLine("teleWithPedro init complete (AIM LOCK ONLY: LB=blue, RB=red).");
        telemetry.update();
    }

    @Override
    public void start() {
        // Puts follower into teleop-drive mode.
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Pedro requirement: update every loop so pose stays fresh
        follower.update();

        // Toggle aim-lock states
        handleAimLockToggles();

        // Drive (manual or aim-lock)
        runDrive();

        // Mechanisms
        runFlywheel();
        runIntake();
        runBarrier();

        flywheelController.update();
        updateFlywheelReadyRumble();

        // Telemetry
        Pose p = follower.getPose();
        telemetry.addData("--- Pedro Pose ---", "");
        telemetry.addData("Pose", p);
        telemetry.addData("Vel", follower.getVelocity());
        telemetry.addData("DriveState", driveState);

        if (driveState != DriveState.MANUAL) {
            double gx = (driveState == DriveState.AIM_LOCK_BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
            double gy = (driveState == DriveState.AIM_LOCK_BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
            double desired = computeAimHeadingRad(p, gx, gy);
            double errDeg = Math.toDegrees(wrapAngle(desired - p.getHeading()));
            telemetry.addData("AimTarget(deg)", "%.1f", Math.toDegrees(desired));
            telemetry.addData("AimError(deg)", "%.1f", errDeg);
        }

        telemetry.update();
    }

    // =========================================================
    // =================== AIM LOCK TOGGLES =====================
    // =========================================================
    private void handleAimLockToggles() {
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        boolean lbPressed = lb && !prevLB1;
        boolean rbPressed = rb && !prevRB1;

        prevLB1 = lb;
        prevRB1 = rb;

        // LB toggles BLUE aim-lock
        if (lbPressed) {
            if (driveState == DriveState.AIM_LOCK_BLUE) driveState = DriveState.MANUAL;
            else driveState = DriveState.AIM_LOCK_BLUE;
        }

        // RB toggles RED aim-lock
        if (rbPressed) {
            if (driveState == DriveState.AIM_LOCK_RED) driveState = DriveState.MANUAL;
            else driveState = DriveState.AIM_LOCK_RED;
        }
    }

    // =========================================================
    // ====================== DRIVE LOGIC ======================
    // =========================================================
    private void runDrive() {
        // Keep your right-trigger slow
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;

        // Driver translation always allowed
        double forward = -gamepad1.left_stick_y * driverScale;
        double strafe  = -gamepad1.left_stick_x * driverScale;

        // Default rotation is driver-controlled
        double rotCmd = -gamepad1.right_stick_x * driverScale;

        if (driveState == DriveState.AIM_LOCK_BLUE || driveState == DriveState.AIM_LOCK_RED) {
            Pose p = follower.getPose();

            double goalX = (driveState == DriveState.AIM_LOCK_BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
            double goalY = (driveState == DriveState.AIM_LOCK_BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;

            // Aim heading computed from CURRENT pose (so wherever you are, you point at the goal)
            double desired = computeAimHeadingRad(p, goalX, goalY);

            // Micro-adjust: right stick adds a small trim (does not “break” lock)
            double trim = gamepad1.right_stick_x * MICRO_ADJUST_GAIN;
            desired = wrapAngle(desired + trim);

            double error = wrapAngle(desired - p.getHeading());
            double autoTurn = AIM_KP * error;
            autoTurn = clamp(autoTurn, -AIM_MAX_TURN, AIM_MAX_TURN);

            // If basically on target AND driver isn't trimming, stop turning
            if (Math.abs(Math.toDegrees(error)) <= AIM_TOL_DEG && Math.abs(gamepad1.right_stick_x) < 0.05) {
                autoTurn = 0.0;
            }

            // Override rotation with aim-lock rotation
            rotCmd = autoTurn;
        }

        // Robot-centric
        follower.setTeleOpDrive(
                forward,
                strafe,
                rotCmd,
                true
        );
    }

    // =========================================================
    // ======================== AIM MATH =======================
    // =========================================================
    private double computeAimHeadingRad(Pose robotPose, double goalX, double goalY) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double h = robotPose.getHeading();

        // Shooter offset from robot frame -> world frame using current heading
        double dxRight = SHOOTER_OFFSET_RIGHT_IN;
        double dyForward = SHOOTER_OFFSET_FORWARD_IN;

        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        // Forward component along heading
        double worldDxFromForward = dyForward * cosH;
        double worldDyFromForward = dyForward * sinH;

        // Right component along (heading - 90deg) => (sinH, -cosH)
        double worldDxFromRight = dxRight * sinH;
        double worldDyFromRight = -dxRight * cosH;

        double shooterX = robotX + worldDxFromForward + worldDxFromRight;
        double shooterY = robotY + worldDyFromForward + worldDyFromRight;

        double vx = goalX - shooterX;
        double vy = goalY - shooterY;

        return wrapAngle(Math.atan2(vy, vx));
    }

    private static double wrapAngle(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ===================== FLYWHEEL =====================
    private void runFlywheel() {
        boolean x2   = gamepad2.x;
        boolean b2   = gamepad2.b;
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

    // ===================== INTAKE (VELOCITY CONTROL) =====================
    private void runIntake() {
        boolean x = gamepad1.x;
        if (x && !prevX) intakeOn = !intakeOn;
        prevX = x;

        boolean rsb1 = gamepad1.right_stick_button;
        if (rsb1 && !prevRSB1) intakePreset250Enabled = !intakePreset250Enabled;
        prevRSB1 = rsb1;

        boolean b = gamepad1.b;
        double targetRpm;

        if (b) {
            targetRpm = -INTAKE_RPM_BARRIER_CLOSED_POSITION;
        } else if (intakeOn) {
            if (intakePreset250Enabled) {
                targetRpm = INTAKE_RPM_PRESET_G1_RSB;
            } else {
                if (!barrierOpen) {
                    targetRpm = INTAKE_RPM_BARRIER_CLOSED_POSITION;
                } else {
                    double flywheelTarget = flywheelController.getTargetRPM();

                    if (Math.abs(flywheelTarget - PRESET_FAR) < 0.5) {
                        targetRpm = INTAKE_RPM_BARRIER_FAR;
                    } else if (Math.abs(flywheelTarget - PRESET_CLOSE) < 0.5) {
                        targetRpm = INTAKE_RPM_BARRIER_CLOSE;
                    } else {
                        targetRpm = INTAKE_RPM_BARRIER_OPEN_DEFAULT;
                    }
                }
            }
        } else {
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
        follower.startTeleopDrive();

        flywheel_Left.setPower(0);
        flywheel_Right.setPower(0);
        intake.setPower(0);

        if (barrierServo != null) {
            barrierServo.setPosition(BARRIER_CLOSED_POS);
            barrierOpen = false;
        }

        if (gamepad1 != null) gamepad1.stopRumble();
        if (gamepad2 != null) gamepad2.stopRumble();
    }
}
