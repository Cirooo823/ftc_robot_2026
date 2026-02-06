package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// ===== Pedro Pathing imports =====
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

@TeleOp(name="BLUE teleWithPedro", group="TeleOp")
public class teleWithPedroBlue extends OpMode {

    // ===================== PEDRO DRIVE =====================
    private Follower follower;

    // Fallback start pose if Auto didn't set PoseStorage.lastPose
    private static final Pose TELEOP_FALLBACK_START_POSE = new Pose(135, 9, Math.PI);

    // Goals (your provided coordinates)
    private static final double BLUE_GOAL_X = 8.0;
    private static final double BLUE_GOAL_Y = 136.0;
    private static final double RED_GOAL_X  = 138.0;
    private static final double RED_GOAL_Y  = 136.0;

    // ===================== PLACEHOLDERS YOU WILL FILL =====================
    // LB1 target shooting position (X,Y). Heading is computed automatically to aim at BLUE goal.
    private static final double SHOOT_POS_LB_X = 84.0;
    private static final double SHOOT_POS_LB_Y = 12.0;

    // RB1 target shooting position (X,Y). Heading computed to aim at RED goal.
    private static final double SHOOT_POS_RB_X = 60.0;
    private static final double SHOOT_POS_RB_Y = 12.0;
    // =======================================================

    // Shooter offset relative to robot center (inches)
    // Robot frame definition: +forward along heading, +right to robot’s right.
    private static final double SHOOTER_OFFSET_RIGHT_IN   = 0.0;
    private static final double SHOOTER_OFFSET_FORWARD_IN = -6.0;

    // ======= TUNE ME: heading hold after arriving =======
    private static final double AIM_TOL_DEG   = 1.5;
    private static final double AIM_KP        = 1.5;
    private static final double AIM_MAX_TURN  = 0.8;
    private static final double MICRO_ADJUST_GAIN = 0.35;

    // Driver scaling (hold right trigger = slow)
    private double driverScale = 1.0;

    // Button edges
    private boolean prevLB1 = false;
    private boolean prevRB1 = false;

    // Lazy path suppliers (rebuilt at press time so they start from CURRENT pose)
    private Supplier<PathChain> toShootLB;
    private Supplier<PathChain> toShootRB;

    // Drive state machine
    private enum DriveState {
        MANUAL,
        AUTO_TO_LB,
        AUTO_TO_RB,
        AIM_LOCK_BLUE, // after arriving from LB path
        AIM_LOCK_RED   // after arriving from RB path
    }
    private DriveState driveState = DriveState.MANUAL;

    // Used for telemetry
    private boolean usedAutoStartPose = false;
    private Pose teleopStartPoseUsed = TELEOP_FALLBACK_START_POSE;

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
        follower = Constants.createFollower(hardwareMap);

        // === Start pose: use Auto’s last pose if present, otherwise fallback ===
        Pose startPose = TELEOP_FALLBACK_START_POSE;
        if (PoseStorage.lastPose != null && isPoseValid(PoseStorage.lastPose)) {
            startPose = PoseStorage.lastPose;
            usedAutoStartPose = true;
        }
        teleopStartPoseUsed = startPose;

        follower.setStartingPose(startPose);
        follower.update(); // prime

        // Build suppliers lazily. NOTE: they compute headings at press time based on goal.
        toShootLB = () -> buildGoToShootAndAimPath(
                SHOOT_POS_LB_X, SHOOT_POS_LB_Y,
                BLUE_GOAL_X, BLUE_GOAL_Y
        );

        toShootRB = () -> buildGoToShootAndAimPath(
                SHOOT_POS_RB_X, SHOOT_POS_RB_Y,
                RED_GOAL_X, RED_GOAL_Y
        );

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

        telemetry.addLine("teleWithPedro init complete (AUTO TO SHOOT POS: LB->posA aim BLUE, RB->posB aim RED).");
        telemetry.addData("StartPoseSource", usedAutoStartPose ? "AUTO (PoseStorage)" : "FALLBACK");
        telemetry.addData("StartPose", teleopStartPoseUsed);
        telemetry.addData("LB shoot pos", "(%.1f, %.1f)", SHOOT_POS_LB_X, SHOOT_POS_LB_Y);
        telemetry.addData("RB shoot pos", "(%.1f, %.1f)", SHOOT_POS_RB_X, SHOOT_POS_RB_Y);
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        handleAutoToggles();
        runDriveStateMachine();

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
        telemetry.addData("StartPoseSource", usedAutoStartPose ? "AUTO" : "FALLBACK");

        telemetry.addLine();
        telemetry.addData("--- Flywheel ---", "");
        telemetry.addData("Flywheel State", flywheelController.isFlywheelOn() ? "ACTIVE" : "OFF");
        telemetry.addData("Target RPM", "%.0f", flywheelController.getTargetRPM());
        telemetry.addData("Actual RPM Left", "%.0f", flywheelController.getCurrentRPM_Left());
        telemetry.addData("Actual RPM Right", "%.0f", flywheelController.getCurrentRPM_Right());
        telemetry.addData("Battery Voltage", "%.2f", flywheelController.getBatteryVoltage());

        telemetry.addLine();
        telemetry.addData("--- Barrier ---", "");
        telemetry.addData("Barrier", barrierOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Barrier Pos", "%.3f", barrierOpen ? BARRIER_OPEN_POS : BARRIER_CLOSED_POS);

        if (driveState == DriveState.AIM_LOCK_BLUE || driveState == DriveState.AIM_LOCK_RED) {
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
    // =============== AUTO TOGGLES (LB/RB) =====================
    // =========================================================
    private void handleAutoToggles() {
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        boolean lbPressed = lb && !prevLB1;
        boolean rbPressed = rb && !prevRB1;

        prevLB1 = lb;
        prevRB1 = rb;

        // LB toggles "go to LB shoot position then aim BLUE"
        if (lbPressed) {
            if (driveState == DriveState.AUTO_TO_LB || driveState == DriveState.AIM_LOCK_BLUE) {
                cancelAutoToManual();
            } else {
                startAutoToLB();
            }
        }

        // RB toggles "go to RB shoot position then aim RED"
        if (rbPressed) {
            if (driveState == DriveState.AUTO_TO_RB || driveState == DriveState.AIM_LOCK_RED) {
                cancelAutoToManual();
            } else {
                startAutoToRB();
            }
        }
    }

    private void startAutoToLB() {
        follower.followPath(toShootLB.get());
        driveState = DriveState.AUTO_TO_LB;
    }

    private void startAutoToRB() {
        follower.followPath(toShootRB.get());
        driveState = DriveState.AUTO_TO_RB;
    }

    private void cancelAutoToManual() {
        follower.startTeleopDrive();
        driveState = DriveState.MANUAL;
    }

    // =========================================================
    // =================== DRIVE STATE MACHINE =================
    // =========================================================
    private void runDriveStateMachine() {
        // Your right-trigger slow
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;

        switch (driveState) {

            case MANUAL: {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * driverScale,
                        -gamepad1.left_stick_x * driverScale,
                        -gamepad1.right_stick_x * driverScale,
                        true
                );
                break;
            }

            case AUTO_TO_LB:
            case AUTO_TO_RB: {
                // While busy, follower owns outputs. When done, enter aim-lock.
                if (!follower.isBusy()) {
                    if (driveState == DriveState.AUTO_TO_LB) driveState = DriveState.AIM_LOCK_BLUE;
                    else driveState = DriveState.AIM_LOCK_RED;

                    follower.startTeleopDrive(); // hand back control but we will override rotation in aim-lock
                }
                break;
            }

            case AIM_LOCK_BLUE:
            case AIM_LOCK_RED: {
                Pose p = follower.getPose();
                double goalX = (driveState == DriveState.AIM_LOCK_BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
                double goalY = (driveState == DriveState.AIM_LOCK_BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;

                // Driver translation allowed
                double forward = -gamepad1.left_stick_y * driverScale;
                double strafe  = -gamepad1.left_stick_x * driverScale;

                // Desired heading aims at goal from CURRENT pose (lets you micro-adjust position and stay aimed)
                double desired = computeAimHeadingRad(p, goalX, goalY);

                // Micro-adjust (trim) with right stick
                double trim = gamepad1.right_stick_x * MICRO_ADJUST_GAIN;
                desired = wrapAngle(desired + trim);

                double error = wrapAngle(desired - p.getHeading());
                double autoTurn = AIM_KP * error;
                autoTurn = clamp(autoTurn, -AIM_MAX_TURN, AIM_MAX_TURN);

                if (Math.abs(Math.toDegrees(error)) <= AIM_TOL_DEG && Math.abs(gamepad1.right_stick_x) < 0.05) {
                    autoTurn = 0.0;
                }

                follower.setTeleOpDrive(forward, strafe, autoTurn, true);
                break;
            }
        }
    }

    // =========================================================
    // ============ BUILD PATH: GO TO (x,y) + AIM ===============
    // =========================================================
    private PathChain buildGoToShootAndAimPath(double targetX, double targetY, double goalX, double goalY) {
        // Compute end heading at the target position that aims to the goal
        double endHeading = computeAimHeadingAtXY(targetX, targetY, goalX, goalY);
        Pose endPose = new Pose(targetX, targetY, endHeading);

        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, endPose)))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, endHeading, 0.8)
                )
                .build();
    }

    // =========================================================
    // ======================== AIM MATH =======================
    // =========================================================
    private double computeAimHeadingRad(Pose robotPose, double goalX, double goalY) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double h = robotPose.getHeading();

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

    private double computeAimHeadingAtXY(double x, double y, double goalX, double goalY) {
        // Start with center-aim guess
        double heading = Math.atan2(goalY - y, goalX - x);

        // Iterate to include shooter offset dependence on heading
        for (int i = 0; i < 3; i++) {
            heading = computeAimHeadingRad(new Pose(x, y, heading), goalX, goalY);
        }
        return wrapAngle(heading);
    }

    private static double wrapAngle(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static boolean isPoseValid(Pose p) {
        if (p == null) return false;
        return Double.isFinite(p.getX()) && Double.isFinite(p.getY()) && Double.isFinite(p.getHeading());
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
