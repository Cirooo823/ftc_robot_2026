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
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.HeadingInterpolator;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="teleWithPedro", group="TeleOp")
public class teleWithPedro extends OpMode {

    // ===================== PEDRO DRIVE =====================
    private Follower follower;

    // You said: start at (135, 9) heading pi (Pedro field coordinates; 0,0 bottom-left red side)
    private static final Pose TELEOP_START_POSE = new Pose(135, 9, Math.PI);

    // Goals (your provided coordinates)
    private static final double BLUE_GOAL_X = 6.0;
    private static final double BLUE_GOAL_Y = 136.0;
    private static final double RED_GOAL_X  = 138.0;
    private static final double RED_GOAL_Y  = 136.0;

    // Staging/shot position you want to drive to in TeleOp
    private static final Pose STAGING_POSE = new Pose(72.0, 26.0, 0.0); // heading will be computed

    // Shooter offset relative to robot center.
    // You defined shooter at (0, -6) assuming robot "forward" is +Y in your mental model.
    // We'll interpret this as: (right, forward) axes in robot frame:
    //   dxRight = 0, dyForward = -6  (6 inches behind center)
    private static final double SHOOTER_OFFSET_RIGHT_IN  = 0.0;
    private static final double SHOOTER_OFFSET_FORWARD_IN = -6.0;

    // ======= TUNE ME: aim lock behavior =======
    // How close is "good enough" to finish aiming (degrees)
    private static final double AIM_TOL_DEG = 1.5;

    // Heading-hold proportional gain (bigger = snaps harder, too big = oscillation)
    private static final double AIM_KP = 3.0;

    // Clamp for auto rotation command (0..1). 1.0 = full speed spin.
    private static final double AIM_MAX_TURN = 1.0;

    // How much right stick contributes as a "trim" while aim-lock is active
    // (this does NOT break the lock; it adds a small offset so driver can micro-adjust)
    private static final double MICRO_ADJUST_GAIN = 0.35;

    // Driver scaling (hold right trigger = slow)
    private double driverScale = 1.0;

    // Optional toggle slowMode (RB) stacks with RT slow
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private boolean prevRB1 = false;

    // Button edges for toggles
    private boolean prevLB1 = false;
    private boolean prevRB1Btn = false;

    // Path suppliers (built on demand so the "start pose" is current)
    private Supplier<PathChain> toStagingBlue;
    private Supplier<PathChain> toStagingRed;

    // Drive state machine
    private enum DriveState {
        MANUAL,
        AUTO_TO_BLUE,   // following path to staging pose and ending at blue-aim heading
        AUTO_TO_RED,    // same but red
        AIM_LOCK_BLUE,  // driver can translate; heading held toward blue goal
        AIM_LOCK_RED
    }

    private DriveState driveState = DriveState.MANUAL;

    // "Aim setpoint" used in AIM_LOCK_* states (radians)
    private double aimHeadingSetpointRad = 0.0;

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
    private static final int PRESET_CLOSE = 3000; // gamepad2 left trigger
    private static final int PRESET_FAR   = 2850; // gamepad2 right trigger

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
    private static final double INTAKE_RPM_BARRIER_CLOSED = 1150.0;
    private static final double INTAKE_RPM_BARRIER_FAR  = 250.0;
    private static final double INTAKE_RPM_BARRIER_CLOSE = 200.0;
    private static final double INTAKE_RPM_BARRIER_OPEN_DEFAULT = 250.0;
    private static final double INTAKE_RPM_PRESET_G1_RSB = 230.0;

    @Override
    public void init() {
        // ===================== PEDRO INIT =====================
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(TELEOP_START_POSE);
        follower.update(); // prime

        // Build path suppliers lazily so they use current pose when triggered
        toStagingBlue = () -> buildPathToStagingAndAim(BLUE_GOAL_X, BLUE_GOAL_Y);
        toStagingRed  = () -> buildPathToStagingAndAim(RED_GOAL_X,  RED_GOAL_Y);

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

        telemetry.addLine("teleWithPedro init complete (AUTO to staging + aim-lock toggles enabled).");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Pedro requirement: update every loop
        follower.update();

        // Handle toggles for LB/RB automated routine
        handleAutoToggles();

        // Drive behavior based on state
        runPedroDriveStateMachine();

        // Mechanisms
        runFlywheel();
        runIntake();
        runBarrier();

        flywheelController.update();
        updateFlywheelReadyRumble();

        // Telemetry
        telemetry.addData("--- Pedro Pose ---", "");
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Vel", follower.getVelocity());
        telemetry.addData("DriveState", driveState);

        telemetry.addData("AimSetpoint(deg)", "%.1f", Math.toDegrees(aimHeadingSetpointRad));

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
        telemetry.addData("Barrier Ctrl", "g2 dpad_left=CLOSE, dpad_right=OPEN, dpad_up=TOGGLE");

        telemetry.update();
    }

    // =========================================================
    // =============== AUTO TOGGLES (LB/RB) =====================
    // =========================================================
    private void handleAutoToggles() {
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        boolean lbPressed = lb && !prevLB1;
        boolean rbPressed = rb && !prevRB1Btn;

        prevLB1 = lb;
        prevRB1Btn = rb;

        // LB toggles BLUE routine
        if (lbPressed) {
            if (driveState == DriveState.AUTO_TO_BLUE || driveState == DriveState.AIM_LOCK_BLUE) {
                // Toggle off
                cancelAutoToManual();
            } else {
                // Start BLUE routine (if currently red, this switches)
                startAutoToGoal(true);
            }
        }

        // RB toggles RED routine
        if (rbPressed) {
            if (driveState == DriveState.AUTO_TO_RED || driveState == DriveState.AIM_LOCK_RED) {
                cancelAutoToManual();
            } else {
                startAutoToGoal(false);
            }
        }
    }

    private void startAutoToGoal(boolean blue) {
        // Start path-following to staging pose, with heading interpolation to aim at goal.
        follower.followPath(blue ? toStagingBlue.get() : toStagingRed.get());
        driveState = blue ? DriveState.AUTO_TO_BLUE : DriveState.AUTO_TO_RED;
    }

    private void cancelAutoToManual() {
        follower.startTeleopDrive();
        driveState = DriveState.MANUAL;
    }

    // =========================================================
    // =================== DRIVE STATE MACHINE =================
    // =========================================================
    private void runPedroDriveStateMachine() {
        // Driver slow controls (kept from your original)
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;

        boolean rb1 = gamepad1.right_bumper; // NOTE: RB is used for auto toggle as well, so we won't use this for slowMode toggle now.
        // If you still want RB slowMode, move slowMode toggle to a different button.
        // Keeping slowMode off by default to avoid conflicts:
        slowMode = false;

        double extraScale = slowMode ? slowModeMultiplier : 1.0;
        double scale = driverScale * extraScale;

        switch (driveState) {
            case MANUAL: {
                // Manual robot-centric teleop drive
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * scale,
                        -gamepad1.left_stick_x * scale,
                        -gamepad1.right_stick_x * scale,
                        true
                );
                break;
            }

            case AUTO_TO_BLUE:
            case AUTO_TO_RED: {
                // While follower is busy, it owns drive outputs (we do NOT send teleop drive commands).
                // When done, switch into AIM_LOCK.
                if (!follower.isBusy()) {
                    boolean blue = (driveState == DriveState.AUTO_TO_BLUE);
                    driveState = blue ? DriveState.AIM_LOCK_BLUE : DriveState.AIM_LOCK_RED;

                    // Set aim setpoint based on CURRENT pose at arrival
                    Pose p = follower.getPose();
                    if (blue) aimHeadingSetpointRad = computeAimHeadingRad(p, BLUE_GOAL_X, BLUE_GOAL_Y);
                    else      aimHeadingSetpointRad = computeAimHeadingRad(p, RED_GOAL_X,  RED_GOAL_Y);

                    // Return to teleop drive so we can do heading hold + driver translation
                    follower.startTeleopDrive();
                }
                break;
            }

            case AIM_LOCK_BLUE:
            case AIM_LOCK_RED: {
                boolean blue = (driveState == DriveState.AIM_LOCK_BLUE);
                double goalX = blue ? BLUE_GOAL_X : RED_GOAL_X;
                double goalY = blue ? BLUE_GOAL_Y : RED_GOAL_Y;

                // Continuously recompute desired heading (so if driver translates, it still points at goal)
                Pose p = follower.getPose();
                double desired = computeAimHeadingRad(p, goalX, goalY);

                // Allow micro-adjust: driver right stick adds a small offset to desired heading
                double trim = gamepad1.right_stick_x * MICRO_ADJUST_GAIN;
                desired = wrapAngle(desired + trim);

                // Heading error -> turn command
                double error = wrapAngle(desired - p.getHeading());
                double turnCmd = AIM_KP * error;

                // Clamp to max turn power
                turnCmd = clamp(turnCmd, -AIM_MAX_TURN, AIM_MAX_TURN);

                // If we're basically on target, you can optionally zero turnCmd
                if (Math.abs(Math.toDegrees(error)) <= AIM_TOL_DEG && Math.abs(gamepad1.right_stick_x) < 0.05) {
                    turnCmd = 0.0;
                }

                // Driver translation still allowed:
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * scale,
                        gamepad1.left_stick_x * scale,
                        turnCmd,
                        true
                );

                aimHeadingSetpointRad = desired;
                break;
            }
        }
    }

    // =========================================================
    // ============ PATH BUILDING TO STAGING + AIM ==============
    // =========================================================
    private PathChain buildPathToStagingAndAim(double goalX, double goalY) {
        // Compute end heading at the staging pose that aims toward the goal
        double endHeading = computeAimHeadingAtXY(STAGING_POSE.getX(), STAGING_POSE.getY(), goalX, goalY);

        Pose endPose = new Pose(STAGING_POSE.getX(), STAGING_POSE.getY(), endHeading);

        // Matches Pedro ExampleTeleOp style: Path(new BezierLine(follower::getPose, new Pose(...)))
        // and HeadingInterpolator.linearFromPoint(follower::getHeading, targetHeading, 0.8). :contentReference[oaicite:2]{index=2}
        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, endPose)))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(follower::getHeading, endHeading, 0.8)
                )
                .build();
    }

    // =========================================================
    // ===================== AIM MATH ==========================
    // =========================================================

    private double computeAimHeadingRad(Pose robotPose, double goalX, double goalY) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double h = robotPose.getHeading();

        // Convert shooter offset from (right, forward) robot frame into world offset.
        // Standard Pose heading is measured from +X axis (east) CCW.
        // Robot frame here:
        //   +forward along heading
        //   +right is heading - 90deg
        double dxRight = SHOOTER_OFFSET_RIGHT_IN;
        double dyForward = SHOOTER_OFFSET_FORWARD_IN;

        double cosH = Math.cos(h);
        double sinH = Math.sin(h);

        // forward component projects along heading
        double worldDxFromForward = dyForward * cosH;
        double worldDyFromForward = dyForward * sinH;

        // right component projects along heading - 90deg: (cos(h-90), sin(h-90)) = (sinH, -cosH)
        double worldDxFromRight = dxRight * sinH;
        double worldDyFromRight = -dxRight * cosH;

        double shooterX = robotX + worldDxFromForward + worldDxFromRight;
        double shooterY = robotY + worldDyFromForward + worldDyFromRight;

        double vx = goalX - shooterX;
        double vy = goalY - shooterY;

        return wrapAngle(Math.atan2(vy, vx));
    }

    /**
     * Computes aim heading for a desired end position (x,y) before we know heading.
     * Uses a few iterations: start with center-aim, then refine using shooter offset.
     */
    private double computeAimHeadingAtXY(double x, double y, double goalX, double goalY) {
        double heading = Math.atan2(goalY - y, goalX - x);

        // Iterate to account for shooter offset depending on heading
        for (int i = 0; i < 3; i++) {
            Pose p = new Pose(x, y, heading);
            heading = computeAimHeadingRad(p, goalX, goalY);
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
            targetRpm = -INTAKE_RPM_BARRIER_CLOSED;
        } else if (intakeOn) {
            if (intakePreset250Enabled) {
                targetRpm = INTAKE_RPM_PRESET_G1_RSB;
            } else {
                if (!barrierOpen) {
                    targetRpm = INTAKE_RPM_BARRIER_CLOSED;
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
