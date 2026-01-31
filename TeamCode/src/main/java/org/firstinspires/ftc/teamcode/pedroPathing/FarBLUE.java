package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "Far BLUE", group = "Autos")
public class FarBLUE extends OpMode {

    // --- 1. CHANGE: Use the Logic Class ---
    private FlywheelLogic shooter;

    private DcMotor intake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // --- POSES (Untouched) ---
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootfarPose = new Pose(57, 19.475409836065573, Math.toRadians(112));
    private final Pose facingartifactsclosePose = new Pose(42.295081967213115, 36, Math.toRadians(180));
    private final Pose pickupclosePose = new Pose(19.672131147540984, 36, Math.toRadians(180));
    private final Pose facingartifactsmiddlePose = new Pose(41.90163934426229, 59.60655737704917, Math.toRadians(180));
    private final Pose pickupmiddlePose = new Pose(19.4098370661501, 59.672127145235656, Math.toRadians(180));
    private final Pose facingartifactsfarPose = new Pose(38.95081967213115, 83.99999999999999, Math.toRadians(180));
    private final Pose pickupfarPose = new Pose(19.882781982421875, 84.19023757684427, Math.toRadians(180));
    private final Pose shootclosePose = new Pose(49.967213114754095, 102.68852459016394, Math.toRadians(137));
    private final Pose leavefarPose = new Pose(37.37704918032787, 90.29508196721311, Math.toRadians(137));
    private final Pose facegatePose = new Pose(37.37704918032787, 90.29508196721311, Math.toRadians(137));

    private PathChain scorepreload;
    private PathChain faceartifactsclose, pickupartifactsclose, shootartifactsclose;
    private PathChain faceartifactsmiddle, pickupartifactsmiddle, shootartifctsmiddle;
    private PathChain faceartifactsfar, pickupartifactsfar, shootartifactsfar;
    private PathChain facegate;

    public void buildPaths() {
        scorepreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootfarPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootfarPose.getHeading())
                .build();

        faceartifactsclose = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(57, 19.475), new Pose(41.902, 33)))
                .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                .build();

        pickupartifactsclose = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.902, 33), new Pose(11, 33)))
                .setTangentHeadingInterpolation()
                .build();

        shootartifactsclose = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(11, 33), new Pose(57, 19.475)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        faceartifactsmiddle = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(57, 19.475), new Pose(41.902, 57)))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        pickupartifactsmiddle = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.902, 57), new Pose(10, 57)))
                .setTangentHeadingInterpolation()
                .build();

        shootartifctsmiddle = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(10, 57), new Pose(57, 19.475)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                .build();

        facegate = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(57.000, 19.475), new Pose(24, 70)))
                .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                .build();
    }

    // --- 2. THE CORRECTED STATE MACHINE ---
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to Shoot Preload
                follower.followPath(scorepreload);
                setPathState(1);
                break;

            case 1: // Wait for arrival -> FIRE
                if (!follower.isBusy()) {
                    // Trigger the shot logic (1 shot)
                    shooter.fireShots(1);
                    setPathState(2);
                }
                break;

            case 2: // Wait for shooter to finish -> Drive to Spike Mark
                if (!shooter.isBusy()) { // Wait until shooter returns to IDLE
                    follower.followPath(faceartifactsclose, true);
                    setPathState(3);
                }
                break;

            case 3: // Arrived at Spike Mark -> Drive to Pickup
                if (!follower.isBusy()) {
                    follower.followPath(pickupartifactsclose, true);
                    setPathState(4);
                }
                break;

            case 4: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                    follower.followPath(shootartifactsclose, true);
                    setPathState(5);
                }
                break;

            case 5: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(6);
                }
                break;

            case 6: // Wait for shooter -> Drive to Middle Spike
                if (!shooter.isBusy()) {
                    follower.followPath(faceartifactsmiddle, true);
                    setPathState(7);
                }
                break;

            case 7: // Arrived at Middle Spike -> Drive to Pickup
                if (!follower.isBusy()) {
                    follower.followPath(pickupartifactsmiddle, true);
                    setPathState(8);
                }
                break;

            case 8: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                    follower.followPath(shootartifctsmiddle, true);
                    setPathState(9);
                }
                break;

            case 9: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(10);
                }
                break;

            case 10: // Wait for shooter -> Park
                if (!shooter.isBusy()) {
                    follower.followPath(facegate, true);
                    setPathState(11);
                }
                break;

            case 11: // End
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // --- MAIN LOOPS ---

    @Override
    public void init() {
        // Initialize Timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- 3. INIT SHOOTER LOGIC ---
        shooter = new FlywheelLogic();
        shooter.init(hardwareMap);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        startIntake();
    }

    @Override
    public void loop() {
        // --- 4. UPDATE BOTH SYSTEMS CONSTANTLY ---

        // This runs the PIDF + Voltage + Servo Logic
        shooter.update();

        // This runs the Driving Logic
        follower.update();

        // This runs the "What should I do next?" Logic
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter Busy?", shooter.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    // ===================== INTAKE CONTROL METHODS =====================
    private void startIntake() {
        // Set power to 1.0 (full speed intake)
        intake.setPower(0.5);
    }

    private void reverseIntake() {
        // Set power to -1.0 (reverse/outtake) - useful for potential unjamming
        intake.setPower(-1.0);
    }

    private void stopIntake() {
        // Set power to 0.0 (stop motor)
        intake.setPower(0.0);
    }


    @Override
    public void init_loop() {}

    @Override
    public void stop() {
        stopIntake();
    }
}
