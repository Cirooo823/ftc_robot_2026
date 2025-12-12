package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Far RED", group = "Autos")
public class FarRED extends OpMode {

    // --- 1. CHANGE: Use the Logic Class ---
    private FlywheelLogic shooter;

    private DcMotor intake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // --- POSES (Untouched) ---

    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));


    private PathChain shootpreload;
    private PathChain faceartifactsclose, pickupartifactsclose, shootartifactsclose;
    private PathChain faceartifactsmiddle, pickupartifactsmiddle, shootartifactsmiddle;
    private PathChain faceartifactsfar, pickupartifactsfar, shootartifactsfar;
    private PathChain facegate;

    public void buildPaths() {
        shootpreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.000, 9.000), new Pose(88.000, 21.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                .build();

        faceartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 21.000), new Pose(98.000, 34.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))
                .build();

        pickupartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(98.000, 34.500), new Pose(133.500, 34.500))
                )
                .setTangentHeadingInterpolation()
                .build();

        shootartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(133.500, 34.500), new Pose(88.000, 21.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                .build();

        faceartifactsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 21.000), new Pose(98.000, 58.200))
                )
                .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))
                .build();

        pickupartifactsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(98.000, 58.200), new Pose(133.500, 58.200))
                )
                .setTangentHeadingInterpolation()
                .build();

        shootartifactsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(133.500, 58.200), new Pose(88.000, 21.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                .build();

        facegate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 21.000), new Pose(120.000, 68.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))
                .build();
    }

    // --- 2. THE CORRECTED STATE MACHINE ---
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to Shoot Preload
                follower.followPath(shootpreload);
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
                    follower.followPath(shootartifactsmiddle, true);
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
