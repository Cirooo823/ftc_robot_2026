package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Close BLUE Fly On", group = "Autos")
public class BlueCloseFlyON extends OpMode {


    // --- 1. CHANGE: Use the Logic Class ---
    private LogicFlyONClose shooter;


    private DcMotor intake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    // --- POSES (Untouched) ---
    private final Pose startPose = new Pose(121.5, 123.5, Math.toRadians(36));


    private PathChain shootpreload;
    private PathChain facefar, pickupfar, opengate, shootfar;
    private PathChain facemiddle, pickupmiddle, shootmiddle;
    private PathChain faceclose, pickupclose, shootclose;
    private PathChain facegate;


    public void buildPaths() {
        shootpreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.000, 123.000),

                                new Pose(56.000, 87.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))

                .build();

        facefar = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 87.000),

                                new Pose(42.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        pickupfar = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 84.000),

                                new Pose(16.000, 84.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        opengate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(16.000, 84.000),
                                new Pose(41.500, 73.500),
                                new Pose(17.000, 77.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        shootfar = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.000, 77.000),

                                new Pose(56.000, 87.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        facemiddle = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 87.000),

                                new Pose(42.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        pickupmiddle = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 60.000),

                                new Pose(9.000, 60.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootmiddle = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9.000, 60.000),
                                new Pose(62.500, 50.000),
                                new Pose(56.000, 87.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        faceclose = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 87.000),

                                new Pose(42.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        pickupclose = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 36.000),

                                new Pose(9.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootclose = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.000, 36.000),

                                new Pose(56.000, 87.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        facegate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 87.000),

                                new Pose(20.000, 70.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

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
                    follower.followPath(facefar, true);
                    setPathState(3);
                }
                break;


            case 3: // Arrived at Spike Mark -> Drive to Pickup
                if (!follower.isBusy()) {
                    follower.followPath(pickupfar, true);
                    setPathState(4);
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(opengate, true);
                    setPathState(5);
                }
                break;


            case 5: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                    follower.followPath(shootfar, true);
                    setPathState(6);
                }
                break;


            case 6: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(7);
                }
                break;


            case 7: // Wait for shooter -> Drive to Middle Spike
                if (!shooter.isBusy()) {
                    follower.followPath(facemiddle, true);
                    setPathState(8);
                }
                break;


            case 8: // Arrived at Middle Spike -> Drive to Pickup
                if (!follower.isBusy()) {
                    follower.followPath(pickupmiddle, true);
                    setPathState(9);
                }
                break;


            case 9: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                    follower.followPath(shootmiddle, true);
                    setPathState(10);
                }
                break;


            case 10: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(11);
                }
                break;


            case 11: // Wait for shooter -> Drive to Middle Spike
                if (!shooter.isBusy()) {
                    follower.followPath(faceclose, true);
                    setPathState(12);
                }
                break;


            case 12: // Arrived at Middle Spike -> Drive to Pickup
                if (!follower.isBusy()) {
                    follower.followPath(pickupclose, true);
                    setPathState(13);
                }
                break;


            case 13: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                    follower.followPath(shootclose, true);
                    setPathState(14);
                }
                break;


            case 14: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(15);
                }
                break;


            case 15: // Wait for shooter -> Park
                if (!shooter.isBusy()) {
                    follower.followPath(facegate, true);
                    setPathState(16);
                }
                break;


            case 16: // End
                if (!follower.isBusy()) {
                    shooter.setFlywheelKeepAlive(false);
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
        shooter = new LogicFlyONClose();
        shooter.init(hardwareMap);
    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        shooter.setFlywheelKeepAlive(true); //put this so that the flywheel stays on

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
        intake.setPower(1); //was at 0.5 in the regional but I changed it bc of hardware updates
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
        shooter.setFlywheelKeepAlive(false);
    }
}

