package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous(name = "far 6 red", group = "Autos")
public class far6red extends OpMode {


    // --- 1. CHANGE: Use the Logic Class ---
    private logic6far shooter;

    private boolean intakeOn   = true;
    private boolean barrierOpen = false;

    private DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private static final double INTAKE_TICKS_PER_REV = 145.1;


    private static final double INTAKE_RPM_BARRIER_CLOSED = 1150.0;
    private static final double INTAKE_RPM_BARRIER_OPEN   = 230.0;


    // --- POSES (Untouched) ---
    private final Pose startPose = new Pose(87, 9.7, Math.toRadians(90));


    private PathChain shootpreload;
    private PathChain facefar, intakefar, shootfar;
    private PathChain intakehpzone, backup, intakehpzone2, shoothpzone;
    private PathChain leavezone;



    public void buildPaths() {
        shootpreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.000, 9.700),

                                new Pose(85.000, 19.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))

                .build();

        facefar = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.000, 19.000),

                                new Pose(102.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))

                .build();

        intakefar = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(102.000, 35.000),

                                new Pose(133.000, 35.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootfar = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(133.000, 35.000),
                                new Pose(94.000, 0.9),
                                new Pose(85.000, 19.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))

                .build();

        intakehpzone = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.000, 19.000),
                                new Pose(98.500, 9.000),
                                new Pose(135.000, 10.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))

                .build();

        backup = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(135.000, 10.500),

                                new Pose(125.000, 10.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        intakehpzone2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.000, 10.500),

                                new Pose(135.000, 10.500)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shoothpzone = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(135.000, 10.500),
                                new Pose(101.000, 34.000),
                                new Pose(85.000, 19.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))

                .build();

        leavezone = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.000, 19.000),

                                new Pose(92.000, 26.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(40))

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
                    follower.followPath(intakefar, true);
                    setPathState(4);
                }
                break;


            case 4: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                    follower.followPath(shootfar, true);
                    setPathState(5);
                }
                break;


            case 5: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(6);
                }
                break;


            case 6: // Wait for shooter to finish -> Drive to Spike Mark
                if (!shooter.isBusy()) { // Wait until shooter returns to IDLE
                    follower.followPath(intakehpzone, true);
                    setPathState(7);
                }
                break;


            case 7: // Arrived at Spike Mark -> Drive to Pickup
               if (!follower.isBusy()) {
                        follower.followPath(backup, true);
                        setPathState(8);
               }
                    else if (pathTimer.getElapsedTime() > 1400) {
                        follower.followPath(backup, true);
                        setPathState(8);
                    }

                break;


            case 8: // Arrived at Spike Mark -> Drive to Pickup
                if (!follower.isBusy()) {
                    follower.followPath(intakehpzone2, true);
                    setPathState(9);
                }
                break;


            case 9: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                        follower.followPath(shoothpzone, true);
                        setPathState(10);
                   }

                    else if (pathTimer.getElapsedTime() > 1500) {
                        follower.followPath(shoothpzone, true);
                        setPathState(10);
                    }

                break;


            case 10: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(11);
                }
                break;


            case 11: // Wait for shooter -> Park
                if (!shooter.isBusy()) {
                    follower.followPath(leavezone, true);
                    setPathState(12);
                }
                break;


            case 12: // End
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


        intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // --- 3. INIT SHOOTER LOGIC ---
        shooter = new logic6far();
        shooter.init(hardwareMap);
    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        shooter.setFlywheelKeepAlive(true); //put this so that the flywheel stays on

        runIntake();
    }


    @Override
    public void loop() {
        // --- 4. UPDATE BOTH SYSTEMS CONSTANTLY ---


        // This runs the PIDF + Voltage + Servo Logic
        shooter.update();


        // This runs the Driving Logic
        follower.update();

        this.barrierOpen = shooter.isBarrierOpen();

        runIntake();

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
        // intake.setPower(1); //was at 0.5 in the regional but I changed it bc of hardware updates
    }

    private void runIntake() {

        // Convert RPM -> ticks/sec because DcMotorEx.setVelocity() expects ticks/sec.
        double targetRpm = 0.0;


        // B overrides everything: hard reverse while held (use max RPM to clear jams)
        if  (intakeOn) {
            targetRpm = barrierOpen ? INTAKE_RPM_BARRIER_OPEN : INTAKE_RPM_BARRIER_CLOSED;
        } else {
            // off
            intake.setPower(0.0);
            return;
        }


        double ticksPerSecond = (targetRpm * INTAKE_TICKS_PER_REV) / 60.0;
        intake.setVelocity(ticksPerSecond);
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

