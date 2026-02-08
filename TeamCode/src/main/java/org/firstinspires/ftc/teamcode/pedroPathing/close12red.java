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
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;



@Autonomous(name = "12 red", group = "Autos")
public class close12red extends OpMode {

    private boolean safetyTriggered = false; //for safety so it leaves


    // --- 1. CHANGE: Use the Logic Class ---
    private logic15red shooter;

    private boolean intakeOn   = true;
    private boolean barrierOpen = false;

    private DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private static final double INTAKE_TICKS_PER_REV = 145.1;


    private static final double INTAKE_RPM_BARRIER_CLOSED = 1150.0;
    private static final double INTAKE_RPM_BARRIER_OPEN   = 400.0;


    // --- POSES (Untouched) ---
    private final Pose startPose = new Pose(117.5, 130, Math.toRadians(44));


    private PathChain shootpreload;
    private PathChain facemiddle, intakemiddle, shootmiddle;
    private PathChain opengate, intakegate, shootgate;
    private PathChain faceclose, intakeclose, shootclose;
    private PathChain facegate;



    public void buildPaths() {
        shootpreload = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(117.500, 130.000),
                                new Pose(89.000, 118.000),
                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(48))

                .build();

        facemiddle = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.000, 88.000),
                                new Pose(84.000, 71.000),
                                new Pose(102.000, 58.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))

                .build();

        intakemiddle = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(101.000, 59.000),

                                new Pose(130.000, 59.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootmiddle = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.000, 59.000),
                                new Pose(92.000, 41.000),
                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))

                .build();

        opengate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(89.000, 88.000),
                                new Pose(88.000, 60.000),
                                new Pose(131.00, 61.00)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(30))

                .build();

        intakegate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131.00, 61.00),

                                new Pose(134.0, 61.0) //was 34, 60
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(35))

                .build();

        shootgate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134.0, 61.0), //was 34, 60
                                new Pose(92.000, 59.000),
                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(48))

                .build();


        faceclose = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 88.000),

                                new Pose(100.000, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))

                .build();

        intakeclose = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(100.000, 85.000),

                                new Pose(128.500, 85.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shootclose = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(128.500, 85.000),
                                new Pose(93.000, 65.000),
                                new Pose(89.000, 88.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))

                .build();

        facegate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89.000, 88.000),

                                new Pose(115.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))

                .build();
    }


    // --- 2. THE CORRECTED STATE MACHINE ---
    public void autonomousPathUpdate() {
        if (opmodeTimer.getElapsedTimeSeconds() > 29 && !safetyTriggered) { //if 28.5 seconds pass, so the auto doesnt just randomly stay there lol
            setPathState(99);
            safetyTriggered = true;
        }

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
                    follower.followPath(facemiddle, true);
                    setPathState(3);
                }
                break;


            case 3: // Arrived at Spike Mark -> Drive to Pickup
                if (!follower.isBusy()) {
                    follower.followPath(intakemiddle, true);
                    setPathState(4);
                }
                break;


            case 4: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                    follower.followPath(shootmiddle, true);
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
                    follower.followPath(opengate, true);
                    setPathState(7);
                }
                break;


            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(intakegate, true);
                    setPathState(8);
                }
                break;


            case 8: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                        follower.followPath(shootgate, true);
                        setPathState(9);
                    } else if (pathTimer.getElapsedTime() > 3000) {
                    follower.followPath(shootgate, true);
                    setPathState(9);
                }

                break;


            case 9: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(10);
                }
                break;


            case 10: // Wait for shooter -> Drive to Middle Spike
                if (!shooter.isBusy()) {
                    follower.followPath(faceclose, true);
                    setPathState(11);
                }
                break;


            case 11: // Arrived at Middle Spike -> Drive to Pickup
                if (!follower.isBusy()) {
                    follower.followPath(intakeclose, true);
                    setPathState(12);
                }
                break;


            case 12: // Arrived at Pickup -> Drive to Shoot
                if (!follower.isBusy()) {
                    follower.followPath(shootclose, true);
                    setPathState(13);
                }
                break;


            case 13: // Arrived at Shooting Spot -> FIRE
                if (!follower.isBusy()) {
                    shooter.fireShots(1);
                    setPathState(14);
                }
                break;


            case 14: // Wait for shooter -> Park
                if (!shooter.isBusy()) {
                    follower.followPath(facegate, true);
                    setPathState(15);
                }
                break;


            case 15: // End
                if (!follower.isBusy()) {
                    shooter.setFlywheelKeepAlive(false);
                    setPathState(-1);
                }
                break;

            case 99: // SAFETY
                shooter.setFlywheelKeepAlive(false); //turn off shooter
                intakeOn = false; //turn off intake

                follower.followPath(facegate, true);

                setPathState(-1); // End the state machine
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
        shooter = new logic15red();
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

        PoseStorage.lastPose = follower.getPose(); //ciro

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
        PoseStorage.lastPose = follower.getPose();
    }
}

