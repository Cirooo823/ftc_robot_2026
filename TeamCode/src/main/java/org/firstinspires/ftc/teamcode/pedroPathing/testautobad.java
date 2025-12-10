package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TeleOp.VoltageFlywheelController;


@Autonomous(name = "Auto9artifacts", group = "Autos")
public class testautobad extends OpMode {

    private VoltageFlywheelController flywheelController;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // --------FLYWHEEL LOGIC --------
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;

    private int pathState;




    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of robot.
    private final Pose shootfarPose = new Pose(57, 19.475409836065573, Math.toRadians(113)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose facingartifactsclosePose = new Pose(42.295081967213115, 36, Math.toRadians(180)); //
    private final Pose pickupclosePose = new Pose(19.672131147540984, 36, Math.toRadians(180)); //
    private final Pose facingartifactsmiddlePose = new Pose(41.90163934426229, 59.60655737704917, Math.toRadians(180)); //
    private final Pose pickupmiddlePose = new Pose(19.4098370661501, 59.672127145235656, Math.toRadians(180)); //
    private final Pose facingartifactsfarPose = new Pose(38.95081967213115, 83.99999999999999, Math.toRadians(180)); //
    private final Pose pickupfarPose = new Pose(19.882781982421875, 84.19023757684427, Math.toRadians(180)); //
    private final Pose shootclosePose = new Pose(49.967213114754095, 102.68852459016394, Math.toRadians(137)); //
    private final Pose leavefarPose = new Pose(37.37704918032787, 90.29508196721311, Math.toRadians(137)); //
    private final Pose leaveclosePose = new Pose(37.37704918032787, 90.29508196721311, Math.toRadians(137)); //


    private PathChain scorepreload;
    private PathChain faceartifactsclose, pickupartifactsclose, shootartifactsclose;
    private PathChain faceartifactsmiddle, pickupartifactsmiddle, shootartifctsmiddle;
    private PathChain faceartifactsfar, pickupartifactsfar, shootartifactsfar;
    private PathChain leaveclose;
    public void buildPaths() {
        scorepreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootfarPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootfarPose.getHeading())
                .build();


        faceartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 19.475), new Pose(42.295, 32.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();


        pickupartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.295, 32.500), new Pose(19.672, 32.500))
                )
                .setTangentHeadingInterpolation()
                .build();


        shootartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.672, 32.500), new Pose(57, 19.475))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();


        faceartifactsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 19.475), new Pose(41.902, 56.1))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();


        pickupartifactsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.902, 56.1), new Pose(19.410, 56.1))
                )
                .setTangentHeadingInterpolation()
                .build();


        shootartifctsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.410, 56.1), new Pose(57, 19.672))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();


        leaveclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.000, 19.672), new Pose(54.885, 24.984))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(130))
                .build();

    }


    public void autonomousPathUpdate () {
        switch(pathState) {
            case 0:
                follower.followPath(scorepreload);
                setPathState(1);
                break;
            case 1:
                //time elapsed 5 secs
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()){
                        //shots are done, free to transition
                        follower.followPath(faceartifactsclose,true);
                        setPathState(2);
                    }
                }

                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(pickupartifactsclose,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(shootartifactsclose,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(faceartifactsmiddle,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(pickupartifactsmiddle,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(shootartifctsmiddle, true);
                    setPathState(7);
                }
            case 7:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(leaveclose, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        flywheelController.update();
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        shooter.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        flywheelController=new VoltageFlywheelController(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        shooter.init(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}}