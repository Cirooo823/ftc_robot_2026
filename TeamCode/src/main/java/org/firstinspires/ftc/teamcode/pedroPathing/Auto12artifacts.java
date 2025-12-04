package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "Auto12artifacts", group = "Autos")
public class Auto12artifacts extends OpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;




    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of robot.
    private final Pose shootfarPose = new Pose(60.39344262295082, 19.475409836065573, Math.toRadians(113)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose facingartifactsclosePose = new Pose(42.295081967213115, 36, Math.toRadians(180)); //
    private final Pose pickupclosePose = new Pose(19.672131147540984, 36, Math.toRadians(180)); //
    private final Pose facingartifactsmiddlePose = new Pose(41.90163934426229, 59.60655737704917, Math.toRadians(180)); //
    private final Pose pickupmiddlePose = new Pose(19.4098370661501, 59.672127145235656, Math.toRadians(180)); //
    private final Pose facingartifactsfarPose = new Pose(38.95081967213115, 83.99999999999999, Math.toRadians(180)); //
    private final Pose pickupfarPose = new Pose(19.882781982421875, 84.19023757684427, Math.toRadians(180)); //
    private final Pose shootclosePose = new Pose(49.967213114754095, 102.68852459016394, Math.toRadians(137)); //
    private final Pose leavefarPose = new Pose(37.37704918032787, 90.29508196721311, Math.toRadians(137)); //


    private PathChain scorepreload;
    private PathChain faceartifactsclose, pickupartifactsclose, shootartifactsclose;
    private PathChain faceartifactsmiddle, pickupartifactsmiddle, shootartifctsmiddle;
    private PathChain faceartifactsfar, pickupartifactsfar, shootartifactsfar;
    private PathChain leavezone;
    public void buildPaths() {
        scorepreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootfarPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootfarPose.getHeading())
                .build();


        faceartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.393, 19.475), new Pose(42.295, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();


        pickupartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.295, 36.000), new Pose(19.672, 36.000))
                )
                .setTangentHeadingInterpolation()
                .build();


        shootartifactsclose = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.672, 36.000), new Pose(60.393, 19.475))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();


        faceartifactsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.393, 19.475), new Pose(41.902, 59.607))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();


        pickupartifactsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.902, 59.607), new Pose(19.410, 59.672))
                )
                .setTangentHeadingInterpolation()
                .build();


        shootartifctsmiddle = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.410, 59.672), new Pose(60.590, 19.672))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();


        faceartifactsfar = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.590, 19.672), new Pose(38.951, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();


        pickupartifactsfar = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(38.951, 84.000), new Pose(19.883, 84.190))
                )
                .setTangentHeadingInterpolation()
                .build();


        shootartifactsfar = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.883, 84.190), new Pose(49.967, 102.689))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();


        leavezone = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(49.967, 102.689), new Pose(37.377, 90.295)))
                .setTangentHeadingInterpolation()
                .build();


    }


    public void autonomousPathUpdate () {
        switch(pathState) {
            case 0:
                follower.followPath(scorepreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(faceartifactsclose,true);
                    setPathState(2);
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
                if(!follower.isBusy()) {
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
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(faceartifactsfar, true);
                    setPathState(7);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(pickupartifactsfar, true);
                    setPathState(7);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(shootartifactsfar, true);
                    setPathState(7);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(leavezone, true);
                    setPathState(7);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
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