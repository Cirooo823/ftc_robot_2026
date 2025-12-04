package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Make sure there is no other @Autonomous with the same "name" string if you change this
@Autonomous(name = "Auto1", group = "Autos")
public class Auto1 extends OpMode {

    // === Pedro core ===
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // === Poses ===
    private final Pose startPose             = new Pose(56, 8, Math.toRadians(90));    // Start Pose of robot
    private final Pose shootfarPose          = new Pose(60.39344262295082, 19.475409836065573, Math.toRadians(113));
    private final Pose facingartifactsclosePose   = new Pose(42.295081967213115, 36, Math.toRadians(180));
    private final Pose pickupclosePose       = new Pose(19.672131147540984, 36, Math.toRadians(180));
    private final Pose facingartifactsmiddlePose  = new Pose(41.90163934426229, 59.60655737704917, Math.toRadians(180));
    private final Pose pickupmiddlePose      = new Pose(19.4098370661501, 59.672127145235656, Math.toRadians(180));
    private final Pose facingartifactsfarPose     = new Pose(38.95081967213115, 84.0, Math.toRadians(180));
    private final Pose pickupfarPose         = new Pose(19.882781982421875, 84.19023757684427, Math.toRadians(180));
    private final Pose shootclosePose        = new Pose(49.967213114754095, 102.68852459016394, Math.toRadians(137));
    private final Pose leavefarPose          = new Pose(37.37704918032787, 90.29508196721311, Math.toRadians(137));

    // === Paths ===
    private PathChain scorePreload;

    private PathChain faceArtifactsClose, pickupArtifactsClose, shootArtifactsClose;
    private PathChain faceArtifactsMiddle, pickupArtifactsMiddle, shootArtifactsMiddle;
    private PathChain faceArtifactsFar, pickupArtifactsFar, shootArtifactsFar;

    private PathChain leaveZone;

    // Build all path chains once
    public void buildPaths() {
        // Preload: start -> far shot
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootfarPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootfarPose.getHeading())
                .build();

        // Close cycle
        faceArtifactsClose = follower.pathBuilder()
                .addPath(new BezierLine(shootfarPose, facingartifactsclosePose))
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();

        pickupArtifactsClose = follower.pathBuilder()
                .addPath(new BezierLine(facingartifactsclosePose, pickupclosePose))
                .setTangentHeadingInterpolation()
                .build();

        shootArtifactsClose = follower.pathBuilder()
                .addPath(new BezierLine(pickupclosePose, shootfarPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();

        // Middle cycle
        faceArtifactsMiddle = follower.pathBuilder()
                .addPath(new BezierLine(shootfarPose, facingartifactsmiddlePose))
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();

        pickupArtifactsMiddle = follower.pathBuilder()
                .addPath(new BezierLine(facingartifactsmiddlePose, pickupmiddlePose))
                .setTangentHeadingInterpolation()
                .build();

        shootArtifactsMiddle = follower.pathBuilder()
                .addPath(new BezierLine(pickupmiddlePose, shootfarPose)) // your original had 60.590, 19.672; adjust if needed
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))
                .build();

        // Far cycle
        faceArtifactsFar = follower.pathBuilder()
                .addPath(new BezierLine(shootfarPose, facingartifactsfarPose))
                .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                .build();

        pickupArtifactsFar = follower.pathBuilder()
                .addPath(new BezierLine(facingartifactsfarPose, pickupfarPose))
                .setTangentHeadingInterpolation()
                .build();

        shootArtifactsFar = follower.pathBuilder()
                .addPath(new BezierLine(pickupfarPose, shootclosePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                .build();

        // Park
        leaveZone = follower.pathBuilder()
                .addPath(new BezierLine(shootclosePose, leavefarPose))
                .setTangentHeadingInterpolation()
                .build();
    }

    /** Finite-state machine for running your cycles. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Preload
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                // Face close stack
                if (!follower.isBusy()) {
                    follower.followPath(faceArtifactsClose, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Pickup close
                if (!follower.isBusy()) {
                    follower.followPath(pickupArtifactsClose, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Shoot close
                if (!follower.isBusy()) {
                    follower.followPath(shootArtifactsClose, true);
                    setPathState(4);
                }
                break;

            case 4:
                // Face middle stack
                if (!follower.isBusy()) {
                    follower.followPath(faceArtifactsMiddle, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Pickup middle
                if (!follower.isBusy()) {
                    follower.followPath(pickupArtifactsMiddle, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Shoot middle
                if (!follower.isBusy()) {
                    follower.followPath(shootArtifactsMiddle, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Face far stack
                if (!follower.isBusy()) {
                    follower.followPath(faceArtifactsFar, true);
                    setPathState(8);
                }
                break;

            case 8:
                // Pickup far
                if (!follower.isBusy()) {
                    follower.followPath(pickupArtifactsFar, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Shoot far
                if (!follower.isBusy()) {
                    follower.followPath(shootArtifactsFar, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Park
                if (!follower.isBusy()) {
                    follower.followPath(leaveZone, true);
                    setPathState(-1);   // done
                }
                break;
        }
    }

    /** Change state and reset the path timer. */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // === OpMode lifecycle methods ===

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Assuming you have a Constants.createFollower(hardwareMap) like in the Pedro docs
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        // Optional: keep follower updated or show telemetry while waiting for start
        follower.update();
        telemetry.addLine("Auto1 init_loop");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        // follower.stop(); // if you have a stop method / mechanism
    }
}
