package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

/**
 * Simple static pose handoff between Autonomous and TeleOp.
 * Works as long as the RC app is not restarted between OpModes.
 */
public final class PoseStorage {
    private PoseStorage() {}

    // If your code never assigns this, TeleOp will fall back to TELEOP_START_POSE
    public static Pose lastPose = null;
}
