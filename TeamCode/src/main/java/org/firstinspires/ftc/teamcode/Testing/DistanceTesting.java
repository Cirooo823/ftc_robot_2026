package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "DistanceTesting", group = "Vision")
public class DistanceTesting extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    @Override
    public void runOpMode() throws InterruptedException {

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(cameraMonitorViewId)
                .build();

        // -------------------------------------------------------------
        // Accessing the Camera Controls
        // -------------------------------------------------------------

        // Wait for the camera to start streaming before accessing controls.
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                && !isStopRequested()) {
            sleep(20);
        }

        ExposureControl exposureControl =
                (ExposureControl) visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null && exposureControl.isExposureSupported()) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(1, TimeUnit.MILLISECONDS); // Change exposure
        }

        GainControl gainControl =
                (GainControl) visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(45); // Change gain
        }

        telemetry.addLine("VisionPortal ready, preview should appear on Driver Hub");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);

                telemetry.addData("tag id", tag.id);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);

                // ===== NEW: distance from camera to tag =====
                // ftcPose.range is the direct distance from the camera to the tag center.
                // Units are whatever you used in the AprilTagLibrary (usually inches).
                double range = tag.ftcPose.range;
                telemetry.addData("distance (range)", range);

                // (Optional) planar distance in X-Y plane if you want it:
                // double planarDistance = Math.hypot(tag.ftcPose.x, tag.ftcPose.y);
                // telemetry.addData("planar distance", planarDistance);

            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
            sleep(50);
        }

        visionPortal.close();
    }
}
