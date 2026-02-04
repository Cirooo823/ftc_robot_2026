package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "Test Vision + Distance Flywheel", group = "Vision")
public class TestVision extends LinearOpMode {

    // ===== VISION =====
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // ===== MECHANISMS (names copied from automatictele) =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor   intake;
    private CRServo   thirdStage;

    // ===== FLYWHEEL CONSTANTS (same encoder spec as automatictele) =====
    private static final double TICKS_PER_REV = 28.0;  // goBILDA / REV 5202-style encoder
    // Gear ratio between motor and flywheel wheel:
    // 1.0 = direct drive. If you upgear/downgear, change this.
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_FLYWHEEL_REV = TICKS_PER_REV * GEAR_RATIO;

    // ===== DISTANCE → RPM MAPPING (EXAMPLE NUMBERS; TUNE THESE) =====
    // Distances in inches (camera to AprilTag)
    private static final double MIN_DISTANCE_IN = 10.0;  // nearest planned shot
    private static final double MAX_DISTANCE_IN = 60.0;  // farthest planned shot

    // Desired RPM at those distances
    private static final double RPM_AT_MIN_DISTANCE = 3200.0;  // TODO: tune on field
    private static final double RPM_AT_MAX_DISTANCE = 5000.0;  // TODO: tune on field

    @Override
    public void runOpMode() throws InterruptedException {

        // =========================
        // 1. Hardware Init (Mechanisms)
        // =========================

        // Mechanism names exactly as in automatictele:
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");
        intake         = hardwareMap.get(DcMotor.class,   "intake");
        thirdStage     = hardwareMap.get(CRServo.class,   "thirdStage");

        // Flywheel configuration similar to your teleop
        flywheel_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Start flywheel stopped
        flywheel_Left.setVelocity(0);
        flywheel_Right.setVelocity(0);

        // Intake + third stage: just make them safe/idle here. You can add logic later.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);

        thirdStage.setPower(0.0);

        // =========================
        // 2. Vision Init: AprilTags
        // =========================
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(cameraMonitorViewId)
                .build();

        // =========================
        // 3. Camera Controls (exposure / gain)
        // =========================
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
                && !isStopRequested()) {
            sleep(20);
        }

        ExposureControl exposureControl =
                (ExposureControl) visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null && exposureControl.isExposureSupported()) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(1, TimeUnit.MILLISECONDS); // tune this if needed
        }

        GainControl gainControl =
                (GainControl) visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(45); // tune this if needed
        }

        telemetry.addLine("VisionPortal ready, preview should appear on Driver Hub");
        telemetry.addLine("Flywheel will use AprilTag distance → setVelocity()");
        telemetry.update();

        waitForStart();

        double lastTargetTps = 0.0;

        // =========================
        // 4. Main Loop
        // =========================
        while (opModeIsActive() && !isStopRequested()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);  // use first visible tag

                // Pose values
                double x     = tag.ftcPose.x;      // inches, X-axis
                double y     = tag.ftcPose.y;      // inches, Y-axis
                double z     = tag.ftcPose.z;      // inches, Z-axis
                double roll  = tag.ftcPose.roll;   // degrees
                double pitch = tag.ftcPose.pitch;  // degrees
                double yaw   = tag.ftcPose.yaw;    // degrees

                // Direct range (distance from camera to tag, in inches by default)
                double range = tag.ftcPose.range;

                // ----- Distance → RPM → ticks/sec -----
                double targetRpm = distanceToTargetRpm(range);
                double targetTps = rpmToTicksPerSecond(targetRpm);

                // Command flywheel using setVelocity()
                flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flywheel_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flywheel_Left.setVelocity(targetTps);
                flywheel_Right.setVelocity(targetTps);
                lastTargetTps = targetTps;

                // ----- Telemetry -----
                telemetry.addLine("=== TAG DETECTED ===");
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("x (in)", "%.2f", x);
                telemetry.addData("y (in)", "%.2f", y);
                telemetry.addData("z (in)", "%.2f", z);
                telemetry.addData("roll (deg)", "%.1f", roll);
                telemetry.addData("pitch (deg)", "%.1f", pitch);
                telemetry.addData("yaw (deg)", "%.1f", yaw);

                telemetry.addData("range (in)", "%.2f", range);
                telemetry.addData("target RPM", "%.1f", targetRpm);
                telemetry.addData("target tps", "%.1f", targetTps);

            } else {
                telemetry.addLine("No tags detected");
                telemetry.addData("flywheel (tps)", "%.1f", lastTargetTps);

                // Depending on your preference, you can:
                //  - keep lastTargetTps (current behavior), or
                //  - stop the flywheel when no tag is visible:
                // flywheel_Left.setVelocity(0);
                // flywheel_Right.setVelocity(0);
                // lastTargetTps = 0.0;
            }

            telemetry.update();
            sleep(50);
        }

        // Clean shutdown
        flywheel_Left.setVelocity(0);
        flywheel_Right.setVelocity(0);
        intake.setPower(0.0);
        thirdStage.setPower(0.0);
        visionPortal.close();
    }

    // =========================================================
    // Helper: map distance (inches) -> target flywheel RPM
    // =========================================================
    private double distanceToTargetRpm(double distanceInches) {
        // Clamp into [MIN_DISTANCE_IN, MAX_DISTANCE_IN] to avoid crazy values
        double d = clamp(distanceInches, MIN_DISTANCE_IN, MAX_DISTANCE_IN);

        // Linear interpolation between two calibrated points
        double t = (d - MIN_DISTANCE_IN) / (MAX_DISTANCE_IN - MIN_DISTANCE_IN); // 0 near, 1 far
        return RPM_AT_MIN_DISTANCE + t * (RPM_AT_MAX_DISTANCE - RPM_AT_MIN_DISTANCE);
    }

    // =========================================================
    // Helper: convert RPM -> ticks per second for setVelocity()
    // =========================================================
    private double rpmToTicksPerSecond(double rpm) {
        double revPerSec = rpm / 60.0;
        return revPerSec * TICKS_PER_FLYWHEEL_REV;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
