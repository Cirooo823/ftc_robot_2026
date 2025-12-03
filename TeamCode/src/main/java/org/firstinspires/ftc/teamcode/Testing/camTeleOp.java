package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase; // optional: season library

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="camTeleOp", group="TeleOp")
public class camTeleOp extends OpMode {

    // --- Drive ---
    private DcMotorEx right_b, left_f, right_f, left_b;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private boolean controlsApplied = false;

    // --- Controls ---
    private static final int TARGET_TAG_ID = 24;
    private boolean followEnabled = false;   // hold RB to center/hold
    private double driverScale = 1.0;        // hold LB for 0.25x
    private boolean prevX = false;           // rising-edge detector for X
    private boolean needRBRelease = false;   // after auto-return, ignore RB until released

    // --- Targets & tuning (FTC camera frame: X=sideways, Y=forward, Z=up) ---
    private double targetY = 0.55;     // meters (desired standoff), captured on X
    private double xBiasM = 0.00;      // meters (camera lateral bias), captured on X
    private double yawBiasDeg = 0.00;  // degrees (camera yaw bias), captured on X

    private static final double DEAD_X_M   = 0.04;
    private static final double DEAD_Y_M   = 0.05;
    private static final double DEAD_YAW_D = 2.0;

    private static final double ALIGN_YAW_OK_DEG = 3.0;
    private static final double ALIGN_X_OK_M     = 0.05;

    private static final double KX   = 0.45;   // strafe P
    private static final double KY   = 0.45;   // forward P
    private static final double KYAW = 0.015;  // turn P

    private static final double MAX_X  = 0.5;
    private static final double MAX_Y  = 0.5;
    private static final double MAX_YR = 0.4;

    // Flip if any axis is inverted on your bot
    private static final int SIGN_X   = +1;
    private static final int SIGN_Y   = +1;
    private static final int SIGN_YAW = -1; // (we flipped this earlier so positive yaw error turns right)

    // Filters
    private static final double ALPHA_X   = 0.5;
    private static final double ALPHA_Y   = 0.5;
    private static final double ALPHA_YAW = 0.5;
    private Double xFilt=null, yFilt=null, yawFilt=null;

    private long lastSeenMs = 0;
    private static final long LOST_MS = 250;

    // --- Search behavior: rotate in place (right) when searching ---
    private static final double SEARCH_TURN = +0.18; // + = rotate right, - = left

    // --- Auto return to DRIVER when near center ---
    private static final double AUTO_RELEASE_X_WIN   = 0.06; // meters from center (x)
    private static final double AUTO_RELEASE_YAW_WIN = 3.5;  // degrees from straight (yaw)
    private static final double AUTO_RELEASE_Y_WIN   = 0.10; // meters from targetY

    @Override
    public void init() {
        // DRIVE
        left_f  = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b  = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");
        left_f.setDirection(DcMotorEx.Direction.REVERSE);

        // VISION
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                // .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // optional: force season library
                .build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int monitorId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(monitorId)
                .build();

        telemetry.addLine("Init: waiting for stream to apply camera controls…");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Apply exposure/gain AFTER streaming starts
        if (visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING
                && !controlsApplied) {

            ExposureControl exposure = (ExposureControl) visionPortal.getCameraControl(ExposureControl.class);
            if (exposure != null && exposure.isExposureSupported()) {
                exposure.setMode(ExposureControl.Mode.Manual);
                exposure.setExposure(1, TimeUnit.MILLISECONDS);
            }
            GainControl gain = (GainControl) visionPortal.getCameraControl(GainControl.class);
            if (gain != null) gain.setGain(45);

            controlsApplied = true;
            telemetry.addLine("Camera controls applied.");
        }
        telemetry.addData("Camera", visionPortal.getCameraState());
        telemetry.update();
    }

    @Override
    public void loop() {
        // 0.25× DPI when holding LB
        driverScale = gamepad1.left_bumper ? 0.25 : 1.0;

        // Rising edge on X to capture setpoint & biases from tag 24
        boolean xPressed = gamepad1.x;
        if (xPressed && !prevX) {
            AprilTagDetection cand = findTagId(tagProcessor.getDetections(), TARGET_TAG_ID);
            if (cand != null) {
                telemetry.addData("Check/ID", cand.id);
                telemetry.addData("hasMetadata", (cand.metadata != null));
                telemetry.addData("hasPose(ftcPose)", (cand.ftcPose != null));
                if (cand.ftcPose != null) {
                    targetY    = cand.ftcPose.y;
                    xBiasM     = cand.ftcPose.x;
                    yawBiasDeg = cand.ftcPose.yaw;
                    telemetry.addData("Captured", "Y=%.3f m, Xbias=%.3f m, YawBias=%.2f°", targetY, xBiasM, yawBiasDeg);
                } else {
                    telemetry.addLine("X pressed: Tag 24 seen, but no pose. Add it to a Library with size.");
                }
            } else {
                telemetry.addLine("X pressed: Tag 24 not detected.");
            }
        }
        prevX = xPressed;

        // RB logic with auto-release latch:
        boolean rb = gamepad1.right_bumper;
        if (needRBRelease) {
            followEnabled = false;         // ignore RB until released once
            if (!rb) needRBRelease = false;
        } else {
            followEnabled = rb;            // hold-to-follow
        }

        if (followEnabled) {
            centerAndHoldTag24();
        } else {
            manualDrive();
        }

        telemetry.addData("DPI Scale", driverScale);
        telemetry.addData("targetY(m)", targetY);
        telemetry.addData("xBias(m)", xBiasM);
        telemetry.addData("yawBias(°)", yawBiasDeg);
        telemetry.addData("needRBRelease", needRBRelease);
        telemetry.update();
    }

    private void manualDrive() {
        double x  = -gamepad1.right_stick_x;
        double y  = -gamepad1.left_stick_y;
        double rx = -gamepad1.left_stick_x;
        setDrivePowers(y, x, rx);
        telemetry.addLine("Mode: DRIVER");
    }

    /** Centers yaw → centers X → holds Y, ONLY for Tag ID 24, with pose check + rotate-right search + auto-return. */
    private void centerAndHoldTag24() {
        List<AprilTagDetection> dets = tagProcessor.getDetections();
        AprilTagDetection cand = findTagId(dets, TARGET_TAG_ID);

        // Diagnostic: show ID, metadata, and pose availability
        if (cand != null) {
            telemetry.addData("Check/ID", cand.id);
            telemetry.addData("hasMetadata", (cand.metadata != null));
            telemetry.addData("hasPose(ftcPose)", (cand.ftcPose != null));
        }

        if (cand == null) {
            // Not seen at all -> rotate right to search
            setDrivePowers(0, 0, SEARCH_TURN);
            telemetry.addLine("CENTER(ID 24): searching… (rotating right)");
            return;
        }

        if (cand.ftcPose == null) {
            // Seen by ID, but no pose -> rotate right until a library pose is available
            setDrivePowers(0, 0, SEARCH_TURN);
            telemetry.addLine("CENTER(ID 24): Tag seen but no pose — rotating right (add Library with size).");
            return;
        }

        // We have pose
        lastSeenMs = System.currentTimeMillis();

        // Errors in FTC camera frame (X=sideways, Y=forward)
        double xErr   = cand.ftcPose.x - xBiasM;           // want 0
        double yErr   = cand.ftcPose.y - targetY;          // want 0
        double yawErr = cand.ftcPose.yaw - yawBiasDeg;     // want 0

        // --- AUTO RETURN TO DRIVER if near center ---
        if (Math.abs(xErr) <= AUTO_RELEASE_X_WIN
                && Math.abs(yawErr) <= AUTO_RELEASE_YAW_WIN
                && Math.abs(yErr) <= AUTO_RELEASE_Y_WIN) {
            setDrivePowers(0, 0, 0);
            needRBRelease = true; // latch so we ignore RB until user releases
            telemetry.addLine("CENTER(ID 24): centered — auto-returning to DRIVER (release RB).");
            return;
        }

        // EMA filtering
        xFilt   = (xFilt   == null) ? xErr   : (ALPHA_X   * xFilt   + (1 - ALPHA_X)   * xErr);
        yFilt   = (yFilt   == null) ? yErr   : (ALPHA_Y   * yFilt   + (1 - ALPHA_Y)   * yErr);
        yawFilt = (yawFilt == null) ? yawErr : (ALPHA_YAW * yawFilt + (1 - ALPHA_YAW) * yawErr);

        double xE = xFilt, yE = yFilt, yawE = yawFilt;

        // Deadbands
        if (Math.abs(xE)   < DEAD_X_M)   xE = 0;
        if (Math.abs(yE)   < DEAD_Y_M)   yE = 0;
        if (Math.abs(yawE) < DEAD_YAW_D) yawE = 0;

        // Stage control: 1) align yaw, 2) center X, 3) hold distance
        double rxCmd, xCmd, yCmd;

        if (Math.abs(yawE) > ALIGN_YAW_OK_DEG) {
            rxCmd = clamp(KYAW * (SIGN_YAW * yawE), -MAX_YR, MAX_YR);
            xCmd  = 0;
            yCmd  = 0;
            telemetry.addLine("CENTER(ID 24): aligning yaw");
        } else if (Math.abs(xE) > ALIGN_X_OK_M) {
            rxCmd = clamp(KYAW * (SIGN_YAW * yawE), -0.25, 0.25);
            xCmd  = clamp(KX   * (SIGN_X   * xE),   -MAX_X,  MAX_X);
            yCmd  = 0;
            telemetry.addLine("CENTER(ID 24): centering X");
        } else {
            rxCmd = clamp(KYAW * (SIGN_YAW * yawE), -0.20, 0.20);
            xCmd  = clamp(KX   * (SIGN_X   * xE),   -0.20, 0.20);
            yCmd  = clamp(KY   * (SIGN_Y   * yE),   -MAX_Y,  MAX_Y);
            telemetry.addLine("CENTER(ID 24): holding distance");
        }

        setDrivePowers(yCmd, xCmd, rxCmd);
    }

    /** Return the first detection with the given ID (even if ftcPose is null). */
    private AprilTagDetection findTagId(List<AprilTagDetection> dets, int id) {
        if (dets == null) return null;
        for (AprilTagDetection d : dets) {
            if (d.id == id) return d;
        }
        return null;
    }

    private void setDrivePowers(double y, double x, double rx) {
        // Apply DPI scaling globally
        double ys  = y  * driverScale;
        double xs  = x  * driverScale;
        double rxs = rx * driverScale;

        // Standard mecanum mixer
        double d = Math.max(Math.abs(ys) + Math.abs(xs) + Math.abs(rxs), 1.0);
        right_f.setPower(( ys + xs + rxs) / d);
        left_b.setPower( ( ys - xs + rxs) / d);
        left_f.setPower( ( ys - xs - rxs) / d);
        right_b.setPower(( ys + xs - rxs) / d);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}
