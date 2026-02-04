package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name="TeleOp RED Adjustable RPM", group="TeleOp")
public class TeleOpAdjustableRed extends OpMode {

    private VoltageFlywheelController flywheelController;

    private DcMotorEx right_b, left_f, right_f, left_b;

    // ===== MECHANISMS =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor   intake;

    // ===== BARRIER (SERVO) =====
    private Servo barrierServo;
    private static final double BARRIER_CLOSED_POS = 0.67;
    private static final double BARRIER_OPEN_POS   = 0.0;
    private boolean barrierOpen = false;

    private double driverScale = 1.0;

    // ===== STATE =====
    private boolean intakeOn   = false;

    // Edge detection (g1 intake toggles)
    private boolean prevX = false;
    private boolean prevDU = false, prevDL = false, prevDR = false;

    // gamepad2 button edges
    private boolean prevA2  = false;
    private boolean prevX2  = false;
    private boolean prevY2  = false;
    private boolean prevB2  = false;
    private boolean prevRB2 = false;
    private boolean prevLT2 = false;
    private boolean prevRT2 = false;
    private boolean prevLSB2 = false;

    // ===== PRESET RPMs =====
    private static final int PRESET_STALL          = 2000;  // A -> low stall preset
    private static final int PRESET_SHORT_MED_RPM  = 3000;  // gamepad2 left trigger
    private static final int PRESET_MED_RPM        = 3400;  // gamepad2 Y
    private static final int PRESET_MED_LONG_RPM   = 3250;  // gamepad2 right bumper
    private static final int PRESET_LONG_RPM       = 3600;  // gamepad2 right trigger

    // Fine adjust step and minimum
    private static final int RPM_STEP = 50;
    private static final int MIN_RPM  = 500;

    // ===== FLYWHEEL READY-TO-SHOOT RUMBLE (gamepad2) =====
    private static final double RPM_TOLERANCE = 50.0;
    private boolean flywheelAtSpeed       = false;
    private boolean flywheelReadyRumbled  = false;
    private double  lastTargetRPM         = 0.0;

    // ===== VISION =====
    private VisionPortal      visionPortal;
    private AprilTagProcessor tagProcessor;
    private boolean           visionConfigured = false;

    // RED goal tag ID (from old red teleop)
    private static final int RED_GOAL_TAG_ID = 24;

    // Last seen tag info (for telemetry continuity)
    private int    lastTagId     = -1;
    private double lastTagRange  = 0.0;
    private double lastYaw       = 0.0;
    private double lastBearing   = 0.0;

    // For current frame
    private boolean tagVisible       = false;
    private double  currentBearing   = 0.0;

    // ===== AIM RUMBLE (gamepad1) WHEN ALIGNED TO TAG =====
    // Red-side aiming offset (ported from old red teleop)
    private static final double AIM_BEARING_TOL_DEG    = 1.5;   // window size
    private static final double AIM_BEARING_OFFSET_DEG = -2.0;  // ~2° to the RIGHT of tag center (negative)
    private boolean aimRumbleActive = false;

    @Override
    public void init() {
        // ---- Map drive ----
        left_f  = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b  = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

        left_f.setDirection(DcMotor.Direction.REVERSE);
        right_b.setDirection(DcMotor.Direction.REVERSE);
        right_f.setDirection(DcMotor.Direction.REVERSE);

        right_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Mechanisms ----
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");
        intake         = hardwareMap.get(DcMotor.class,   "intake");

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel_Right.setDirection(DcMotorSimple.Direction.FORWARD);

        // Voltage-compensated controller
        flywheelController = new VoltageFlywheelController(hardwareMap);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- Barrier servo ----
        barrierServo = hardwareMap.get(Servo.class, "barrierServo");
        barrierServo.setPosition(BARRIER_CLOSED_POS);
        barrierOpen = false;

        // ---- Vision init ----
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

        telemetry.addLine("TeleOp RED init: VisionPortal starting...");
        telemetry.update();
    }

    @Override
    public void loop() {
        configureVisionIfReady();

        drive();
        runFlywheel();
        runIntake();
        runBarrier();

        updateTagTelemetry_RedOnly();  // RED-specific: only tag 24
        flywheelController.update();
        updateFlywheelReadyRumble();
        updateAimRumble_RedOffset();   // RED-specific offset aiming

        // ===== TELEMETRY =====
        telemetry.addLine();
        telemetry.addData("--- Flywheel ---", "");
        telemetry.addData("Flywheel State", flywheelController.isFlywheelOn() ? "ACTIVE" : "OFF");
        telemetry.addData("Target RPM", "%.0f", flywheelController.getTargetRPM());
        telemetry.addData("Error RPM Left", "%.0f", flywheelController.getErrorRPM_Left());
        telemetry.addData("Error RPM Right", "%.0f", flywheelController.getErrorRPM_Right());
        telemetry.addData("Actual RPM Left", "%.0f", flywheelController.getCurrentRPM_Left());
        telemetry.addData("Actual RPM Right", "%.0f", flywheelController.getCurrentRPM_Right());
        telemetry.addData("Battery Voltage", "%.2f", flywheelController.getBatteryVoltage());

        telemetry.addLine();
        telemetry.addData("--- Barrier ---", "");
        telemetry.addData("Barrier", barrierOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Barrier Pos", "%.3f", barrierOpen ? BARRIER_OPEN_POS : BARRIER_CLOSED_POS);
        telemetry.addData("Barrier Ctrl", "g2 dpad_left=CLOSE, dpad_right=OPEN, dpad_up=TOGGLE");

        telemetry.addLine();
        telemetry.addData("--- Aim Assist (RED) ---", "");
        telemetry.addData("Target Tag ID", RED_GOAL_TAG_ID);
        telemetry.addData("Tag Visible", tagVisible);
        telemetry.addData("Last Tag ID", lastTagId);
        telemetry.addData("Last Range (in)", "%.2f", lastTagRange);
        telemetry.addData("Last Yaw (deg)", "%.2f", lastYaw);
        telemetry.addData("Last Bearing (deg)", "%.2f", lastBearing);
        telemetry.addData("Aim Offset (deg)", "%.2f", AIM_BEARING_OFFSET_DEG);
        telemetry.addData("Aim Aligned", aimRumbleActive);

        telemetry.update();
    }

    // ===================== VISION CONFIG =====================
    private void configureVisionIfReady() {
        if (visionPortal == null || visionConfigured) return;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl =
                    (ExposureControl) visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null && exposureControl.isExposureSupported()) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
            }

            GainControl gainControl =
                    (GainControl) visionPortal.getCameraControl(GainControl.class);
            if (gainControl != null) {
                gainControl.setGain(45);
            }

            visionConfigured = true;
            telemetry.addLine("Vision configured: exposure/gain set.");
        }
    }

    // ===================== VISION: RED GOAL ONLY =====================
    private void updateTagTelemetry_RedOnly() {
        if (tagProcessor == null) {
            tagVisible = false;
            return;
        }

        List<AprilTagDetection> detections = tagProcessor.getDetections();
        AprilTagDetection targetTag = null;

        // Only accept the RED goal tag (ID 24)
        for (AprilTagDetection det : detections) {
            if (det.id == RED_GOAL_TAG_ID) {
                targetTag = det;
                break;
            }
        }

        if (targetTag != null) {
            double range   = targetTag.ftcPose.range;
            double yaw     = targetTag.ftcPose.yaw;
            double bearing = targetTag.ftcPose.bearing;

            lastTagId     = targetTag.id;
            lastTagRange  = range;
            lastYaw       = yaw;
            lastBearing   = bearing;

            tagVisible     = true;
            currentBearing = bearing;
        } else {
            tagVisible = false;
        }
    }

    // ===================== FLYWHEEL =====================
    private void runFlywheel() {
        boolean x2   = gamepad2.x;
        boolean y2   = gamepad2.y;
        boolean b2   = gamepad2.b;
        boolean rb2  = gamepad2.right_bumper;
        boolean a2   = gamepad2.a;
        boolean lsb2 = gamepad2.left_stick_button;

        double ltVal = gamepad2.left_trigger;
        double rtVal = gamepad2.right_trigger;
        boolean lt2  = ltVal > 0.5;
        boolean rt2  = rtVal > 0.5;

        if (y2 && !prevY2) setFlywheelPreset(PRESET_MED_RPM);
        prevY2 = y2;

        if (rb2 && !prevRB2) setFlywheelPreset(PRESET_MED_LONG_RPM);
        prevRB2 = rb2;

        if (lt2 && !prevLT2) setFlywheelPreset(PRESET_SHORT_MED_RPM);
        prevLT2 = lt2;

        if (rt2 && !prevRT2) setFlywheelPreset(PRESET_LONG_RPM);
        prevRT2 = rt2;

        if (a2 && !prevA2) setFlywheelPreset(PRESET_STALL);
        prevA2 = a2;

        if (lsb2 && !prevLSB2) flywheelController.turnFlywheelOff();
        prevLSB2 = lsb2;

        if (b2 && !prevB2 && flywheelController.isFlywheelOn()) {
            double newTarget = flywheelController.getTargetRPM() + RPM_STEP;
            flywheelController.setFlywheelTargetRPM(newTarget);
        }

        if (x2 && !prevX2 && flywheelController.isFlywheelOn()) {
            double newTarget = flywheelController.getTargetRPM() - RPM_STEP;
            if (newTarget < MIN_RPM) newTarget = MIN_RPM;
            flywheelController.setFlywheelTargetRPM(newTarget);
        }

        prevX2 = x2;
        prevB2 = b2;
    }

    private void setFlywheelPreset(int rpm) {
        flywheelController.setFlywheelTargetRPM(rpm);
        flywheelController.turnFlywheelOn();
    }

    // ===================== READY-TO-SHOOT RUMBLE (gamepad2) =====================
    private void updateFlywheelReadyRumble() {
        double target = flywheelController.getTargetRPM();

        if (!flywheelController.isFlywheelOn() || target <= 0.0) {
            flywheelAtSpeed      = false;
            flywheelReadyRumbled = false;
            lastTargetRPM        = target;
            return;
        }

        if (target != lastTargetRPM) {
            flywheelAtSpeed      = false;
            flywheelReadyRumbled = false;
        }

        double errL = Math.abs(flywheelController.getErrorRPM_Left());
        double errR = Math.abs(flywheelController.getErrorRPM_Right());
        boolean atSpeedNow = (errL <= RPM_TOLERANCE) && (errR <= RPM_TOLERANCE);

        if (atSpeedNow && !flywheelAtSpeed && !flywheelReadyRumbled) {
            if (gamepad2 != null) gamepad2.rumbleBlips(1);
            flywheelReadyRumbled = true;
        }

        flywheelAtSpeed = atSpeedNow;
        lastTargetRPM   = target;
    }

    // ===================== AIM RUMBLE (gamepad1) RED OFFSET =====================
    private void updateAimRumble_RedOffset() {
        boolean flywheelActive = flywheelController.isFlywheelOn();

        // Bearing is "how many degrees to turn to face tag center".
        // For RED, we want a small offset from center.
        double bearingError = currentBearing - AIM_BEARING_OFFSET_DEG;

        boolean alignedToTag = flywheelActive
                && tagVisible
                && Math.abs(bearingError) <= AIM_BEARING_TOL_DEG;

        if (alignedToTag && !aimRumbleActive) {
            if (gamepad1 != null) {
                gamepad1.rumble(0.7, 0.7, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            }
            aimRumbleActive = true;
        } else if (!alignedToTag && aimRumbleActive) {
            if (gamepad1 != null) gamepad1.stopRumble();
            aimRumbleActive = false;
        }
    }

    // ===================== DRIVE =====================
    private void drive() {
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;

        double y  = -gamepad1.left_stick_y;
        double x  = -gamepad1.right_stick_x;
        double rx = -gamepad1.left_stick_x;

        double ys  = y  * driverScale;
        double xs  = x  * driverScale;
        double rxs = rx * driverScale;

        double d = Math.max(Math.abs(ys) + Math.abs(xs) + Math.abs(rxs), 1.0);

        right_f.setPower(( ys + xs + rxs) / d);
        left_b.setPower( ( ys - xs + rxs) / d);
        left_f.setPower( ( ys - xs - rxs) / d);
        right_b.setPower(( ys + xs - rxs) / d);
    }

    // ===================== INTAKE =====================
    private void runIntake() {
        boolean x = gamepad1.x;
        if (x && !prevX) intakeOn = !intakeOn;
        prevX = x;

        boolean b = gamepad1.b;

        double intakePower = 0.0;

        if (b) {
            intakePower = -1.0;
        } else if (intakeOn) {
            intakePower = 1.0;
        }

        intake.setPower(intakePower);
    }

    // ===================== BARRIER (SERVO) =====================
    private void runBarrier() {
        if (barrierServo == null) return;

        boolean dl = gamepad2.dpad_left;
        boolean dr = gamepad2.dpad_right;
        boolean du = gamepad2.dpad_up;

        if (dl && !prevDL) {
            barrierServo.setPosition(BARRIER_CLOSED_POS);
            barrierOpen = false;
        }

        if (dr && !prevDR) {
            barrierServo.setPosition(BARRIER_OPEN_POS);
            barrierOpen = true;
        }

        if (du && !prevDU) {
            barrierOpen = !barrierOpen;
            barrierServo.setPosition(barrierOpen ? BARRIER_OPEN_POS : BARRIER_CLOSED_POS);
        }

        prevDL = dl;
        prevDR = dr;
        prevDU = du;
    }

    @Override
    public void stop() {
        right_f.setPower(0);
        left_f.setPower(0);
        right_b.setPower(0);
        left_b.setPower(0);

        flywheelController.turnFlywheelOff();
        intake.setPower(0);

        if (barrierServo != null) {
            barrierServo.setPosition(BARRIER_CLOSED_POS);
            barrierOpen = false;
        }

        if (gamepad1 != null) gamepad1.stopRumble();
        if (gamepad2 != null) gamepad2.stopRumble();

        if (visionPortal != null) visionPortal.close();
    }
}
