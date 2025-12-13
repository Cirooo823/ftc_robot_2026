package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOp RED Adjustable RPM", group="TeleOp")
public class TeleOpAdjustableRed extends OpMode {

    private VoltageFlywheelController flywheelController;

    private DcMotorEx right_b, left_f, right_f, left_b;

    // ===== MECHANISMS =====
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor intake;
    private CRServo thirdStage;

    private double driverScale = 1.0;

    // ===== STATE =====
    private boolean flywheelOn = false; // legacy flag, not strictly needed
    private boolean intakeOn = false;

    // Edge detection (g1 intake toggles)
    private boolean prevA = false, prevX = false;
    private boolean prevDU = false, prevDD = false, prevDL = false, prevDR = false;

    // gamepad2 button edges
    private boolean prevA2 = false;
    private boolean prevX2 = false;
    private boolean prevY2 = false;
    private boolean prevB2 = false;
    private boolean prevLB2 = false;
    private boolean prevRB2 = false;
    private boolean prevLT2 = false;
    private boolean prevRT2 = false;
    private boolean prevLSB2 = false;

    private int thirdDir = 0;

    // ===== PRESET RPMs =====
    private static final int PRESET_STALL = 500;   // A -> low stall preset
    private static final int PRESET_SHORT_MED_RPM = 2900;  // gamepad2 left trigger
    private static final int PRESET_MED_RPM = 3400;  // gamepad2 Y
    private static final int PRESET_MED_LONG_RPM = 3250;  // gamepad2 right bumper
    private static final int PRESET_LONG_RPM = 3300;  // gamepad2 right trigger

    // Fine adjust step and minimum
    private static final int RPM_STEP = 50;
    private static final int MIN_RPM = 500;

    // Intake power threshold based on flywheel RPM
    private static final double INTAKE_RPM_LIMIT_THRESHOLD = 700.0;

    // ===== FLYWHEEL READY-TO-SHOOT RUMBLE (gamepad2) =====
    private static final double RPM_TOLERANCE = 50.0;
    private boolean flywheelAtSpeed = false;
    private boolean flywheelReadyRumbled = false;
    private double lastTargetRPM = 0.0;

    // ===== VISION =====
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private boolean visionConfigured = false;

    // Red goal tag ID
    private static final int RED_GOAL_TAG_ID = 24;

    // Last seen tag info (for telemetry continuity)
    private int lastTagId = -1;
    private double lastTagRange = 0.0;
    private double lastYaw = 0.0;
    private double lastBearing = 0.0;

    // For current frame
    private boolean tagVisible = false;
    private double currentBearing = 0.0;

    // ===== AIM RUMBLE (gamepad1) WHEN ALIGNED TO TAG =====
    // For RED, we want to be aimed slightly RIGHT of tag center -> negative offset.
    private static final double AIM_BEARING_TOL_DEG = 1.5;   // window size
    private static final double AIM_BEARING_OFFSET_DEG = -2.0;  // ~2° to the RIGHT
    private boolean aimRumbleActive = false;

    @Override
    public void init() {
        // ---- Map drive ----
        left_f = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b = hardwareMap.get(DcMotorEx.class, "leftBack");
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
        flywheel_Left = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");
        intake = hardwareMap.get(DcMotor.class, "intake");

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use the voltage-compensated controller
        flywheelController = new VoltageFlywheelController(hardwareMap);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        thirdStage = hardwareMap.get(CRServo.class, "thirdStage");
        thirdStage.setPower(0.0);

        // ===== VISION INIT =====
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
        // Configure camera controls once streaming
        configureVisionIfReady();

        drive();
        runFlywheel();
        runThirdStage();
        runIntake();

        // ===== VISION UPDATE =====
        updateTagTelemetry();

        // ===== FLYWHEEL UPDATE =====
        flywheelController.update();

        // flywheel ready rumble on gamepad2
        updateFlywheelReadyRumble();

        // aim-based rumble on gamepad1 when aligned to red tag with offset
        updateAimRumble();

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
                exposureControl.setExposure(1, TimeUnit.MILLISECONDS); // tune as needed
            }

            GainControl gainControl =
                    (GainControl) visionPortal.getCameraControl(GainControl.class);
            if (gainControl != null) {
                gainControl.setGain(45); // tune as needed
            }

            visionConfigured = true;
            telemetry.addLine("Vision configured: exposure/gain set.");
        }
    }

    // ===================== VISION TELEMETRY & AIM DATA =====================
    private void updateTagTelemetry() {
        if (tagProcessor == null) {
            telemetry.addLine("--- Vision ---");
            telemetry.addLine("Tag processor not initialized");
            tagVisible = false;
            return;
        }

        List<AprilTagDetection> detections = tagProcessor.getDetections();

        telemetry.addLine("--- Vision (AprilTag) ---");

        AprilTagDetection targetTag = null;

        // Find the RED goal tag (ID 24)
        for (AprilTagDetection det : detections) {
            if (det.id == RED_GOAL_TAG_ID) {
                targetTag = det;
                break;
            }
        }

        if (targetTag != null) {
            double range = targetTag.ftcPose.range;    // distance from camera to tag center
            double yaw = targetTag.ftcPose.yaw;      // rotation of tag
            double bearing = targetTag.ftcPose.bearing;  // degrees camera must turn to aim at tag

            lastTagId = targetTag.id;
            lastTagRange = range;
            lastYaw = yaw;
            lastBearing = bearing;

            tagVisible = true;
            currentBearing = bearing;

            telemetry.addData("Tag ID", targetTag.id);
            telemetry.addData("Range (in)", "%.2f", range);
            telemetry.addData("Yaw (deg)", "%.2f", yaw);
            telemetry.addData("Bearing (deg)", "%.2f", bearing);
        } else {
            telemetry.addLine("Red goal tag not detected");
            tagVisible = false;

            if (lastTagId != -1) {
                telemetry.addData("Last Tag ID", lastTagId);
                telemetry.addData("Last Range (in)", "%.2f", lastTagRange);
                telemetry.addData("Last Yaw (deg)", "%.2f", lastYaw);
                telemetry.addData("Last Bearing (deg)", "%.2f", lastBearing);
            }
        }
    }

    // ===================== FLYWHEEL =====================
    private void runFlywheel() {
        // ---- Inputs ----
        boolean x2 = gamepad2.x;
        boolean y2 = gamepad2.y;
        boolean b2 = gamepad2.b;
        boolean lb2 = gamepad2.left_bumper;
        boolean rb2 = gamepad2.right_bumper;
        boolean a2 = gamepad2.a;
        boolean lsb2 = gamepad2.left_stick_button;

        // triggers as digital edges (pressed if > 0.5)
        double ltVal = gamepad2.left_trigger;
        double rtVal = gamepad2.right_trigger;
        boolean lt2 = ltVal > 0.5;
        boolean rt2 = rtVal > 0.5;

        // ---- Presets ----
        // medium preset (3400) on Y
        if (y2 && !prevY2) {
            setFlywheelPreset(PRESET_MED_RPM);
        }
        prevY2 = y2;

        // med-long preset (3250) on right bumper
        if (rb2 && !prevRB2) {
            setFlywheelPreset(PRESET_MED_LONG_RPM);
        }
        prevRB2 = rb2;

        // short-medium preset (3100) on left trigger
        if (lt2 && !prevLT2) {
            setFlywheelPreset(PRESET_SHORT_MED_RPM);
        }
        prevLT2 = lt2;

        // long preset (3300) on right trigger
        if (rt2 && !prevRT2) {
            setFlywheelPreset(PRESET_LONG_RPM);
        }
        prevRT2 = rt2;

        // (Left bumper currently unused; keep edge updated)
        prevLB2 = lb2;

        // A sets a low "stall" preset
        if (a2 && !prevA2) {
            setFlywheelPreset(PRESET_STALL);
        }
        prevA2 = a2;

        // Left stick button fully turns flywheel off
        if (lsb2 && !prevLSB2) {
            flywheelController.turnFlywheelOff();
        }
        prevLSB2 = lsb2;

        // ---- Adjust RPM only when flywheel is ON ----
        // B = increase target, X = decrease target
        if (b2 && !prevB2 && flywheelController.isFlywheelOn()) {
            double newTarget = flywheelController.getTargetRPM() + RPM_STEP;
            flywheelController.setFlywheelTargetRPM(newTarget);
        }

        if (x2 && !prevX2 && flywheelController.isFlywheelOn()) {
            double newTarget = flywheelController.getTargetRPM() - RPM_STEP;
            if (newTarget < MIN_RPM) newTarget = MIN_RPM;
            flywheelController.setFlywheelTargetRPM(newTarget);
        }

        // update edges for X/B after using them
        prevX2 = x2;
        prevB2 = b2;
    }

    // Helper so all presets behave identically
    private void setFlywheelPreset(int rpm) {
        flywheelController.setFlywheelTargetRPM(rpm);
        flywheelController.turnFlywheelOn();
    }

    // ===================== READY-TO-SHOOT RUMBLE LOGIC (gamepad2) =====================
    private void updateFlywheelReadyRumble() {
        double target = flywheelController.getTargetRPM();

        // If flywheel is off or target is zero/invalid, clear state
        if (!flywheelController.isFlywheelOn() || target <= 0.0) {
            flywheelAtSpeed = false;
            flywheelReadyRumbled = false;
            lastTargetRPM = target;
            return;
        }

        // If the target RPM changed, allow a new rumble when we next reach speed
        if (target != lastTargetRPM) {
            flywheelAtSpeed = false;
            flywheelReadyRumbled = false;
        }

        double errL = Math.abs(flywheelController.getErrorRPM_Left());
        double errR = Math.abs(flywheelController.getErrorRPM_Right());
        boolean atSpeedNow = (errL <= RPM_TOLERANCE) && (errR <= RPM_TOLERANCE);

        // Fire rumble once when we newly enter the "ready" band for this target
        if (atSpeedNow && !flywheelAtSpeed && !flywheelReadyRumbled) {
            if (gamepad2 != null) {
                gamepad2.rumbleBlips(1);
            }
            flywheelReadyRumbled = true;
        }

        flywheelAtSpeed = atSpeedNow;
        lastTargetRPM = target;
    }

    // ===================== AIM RUMBLE LOGIC (gamepad1) =====================
    private void updateAimRumble() {
        boolean flywheelActive = flywheelController.isFlywheelOn();

        // For RED: aligned when bearing is close to AIM_BEARING_OFFSET_DEG (~-2°, slightly right)
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
            if (gamepad1 != null) {
                gamepad1.stopRumble();
            }
            aimRumbleActive = false;
        }
    }

    // ===================== DRIVE (WITH DEFENSE MODE) =====================
    private void drive() {
        // Defense mode: when gamepad1 left trigger is held,
        // front wheels go backwards at 0.1, back wheels forwards at 0.1
        boolean defenseMode = gamepad1.left_trigger > 0.5;

        if (defenseMode) {
            left_f.setPower(-0.1);
            right_f.setPower(-0.1);
            left_b.setPower(0.1);
            right_b.setPower(0.1);
            return;
        }

        // Normal mecanum drive
        driverScale = (gamepad1.right_trigger > 0.05) ? 0.25 : 1.0;

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.right_stick_x;
        double rx = -gamepad1.left_stick_x;

        double ys = y * driverScale;
        double xs = x * driverScale;
        double rxs = rx * driverScale;

        double d = Math.max(Math.abs(ys) + Math.abs(xs) + Math.abs(rxs), 1.0);

        right_f.setPower((ys + xs + rxs) / d);
        left_b.setPower((ys - xs + rxs) / d);
        left_f.setPower((ys - xs - rxs) / d);
        right_b.setPower((ys + xs - rxs) / d);
    }

    // ===================== INTAKE =====================
    private void runIntake() {
        boolean x = gamepad1.x;
        if (x && !prevX) intakeOn = !intakeOn;
        prevX = x;

        boolean b = gamepad1.b;

        double intakePower = 0.0;

        if (b) {
            // Hard reverse always full power
            intakePower = -1.0;
        } else if (intakeOn) {
            // Compute average current RPM of both flywheel motors
            double currentRpmLeft = flywheelController.getCurrentRPM_Left();
            double currentRpmRight = flywheelController.getCurrentRPM_Right();
            double avgRpm = (currentRpmLeft + currentRpmRight) / 2.0;

            // If flywheel is spinning "fast" (> 700 rpm), limit intake to 0.5
            // Otherwise use full intake power
            if (avgRpm > INTAKE_RPM_LIMIT_THRESHOLD) {
                intakePower = 0.5;
            } else {
                intakePower = 1.0;
            }
        }

        intake.setPower(intakePower);
    }

    // ===================== THIRD STAGE (CR SERVO) =====================
    private void runThirdStage() {
        // dpad_left toggles LEFT spin on/off
        boolean dl = gamepad2.dpad_left;
        if (dl && !prevDL) thirdDir = (thirdDir == -1) ? 0 : -1;
        prevDL = dl;

        // dpad_right toggles RIGHT spin on/off
        boolean dr = gamepad2.dpad_right;
        if (dr && !prevDR) thirdDir = (thirdDir == +1) ? 0 : +1;
        prevDR = dr;

        double power = (thirdDir == 0) ? 0.0 : (thirdDir < 0 ? -1.0 : 1.0);
        thirdStage.setPower(power);

        telemetry.addData("ThirdStage", thirdDir == 0 ? "STOP" : (thirdDir > 0 ? "RIGHT" : "LEFT"));
        telemetry.addData("ThirdStage Power", "%.2f", power);
    }

    @Override
    public void stop() {
        right_f.setPower(0);
        left_f.setPower(0);
        right_b.setPower(0);
        left_b.setPower(0);

        flywheel_Left.setVelocity(0);
        flywheel_Right.setVelocity(0);
        intake.setPower(0);
        thirdStage.setPower(0.0);

        // Make sure all rumbles are off when OpMode ends
        if (gamepad1 != null) gamepad1.stopRumble();
        if (gamepad2 != null) gamepad2.stopRumble();

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}