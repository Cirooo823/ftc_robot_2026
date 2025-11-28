package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp(name="FlywheelMaxVelocityTest", group="Tuning")
public class FlywheelMaxVelocityTest extends LinearOpMode {

    private DcMotorEx flywheel_Left, flywheel_Right;

    private enum TestMode {
        LEFT,
        RIGHT,
        BOTH
    }

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel_Left  = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel_Right.setDirection(DcMotorEx.Direction.REVERSE);

        // Use WITHOUT_ENCODER for max velocity test
        flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Default: test LEFT
        TestMode mode = TestMode.LEFT;

        // ===== PRE-START MODE SELECTION =====
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x)      mode = TestMode.LEFT;
            else if (gamepad1.b) mode = TestMode.RIGHT;
            else if (gamepad1.y) mode = TestMode.BOTH;

            telemetry.addLine("=== Flywheel Max Velocity Test ===");
            telemetry.addLine("Select test mode before pressing PLAY:");
            telemetry.addLine("  X → Test LEFT motor only");
            telemetry.addLine("  B → Test RIGHT motor only");
            telemetry.addLine("  Y → Test BOTH motors together");
            telemetry.addData("Selected Mode", mode.toString());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Stop both motors initially
        flywheel_Left.setPower(0.0);
        flywheel_Right.setPower(0.0);

        // Apply full power based on selected mode
        switch (mode) {
            case LEFT:
                flywheel_Left.setPower(1.0);
                flywheel_Right.setPower(0.0);
                break;

            case RIGHT:
                flywheel_Left.setPower(0.0);
                flywheel_Right.setPower(1.0);
                break;

            case BOTH:
                flywheel_Left.setPower(1.0);
                flywheel_Right.setPower(1.0);
                break;
        }

        double maxTpsLeft = 0.0;
        double maxTpsRight = 0.0;

        while (opModeIsActive()) {
            double tpsLeft  = Math.abs(flywheel_Left.getVelocity());
            double tpsRight = Math.abs(flywheel_Right.getVelocity());

            if (tpsLeft > maxTpsLeft)   maxTpsLeft = tpsLeft;
            if (tpsRight > maxTpsRight) maxTpsRight = tpsRight;

            telemetry.addLine("=== Running Max Velocity Test ===");
            telemetry.addData("Mode", mode.toString());

            telemetry.addLine("\n--- CURRENT ---");
            telemetry.addData("Left  tps",  tpsLeft);
            telemetry.addData("Right tps", tpsRight);

            telemetry.addLine("\n--- MAX SEEN ---");
            telemetry.addData("LEFT  max tps",  maxTpsLeft);
            telemetry.addData("RIGHT max tps", maxTpsRight);

            if (mode == TestMode.BOTH) {
                double avg = 0.5 * (tpsLeft + tpsRight);
                double avgMax = 0.5 * (maxTpsLeft + maxTpsRight);
                telemetry.addLine("\n--- BOTH MODE STATS ---");
                telemetry.addData("AVG tps (current)", avg);
                telemetry.addData("AVG MAX tps", avgMax);
            }

            telemetry.addLine("\nLet motors spin ~3–5 seconds, then press STOP");
            telemetry.update();
        }

        // Stop motors
        flywheel_Left.setPower(0.0);
        flywheel_Right.setPower(0.0);
    }
}
