package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="newTeleOp", group="TeleOp")
public class newTeleOp extends OpMode {
    private DcMotorEx right_b, left_f, right_f, left_b;
    private DcMotor flywheel, intake;


    private boolean flywheelOn = false;
    private boolean prevA = false;

    private double driverScale = 1.0;

    @Override
    public void init() {
        left_f  = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b  = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

        left_f.setDirection(DcMotor.Direction.REVERSE);
        right_b.setDirection(DcMotor.Direction.REVERSE);

        right_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mechanisms
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
//region Loop
    @Override
    public void loop() {
       drive();
       runFlywheel();
       runIntake();
    }
//endregion
//region Class Methods
    private void drive(){
        driverScale = gamepad1.left_bumper ? 0.25 : 1.0;

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

    private void runFlywheel(){
        boolean aPressed = gamepad1.a;

        // Detect rising edge (button pressed, not held)
        if (aPressed && !prevA) {
            flywheelOn = !flywheelOn; // toggle state
        }
        prevA = aPressed;

        flywheel.setPower(flywheelOn ? 1.0 : 0.0);
    }

    private void runIntake(){
        if (gamepad1.x) {
            intake.setPower(1.0);     // intake in
        } else if (gamepad1.b) {
            intake.setPower(-1.0);    // reverse
        } else {
            intake.setPower(0.0);     // stop
        }
    }
//endregion

    @Override
    public void stop() {
        // Stop all drive motors
        right_f.setPower(0);
        left_f.setPower(0);
        right_b.setPower(0);
        left_b.setPower(0);
        flywheel.setPower(0);
        intake.setPower(0);
    }
}
