package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Test TeleOP", group="TeleOp")
public class TestTeleOP extends OpMode {

    public DcMotorEx right_b;
    public DcMotorEx left_f;
    public DcMotorEx right_f;
    public DcMotorEx left_b;

    public double drivePowerScale=1;

    @Override
    public void init() {
        left_f = hardwareMap.get(DcMotorEx.class, "leftFront");
        right_f = hardwareMap.get(DcMotorEx.class, "rightFront");
        left_b = hardwareMap.get(DcMotorEx.class, "leftBack");
        right_b = hardwareMap.get(DcMotorEx.class, "rightBack");

        left_f.setDirection((DcMotorEx.Direction.REVERSE));
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double x = -gamepad1.right_stick_x; //flipped the sign to reverse strafing (undid it again after advik changed wheels)
        double y = -gamepad1.left_stick_y;
        double rx = -gamepad1.left_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        right_f.setPower(((y + x + rx) / denominator) * drivePowerScale);
        left_b.setPower(((y - x + rx) / denominator) * drivePowerScale);
        left_f.setPower(((y - x - rx) / denominator) * drivePowerScale);
        right_b.setPower(((y + x - rx) / denominator) * drivePowerScale);
    }
}
