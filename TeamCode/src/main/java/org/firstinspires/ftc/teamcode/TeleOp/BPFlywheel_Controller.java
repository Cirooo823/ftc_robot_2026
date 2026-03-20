package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class BPFlywheel_Controller extends OpMode {

    public DcMotorEx flywheel_Left;

    public double highVelocity = 2700;
    public double lowVelocity = 3300;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;


    double[] stepSizes = {10.0,1.0,0.1,0.001,0.0001};

    private static final DcMotorSimple.Direction LEFT_DIR = DcMotorSimple.Direction.REVERSE;
    int stepIndex = 1;


    @Override
    public void init() {
        flywheel_Left = hardwareMap.get(DcMotorEx.class, "flywheel_Left");

        flywheel_Left.setDirection(LEFT_DIR);

        flywheel_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);

        flywheel_Left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("init complete");

    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity;}
        }if
        (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];

        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);

        flywheel_Left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel_Left.setVelocity(curTargetVelocity);

        double curVelocity = flywheel_Left.getVelocity();
        double error = curTargetVelocity - curVelocity;
        telemetry.addLine();
        telemetry.addData("Target Velocity",curTargetVelocity);
        telemetry.addData("Actual Velocity", curVelocity);
        telemetry.addData("Error", error);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("P",P);
        telemetry.addData("F",F);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
    }
}
