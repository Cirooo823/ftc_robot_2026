package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.VoltageFlywheelController;

public class FlywheelLogic {
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotor intake;
    private CRServo   thirdStage;
    private VoltageFlywheelController flywheelController;
    private ElapsedTime stateTimer = new ElapsedTime();
    private enum FlywheelState {

        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_GATE
    }

    private FlywheelState flywheelState;

    //------THIRD STAGE CONSTANTS-------
    private double THIRD_STAGE_ON_RPM = -200;
    private double THIRD_STAGE_OFF_RPM = 0;
    private double THIRD_STAGE_ON_TIME = 5;
    private double THIRD_STAGE_OFF_TIME = 0.2;

    //--------FLYWHEEL CONSTANTS-----------
    private int shotsRemaining = 0;
    private double flywheelvelocity = 0;
    private double MIN_FLYWHEEL_RPM = 3100;
    private double TARGET_FLYWHEEL_RPM = 3200;
    private double FLYWHEEL_MAX_SPINUP_TIME = 5;

    public void init(HardwareMap hwMap) {
thirdStage = hwMap.get(CRServo.class, "thirdStage");
        flywheelController = new VoltageFlywheelController(hwMap);

     // flywheel_Left = hwMap.get(DcMotorEx.class, "flywheel_Left");
      //flywheel_Right = hwMap.get(DcMotorEx.class, "flywheel_Right");

      flywheelState = FlywheelState.IDLE;
     // flywheel_Left.setPower(0);
       // flywheel_Right.setPower(0);
        thirdStage.setPower(THIRD_STAGE_OFF_RPM);

    }

public void update() {

    flywheelController.update();

    switch (flywheelState) {
        case IDLE:
            if (shotsRemaining > 0) {
                thirdStage.setPower(THIRD_STAGE_OFF_RPM);
                flywheelController.setFlywheelTargetRPM(TARGET_FLYWHEEL_RPM);
                flywheelController.turnFlywheelOn();
               // flywheel_Left.setVelocity(TARGET_FLYWHEEL_RPM);
                //flywheel_Right.setVelocity(TARGET_FLYWHEEL_RPM);

                stateTimer.reset();
                flywheelState = FlywheelState.SPIN_UP;
            }
            break;
        case SPIN_UP:
            double leftRPM  = flywheelController.getCurrentRPM_Left();
            double rightRPM = flywheelController.getCurrentRPM_Right();
            double avgRPM = (leftRPM + rightRPM) / 2.0;

            if (avgRPM > MIN_FLYWHEEL_RPM || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                thirdStage.setPower(THIRD_STAGE_ON_RPM);
                stateTimer.reset();

                flywheelState = FlywheelState.LAUNCH;
            }
            break;
        case LAUNCH:
            if (stateTimer.seconds() > THIRD_STAGE_ON_TIME) {
                shotsRemaining-=3; //increment by -3
                thirdStage.setPower(THIRD_STAGE_OFF_RPM);
                stateTimer.reset();

                flywheelState = FlywheelState.RESET_GATE;
            }
            break;
        case RESET_GATE:
            if (stateTimer.seconds() > THIRD_STAGE_OFF_TIME) {
                if (shotsRemaining > 0) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                } else {

                    flywheelController.setFlywheelTargetRPM(0);
                    flywheelController.turnFlywheelOff();
                    // flywheel_Left.setVelocity(0);//stop flywheel
                    //flywheel_Right.setVelocity(0);//stop flywheel
                    flywheelState = FlywheelState.IDLE;
                }
            }
            break;
    }
}

public void fireShots (int numberOfShots) {
        if(flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
}

public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
}

}
