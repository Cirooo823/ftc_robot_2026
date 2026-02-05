package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.VoltageFlywheelController;

public class logic6far {
    public boolean isBarrierOpen() {
        // Check if the servo is currently at the open position
        return Math.abs(barrierServo.getPosition() - BARRIER_OPEN_POS) < 0.1;
    }
    private boolean keepFlywheelRunning = false; //NEW
    private DcMotorEx flywheel_Left, flywheel_Right;
    private DcMotorEx intake;
    private Servo barrierServo;
    private final double BARRIER_CLOSED_POS = 0.6;
    private final double BARRIER_OPEN_POS   = 0.0;
    private double BARRIER_RELEASE_TIME = 1.9; //time for shots before gate closes again
    private double BARRIER_RESET_TIME = 0.1; //time it takes t close gate
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
    //  private double THIRD_STAGE_ON_RPM = -200;
    // private double THIRD_STAGE_OFF_RPM = 0;
    // private double THIRD_STAGE_ON_TIME = 3; //was 5
    // private double THIRD_STAGE_OFF_TIME = 0.2;

    //--------FLYWHEEL CONSTANTS-----------
    private int shotsRemaining = 0;
    private double flywheelvelocity = 0;
    private double MIN_FLYWHEEL_RPM = 2970;
    private double TARGET_FLYWHEEL_RPM = 2980;
    private double FLYWHEEL_MAX_SPINUP_TIME = 2.7;

    public void init(HardwareMap hwMap) {
        barrierServo = hwMap.get(Servo.class, "barrierServo");
        flywheelController = new VoltageFlywheelController(hwMap);

        // flywheel_Left = hwMap.get(DcMotorEx.class, "flywheel_Left");
        //flywheel_Right = hwMap.get(DcMotorEx.class, "flywheel_Right");

        flywheelState = FlywheelState.IDLE;
        // flywheel_Left.setPower(0);
        // flywheel_Right.setPower(0);
        barrierServo.setPosition(BARRIER_CLOSED_POS);//barrier is going to start closed
    }

    //this stuff is new i added it so the flywheel is always on
    public void setFlywheelKeepAlive(boolean keepAlive) {
        this.keepFlywheelRunning = keepAlive;
        if (keepAlive) {
            flywheelController.setFlywheelTargetRPM(TARGET_FLYWHEEL_RPM);
            flywheelController.turnFlywheelOn();
        } else {
            flywheelController.setFlywheelTargetRPM(0);
            flywheelController.turnFlywheelOff();
        }
    }

    public void update() {

        flywheelController.update();

        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    barrierServo.setPosition(BARRIER_CLOSED_POS);
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
                    barrierServo.setPosition(BARRIER_OPEN_POS); //barrier opens
                    stateTimer.reset();

                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > BARRIER_RELEASE_TIME) {
                    shotsRemaining=0; //increment by -3
                    barrierServo.setPosition(BARRIER_CLOSED_POS);//close barrier
                    stateTimer.reset();

                    flywheelState = FlywheelState.RESET_GATE;
                }
                break;
            case RESET_GATE:
                if (stateTimer.seconds() > BARRIER_RESET_TIME) {
                    if (!keepFlywheelRunning) {
                        flywheelController.setFlywheelTargetRPM(0);
                        flywheelController.turnFlywheelOff();
                    }
                    flywheelState = FlywheelState.IDLE;
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
