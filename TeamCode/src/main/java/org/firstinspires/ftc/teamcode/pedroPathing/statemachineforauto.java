package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// IMPORT YOUR CUSTOM CONTROLLER
// Make sure this matches where you actually saved the controller file!
import org.firstinspires.ftc.teamcode.TeleOp.VoltageFlywheelController;

public class statemachineforauto {

    // --- HARDWARE ---
    // We do NOT define DcMotorEx here. The Controller handles that.
    private VoltageFlywheelController flywheelController;
    private CRServo thirdStage;

    // --- STATE MACHINE ---
    private ElapsedTime stateTimer = new ElapsedTime();

    public enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_GATE
    }
    private FlywheelState flywheelState = FlywheelState.IDLE;

    // --- CONSTANTS ---
    // CRServo uses Power (-1.0 to 1.0), not RPM.
    private final double THIRD_STAGE_PUSH_POWER = 1.0;
    private final double THIRD_STAGE_STOP_POWER = 0.0;

    // Timing
    private final double SERVO_PUSH_TIME = 0.3;  // Time to push ring in
    private final double SERVO_RESET_TIME = 0.3; // Time to pull back/wait
    private final double MAX_SPINUP_TIME = 2.0;  // Safety timeout

    // Shooting Variables
    private int shotsRemaining = 0;
    private final double TARGET_RPM = 3300; // High Goal Speed
    private final double ACCEPTABLE_ERROR = 100; // RPM tolerance

    // --- INIT ---
    public void init(HardwareMap hwMap) {
        // 1. Initialize the Servo
        thirdStage = hwMap.get(CRServo.class, "thirdstage");
        thirdStage.setPower(THIRD_STAGE_STOP_POWER);

        // 2. Initialize YOUR Custom Controller
        // This connects to the motors and sets up the PIDF/Voltage logic
        flywheelController = new VoltageFlywheelController(hwMap);
        flywheelController.turnFlywheelOff();

        flywheelState = FlywheelState.IDLE;
    }

    // --- UPDATE LOOP (The Brain) ---
    public void update() {

        // CRITICAL: Keep the PID loop running every single cycle
        flywheelController.update();

        // State Machine Logic
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    // Turn on the flywheel
                    flywheelController.setFlywheelTargetRPM(TARGET_RPM);
                    flywheelController.turnFlywheelOn();

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                } else {
                    // Turn everything off
                    flywheelController.turnFlywheelOff();
                    thirdStage.setPower(THIRD_STAGE_STOP_POWER);
                }
                break;

            case SPIN_UP:
                // Check if we are at speed OR if we timed out
                double currentRPM = flywheelController.getCurrentRPM_Average();
                double error = Math.abs(TARGET_RPM - currentRPM);

                if (error < ACCEPTABLE_ERROR || stateTimer.seconds() > MAX_SPINUP_TIME) {
                    // Speed is good! Push the ring.
                    thirdStage.setPower(THIRD_STAGE_PUSH_POWER);
                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;

            case LAUNCH:
                // Wait for servo to push ring into wheel
                if (stateTimer.seconds() > SERVO_PUSH_TIME) {
                    shotsRemaining--; // Count the shot
                    thirdStage.setPower(THIRD_STAGE_STOP_POWER); // Stop pushing
                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET_GATE;
                }
                break;

            case RESET_GATE:
                // Wait for mechanism to reset before next shot
                if (stateTimer.seconds() > SERVO_RESET_TIME) {
                    if (shotsRemaining > 0) {
                        // Go back to Spin Up (Motor is likely still spinning, just needs to check RPM)
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    } else {
                        // All shots done
                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }
    }

    // --- COMMANDS ---
    public void fireShots(int numberOfShots) {
        // Only accept command if not already busy (or add to queue if you prefer)
        if (numberOfShots > 0) {
            this.shotsRemaining = numberOfShots;
        }
    }

    // --- STATUS ---
    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }
}


