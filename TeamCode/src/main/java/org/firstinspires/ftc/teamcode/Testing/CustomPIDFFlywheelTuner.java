package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="Custom PIDF Flywheel Tuner", group="Calibration")
public class CustomPIDFFlywheelTuner extends LinearOpMode {

    private DcMotorEx flywheel_Left;
    private DcMotorEx flywheel_Right;
    private ElapsedTime pidTimer = new ElapsedTime(); // Timer to calculate dt for PID

    // --- MOTOR SPECIFICS (GoBilda 5203 1:1 Ratio) ---
    // VERIFIED: 5203 Series Yellow Jacket Motor (1:1 Ratio)
    // Encoder PPR (Pulses Per Revolution) at Output Shaft: 28 PPR
    // TICKS_PER_REVOLUTION (of output shaft) = 28 * 1 = 28.0
    private final double TICKS_PER_REVOLUTION = 28.0;

    // --- HARDWARE CONSTANTS (Confirmed by diagnostics) ---
    // These directions make the motors spin physically in the "Launch Direction"
    // and cooperate on the common shaft. getVelocity() will report NEGATIVE.
    private final DcMotorSimple.Direction LEFT_MOTOR_FINAL_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private final DcMotorSimple.Direction RIGHT_MOTOR_FINAL_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // --- TUNING VARIABLES (Use these to adjust with Gamepad) ---
    public static double targetRPM = 3000; // Initial target RPM
    public static boolean flywheelOn = false; // Toggle state

    // --- CUSTOM PIDF COEFFICIENTS ---
    // kF: Feedforward. Directly relates desired velocity to base power.
    //     Estimate: Max Power (1.0) / Max Achievable TPS.
    //     Your measured Max TPS = 2550
    //     So, a strong starting kF estimate is 1.0 / 2550.0 = 0.00039215
    public static double kF = 1.0 / 2550.0; // Start with this estimate (~0.000392)

    // PID terms act on the error. Start very small or zero.
    public static double kP = 0.0000;  // Start at 0.0, tune after F
    public static double kI = 0.0;     // Keep at 0.0 for initial flywheel tuning
    public static double kD = 0.0;     // Keep at 0.0 for initial flywheel tuning

    // --- PID State Variables (Internal to the controller) ---
    double lastErrorL = 0, lastErrorR = 0;
    double integralL = 0, integralR = 0;

    // --- Gamepad Debounce variables ---
    private boolean debounceA, debounceUp, debounceDown, debounceFUp, debounceFDown, debouncePUp, debouncePDown;


    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware Initialization ---
        flywheel_Left = hardwareMap.get(DcMotorEx.class, "flywheel_Left");
        flywheel_Right = hardwareMap.get(DcMotorEx.class, "flywheel_Right");

        flywheel_Left.setDirection(LEFT_MOTOR_FINAL_DIRECTION);
        flywheel_Right.setDirection(RIGHT_MOTOR_FINAL_DIRECTION);

        // IMPORTANT: For custom PID, we must use RUN_WITHOUT_ENCODER
        // We are calculating and setting raw power ourselves.
        flywheel_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("--- Custom PIDF Flywheel Tuner ---");
        telemetry.addData("Left Motor Dir", LEFT_MOTOR_FINAL_DIRECTION.toString());
        telemetry.addData("Right Motor Dir", RIGHT_MOTOR_FINAL_DIRECTION.toString());
        telemetry.addData("Ticks Per Rev (Output)", TICKS_PER_REVOLUTION);
        telemetry.addLine("Gamepad1: A=Toggle, DpadU/D=TargetRPM");
        telemetry.addLine("Gamepad2: DpadU/D=kF, Left/Right=kP");
        telemetry.addLine("Press START to begin tuning.");
        telemetry.update();

        waitForStart();
        pidTimer.reset(); // Start the timer for dt calculation

        while (opModeIsActive()) {
            // 1. Handle User Input for Tuning
            handleGamepadInput();

            // 2. Calculate Target Ticks Per Second
            double targetTPS = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;

            // 3. Run the Custom PID Loop
            if (flywheelOn) {
                // Calculate power for each motor
                // Pass true for left motor, false for right to track PID state separately
                double powerL = calculatePIDF(flywheel_Left, targetTPS, lastErrorL, integralL, true);
                double powerR = calculatePIDF(flywheel_Right, targetTPS, lastErrorR, integralR, false);

                flywheel_Left.setPower(powerL);
                flywheel_Right.setPower(powerR);
            } else {
                // If flywheel is off, set power to 0 and reset PID state
                flywheel_Left.setPower(0);
                flywheel_Right.setPower(0);
                resetPIDState();
            }

            // 4. Telemetry Updates
            // Negate raw velocity for human-readable positive RPMs
            double currentTPS_L = -flywheel_Left.getVelocity();
            double currentRPM_L = (currentTPS_L / TICKS_PER_REVOLUTION) * 60.0;
            double currentTPS_R = -flywheel_Right.getVelocity();
            double currentRPM_R = (currentTPS_R / TICKS_PER_REVOLUTION) * 60.0;

            telemetry.addData("Flywheel State", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", "%.2f", targetRPM);
            telemetry.addData("Actual RPM (L)", "%.2f", currentRPM_L);
            telemetry.addData("Actual RPM (R)", "%.2f", currentRPM_R);
            telemetry.addData("Error RPM (L)", "%.2f", targetRPM - currentRPM_L);
            telemetry.addData("Error RPM (R)", "%.2f", targetRPM - currentRPM_R);
            telemetry.addLine("--- Coefficients ---");
            telemetry.addData("kF (Feedforward)", "%.6f", kF);
            telemetry.addData("kP (Proportional)", "%.6f", kP);
            // Uncomment kI/kD if you start tuning them
            // telemetry.addData("kI (Integral)", "%.6f", kI);
            // telemetry.addData("kD (Derivative)", "%.6f", kD);
            telemetry.update();
        }

        // --- Stop motors on OpMode End ---
        flywheel_Left.setPower(0);
        flywheel_Right.setPower(0);
    }

    // ==================================================================
    // THE CUSTOM PID CALCULATION METHOD
    // This is the heart of your custom controller.
    // ==================================================================
    private double calculatePIDF(DcMotorEx motor, double targetTPS, double lastError, double integralSum, boolean isLeftMotor) {
        // Calculate time elapsed since last loop iteration
        // This ensures PID terms are time-dependent, not just loop-dependent
        double dt = pidTimer.seconds();
        pidTimer.reset(); // Reset timer for the next iteration

        // 1. Get current velocity and NEGATE it to work with positive numbers
        double currentTPS = -motor.getVelocity();

        // 2. Calculate Error (how far off we are from the target)
        double error = targetTPS - currentTPS;

        // 3. Feedforward (kF): The base power needed to reach the target velocity
        //    This is proportional to the target velocity itself.
        double feedforward = targetTPS * kF;

        // 4. Proportional Term (kP): Corrects error based on the current difference
        double proportional = error * kP;

        // 5. Integral Term (kI): Corrects for steady-state errors over time
        //    Only accumulate integral if error is significant and motor is not saturated
        if (Math.abs(error) > 50 && Math.abs(currentTPS) < targetTPS * 1.1) { // Example conditions
            integralSum += error * dt;
        }
        // Clamp integral term contribution to prevent windup (integral growing too large)
        double maxIntegralContribution = 0.15; // Max 15% power from integral term
        double integralTerm = Range.clip(integralSum * kI, -maxIntegralContribution, maxIntegralContribution);

        // 6. Derivative Term (kD): Corrects for the rate of change of error
        //    Dampens oscillations and reacts to sudden changes.
        double derivative = (error - lastError) / dt;
        double derivativeTerm = derivative * kD;

        // Save PID state for the next iteration of this specific motor
        if (isLeftMotor) {
            this.lastErrorL = error;
            this.integralL = integralSum;
        } else {
            this.lastErrorR = error;
            this.integralR = integralSum;
        }

        // 7. Sum all components to get the total power output
        double totalPower = feedforward + proportional + integralTerm + derivativeTerm;

        // 8. Clip the total power to the valid motor range [0.0, 1.0] (flywheel only goes one way)
        return Range.clip(totalPower, 0.0, 1.0);
    }

    // --- Reset PID State Variables when flywheel is turned off ---
    private void resetPIDState() {
        lastErrorL = 0; lastErrorR = 0;
        integralL = 0; integralR = 0;
        pidTimer.reset(); // Reset timer to prevent large dt on next start
    }

    // ==================================================================
    // GAMEPAD INPUT HANDLING (For live tuning)
    // ==================================================================
    private void handleGamepadInput() {
        // Gamepad1 A: Toggle flywheel ON/OFF
        if (gamepad1.a && !debounceA) { flywheelOn = !flywheelOn; debounceA = true; }
        else if (!gamepad1.a) { debounceA = false; }

        // Gamepad1 Dpad Up/Down: Adjust Target RPM
        if (gamepad1.dpad_up && !debounceUp) { targetRPM += 100; debounceUp = true; }
        else if (!gamepad1.dpad_up) { debounceUp = false; }
        if (gamepad1.dpad_down && !debounceDown) { targetRPM -= 100; if(targetRPM < 0) targetRPM = 0; debounceDown = true; }
        else if (!gamepad1.dpad_down) { debounceDown = false; }

        // Gamepad2 Dpad Up/Down: Adjust kF (Feedforward) - VERY SENSITIVE
        if (gamepad2.dpad_up && !debounceFUp) { kF += 0.000001; debounceFUp = true; } // Smaller step for kF
        else if (!gamepad2.dpad_up) { debounceFUp = false; }
        if (gamepad2.dpad_down && !debounceFDown) { kF -= 0.000001; if(kF<0) kF=0; debounceFDown = true; }
        else if (!gamepad2.dpad_down) { debounceFDown = false; }

        // Gamepad2 Left/Right Bumper: Adjust kP (Proportional)
        if (gamepad2.right_bumper && !debouncePUp) { kP += 0.00001; debouncePUp = true; } // Smaller step for kP
        else if (!gamepad2.right_bumper) { debouncePUp = false; }
        if (gamepad2.left_bumper && !debouncePDown) { kP -= 0.00001; if(kP<0) kP=0; debouncePDown = true; }
        else if (!gamepad2.left_bumper) { debouncePDown = false; }

        // TODO: Add controls for kI and kD if you choose to tune them later
        // Example for kI: gamepad2.x, gamepad2.b
        // Example for kD: gamepad2.y, gamepad2.a
    }
}

