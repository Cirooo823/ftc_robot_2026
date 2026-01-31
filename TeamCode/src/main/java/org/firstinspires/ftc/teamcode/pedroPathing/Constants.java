//This is to configure the constants
//follower- how pedro follows path, drivetrain- motor names for mecanum, localizer- pinpoint setup values, path- when path should end




package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.5) //write the robot's mass in the parenthesis, in kg
           .forwardZeroPowerAcceleration(-36.63173124)
          .lateralZeroPowerAcceleration(-78.75452249)
         //  .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
        //   .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0))
        //   .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.0008,0.6,0.0))
        //    .centripetalScaling(0.0005);
;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);




    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(77.51940854)
          .yVelocity(60.40092152)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE) //was reversed
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);






    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.445) //X distance from center of the one that rolls forwards/back, in inches
            .strafePodX(-6.496)//Y distance from center of the one that rolls left/right, in inches
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")// change name to what we named the pinpoint in the controller
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    //
    // TO REVERSE ENCODER:
    //.forwardEncoderDirection(Encoder.REVERSE)
    /// / and/or:
    //.strafeEncoderDirection(Encoder.REVERSE)






    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
