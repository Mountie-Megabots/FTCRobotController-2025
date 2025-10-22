package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .lateralZeroPowerAcceleration(-66.8071)
        .forwardZeroPowerAcceleration(-38.0746)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0.0001, 0.007, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(4, 0, 0.25, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05, 0, 0.0000001, 0.01, 0.6))
            .centripetalScaling(0.0005)
            .mass(25);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 0.01);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(47.2831)
            .yVelocity(32.0561)
            .rightFrontMotorName("Front-Right")
            .rightRearMotorName("Back-Right")
            .leftRearMotorName("Back-Left")
            .leftFrontMotorName("Front-Left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);


    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
//            .forwardTicksToInches(.001989436789)
//            .strafeTicksToInches(.001989436789)
//            .turnTicksToInches(.001989436789)
            .leftPodY(2.6)
            .rightPodY(-2.8)
            .strafePodX(-3.4)
            .leftEncoder_HardwareMapName("Front-Left")
            .rightEncoder_HardwareMapName("Back-Right")
            .strafeEncoder_HardwareMapName("Front-Right")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.002946)
            .strafeTicksToInches(0.003008)
            .turnTicksToInches(0.0019425);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
