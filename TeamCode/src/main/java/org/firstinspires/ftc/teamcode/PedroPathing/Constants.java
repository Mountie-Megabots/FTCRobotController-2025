package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static final class paths {
        public static int farShotHeading = -20;
        public static final class GrabConst {
            public static Pose GPPStart = new Pose(52.000, 84.000, Math.toRadians(180));
            public static BezierLine GPP = new BezierLine(new Pose(52.000, 84.000, Math.toRadians(180)), new Pose(17.5, 84.000, Math.toRadians(180)));
            public static BezierLine GPPRev = new BezierLine(new Pose(17.5000, 84.000, Math.toRadians(180)), new Pose(52.000, 84.000, Math.toRadians(180)));
            public static Pose PGPStart = new Pose(48.000, 60.000, Math.toRadians(180));
            public static BezierLine PGP = new BezierLine(new Pose(48.000, 60.000, Math.toRadians(180)), new Pose(17.500, 60.000, Math.toRadians(180)));
            public static Pose PPGStart = new Pose(48.000, 36.000, Math.toRadians(180));
            public static BezierLine PPG = new BezierLine(new Pose(48.000, 36.000, Math.toRadians(180)), new Pose(17.500, 36.000, Math.toRadians(180)));
            //degrees for all: 180
        }
        public static final class FarScoreConst {
            public static Pose farStart = new Pose(56.299065420560744, 8.074766355140188,Math.toRadians(90));
            public static Pose farScore = new Pose (60.5607476635514, 12.785046728971956, Math.toRadians(113));
        }
        public static final class CloseScoreConst {
            //right side
            //degrees: 145
            public static Pose backupRightEnd = new Pose(31.402, 129.196, Math.toRadians(160));
            public static Pose rightStart = new Pose(27.364, 132.561, Math.toRadians(145));

            public static BezierLine backupRight = new BezierLine(rightStart, backupRightEnd);

            //center
            public static Pose centerStart = new Pose(20.5, 123, Math.toRadians(140));
            public static Pose centerEnd = new Pose(48, 95.5, Math.toRadians(135));
            public static BezierLine backupCenter = new BezierLine(centerStart, centerEnd);
            public static Pose launchLeave = new Pose(25, 95, Math.toRadians(180));

            public static Pose curveControlPoint = new Pose(95, 95.776);
        }

    }


    public static FollowerConstants followerConstants = new FollowerConstants()
            .lateralZeroPowerAcceleration(-79.2023)
            .forwardZeroPowerAcceleration(-52.723)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0.00005, 0.025, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0.001, 0.1, 0.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1, 0.5, 0.075, 0, 0.04))
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .centripetalScaling(0)
            .mass(10.7);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.7, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(57.67577)
            .yVelocity(46.208)
            .rightFrontMotorName("Front-Right")
            .rightRearMotorName("Back-Right")
            .leftRearMotorName("Back-Left")
            .leftFrontMotorName("Front-Left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            ;


    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
//            .forwardTicksToInches(.001989436789)
//            .strafeTicksToInches(.001989436789)
//            .turnTicksToInches(.001989436789)
            .leftPodY(4.0625)
            .rightPodY(-4.0625)
            .strafePodX(-5.125)
            .leftEncoder_HardwareMapName("Front-Left")
            .rightEncoder_HardwareMapName("Back-Right")
            .strafeEncoder_HardwareMapName("Front-Right")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.002984)
            .strafeTicksToInches(0.003)
            .turnTicksToInches(0.002923);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
