package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class TestAuto extends OpMode {
    Follower follower;
    public PathChain Path1;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.206, 122.656, Math.toRadians(323)));
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(new Pose(20.206, 122.656),
                        new Pose(72.285, 72.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-37),
                         Math.toRadians(180))
                .build();

    }

    @Override
    public void start() {
        follower.followPath(Path1, true);
    }

    @Override
    public void loop() {
        follower.update();

    }


}
