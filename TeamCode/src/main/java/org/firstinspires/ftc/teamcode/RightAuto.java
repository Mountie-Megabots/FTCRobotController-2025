package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedro.Constants;

@Autonomous(name = "Right Auto", group = "Comp")
public class RightAuto extends OpMode {
    Follower follower;
    public Path Path1;

    // Functions Motors


    @Override
    public void init() {
        //Configure Functions Motors


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        Path1 = new Path(new BezierLine(new Pose(0, 0), new Pose(30, -17)));
        Path1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));


    }

    @Override
    public void start() {
        follower.followPath(Path1, false);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }


}
