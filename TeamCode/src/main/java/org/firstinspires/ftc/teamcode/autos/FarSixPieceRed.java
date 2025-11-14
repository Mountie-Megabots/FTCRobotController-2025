package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ShooterSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarSixPieceRed", group = "Blue")
public class FarSixPieceRed extends OpMode {
    Follower follower;
    ShooterSystem shooter;
    public Path path1, path2, path3, path4;
    public PathChain pickupChain;
    ElapsedTime timer;

    boolean initVar = false;
    enum State{
        firstPath,
        shoot1,
        toPickup,
        shoot2,
    }
    private State pathState;

    @Override
    public void init() {
        timer = new ElapsedTime();
        pathState = State.firstPath;
        shooter = new ShooterSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.paths.FarScoreConst.farStart.mirror());
        //insert bezier line or curve
        path1 = new Path(new BezierLine(Constants.paths.FarScoreConst.farStart.mirror(), Constants.paths.FarScoreConst.farScore.mirror()));
        //put in paths in chain
        path1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(75));
        path2 = new Path(new BezierLine(Constants.paths.FarScoreConst.farScore.mirror(), Constants.paths.GrabConst.PPGStart.mirror()));
        path2.setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(0));
        path3 = new Path(new BezierLine(Constants.paths.GrabConst.PPG.getFirstControlPoint().mirror(), Constants.paths.GrabConst.PPG.getLastControlPoint().mirror()));
        path3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        path4 = new Path(new BezierLine(Constants.paths.GrabConst.PPG.getLastControlPoint().mirror(), Constants.paths.FarScoreConst.farScore.mirror()));
        path4.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(75));
        pickupChain = new PathChain(path2, path3, path4);
    }

    private void runPath() {
        switch (pathState) {
            case firstPath:
                follower.followPath(path1, true);
                pathState = State.shoot1;
                break;

            case shoot1:
                //ready shooter
                shooter.setShooterFast();

                if (!follower.isBusy()) {
                    //fire
                    shooter.nextState(true);
                    if (timer.seconds() > 5) {
                        pathState = State.toPickup;
                        shooter.setStopState(true);
                        shooter.nextState(false);
                    }
                } else {
                    timer.reset();
                }

                break;

            case toPickup:
                follower.followPath(pickupChain);
                pathState = State.shoot2;
                break;

            case shoot2:
                //Use if grabbing pieces

                if (follower.getCurrentPath() == path3 && follower.getPathCompletion() > 0.05) {
                    shooter.setStopState(true);
                    shooter.nextState(false);
                    follower.setMaxPower(0.25);
                }

                if (follower.getCurrentPath() == path3 && follower.getPathCompletion() > 0.85) {
                    follower.setMaxPower(1);
                }

                if (follower.getCurrentPath() == path4 && !initVar) {
                    initVar = true;
                    shooter.setStopState(false);
                }


                if (!follower.isBusy()) {
                    shooter.nextState(true);
                    if (timer.seconds() > 5) {
                        shooter.setStopState(true);
                        initVar = false;
                    }
                } else {
                    timer.reset();
                }

        }
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        runPath();
        follower.update();
        shooter.functions();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        shooter.pushTelemetry(telemetry);
        telemetry.update();
    }


}
