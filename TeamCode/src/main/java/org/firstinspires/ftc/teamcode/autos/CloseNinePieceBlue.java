package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ShooterSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "CloseNinePieceBlue", group = "UNFINSHED")
public class CloseNinePieceBlue extends OpMode {
    Follower follower;
    ShooterSystem shooter;
    private Path backupShoot, path2, path3, path4, path5, path6, path7;
    private PathChain pickupChain, pickupChain2;
    ElapsedTime timer;

    boolean initVar = false;
    enum State{
        firstPath,
        shoot1,
        toPickup,
        shoot2,
        toPickupTwo,
        shoot3
    }
    private State pathState;

    @Override
    public void init() {
        timer = new ElapsedTime();
        pathState = State.firstPath;
        shooter = new ShooterSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.paths.CloseScoreConst.centerStart);

        backupShoot = new Path(Constants.paths.CloseScoreConst.backupCenter);
        backupShoot.setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135));

        path2 = new Path(new BezierLine(Constants.paths.CloseScoreConst.centerEnd, Constants.paths.GrabConst.GPPStart));
        path2.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180));

        path3 = new Path(Constants.paths.GrabConst.GPP);

        path4 = new Path(new BezierLine(Constants.paths.GrabConst.GPP.getLastControlPoint(), Constants.paths.CloseScoreConst.centerEnd));
        path4.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135));

        pickupChain = new PathChain(path2, path3, path4);

        path5 = new Path(new BezierLine(Constants.paths.CloseScoreConst.centerEnd, Constants.paths.GrabConst.PGPStart));
        path5.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180));

        path6 = new Path(Constants.paths.GrabConst.PGP);

        path7 = new Path(new BezierLine(Constants.paths.GrabConst.PGP.getLastControlPoint(), Constants.paths.CloseScoreConst.centerEnd));
        path7.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135));

        pickupChain2 = new PathChain(path5, path6, path7);
    }

    private void runPath() {
        switch (pathState) {
            case firstPath:
                follower.followPath(backupShoot, true);
                pathState = State.shoot1;
                break;

            case shoot1:
                shooter.setShooterSlow();
                if (!follower.isBusy()) {
                    shooter.nextState(true);
                    if (timer.seconds() > 5) {
                        pathState = State.toPickup;
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
                //path 3 = pickup path
                if (follower.getCurrentPath() == path3 && follower.getPathCompletion() > 0.1) {
                    shooter.setStopState(true);
                    shooter.nextState(false);
                    follower.setMaxPower(0.3);
                }

                if (follower.getCurrentPath() == path3 && follower.getPathCompletion() > 0.85) {
                    follower.setMaxPower(1);
                }

                //path 4 = path after path 3 - refer to path 3
                if (follower.getCurrentPath() == path4 && !initVar) {
                    initVar = true;
                    shooter.setStopState(false);
                }

                if (!follower.isBusy()) {
                    shooter.nextState(true);
                    if (timer.seconds() > 5) {
                        shooter.setStopState(true);
                        initVar = false;
                        pathState = State.toPickupTwo;
                    }
                } else {
                    timer.reset();
                }

            case toPickupTwo:
                follower.followPath(pickupChain2);
                pathState = State.shoot3;
                break;

            case shoot3:
                //path 7 = pickup path
                if (follower.getCurrentPath() == path6 && follower.getPathCompletion() > 0.1) {
                    shooter.setStopState(true);
                    shooter.nextState(false);
                    follower.setMaxPower(0.3);
                }

                if (follower.getCurrentPath() == path6 && follower.getPathCompletion() > 0.85) {
                    follower.setMaxPower(1);
                }

                //path 8 = path after path 7 - refer to path 7
                if (follower.getCurrentPath() == path7 && !initVar) {
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
