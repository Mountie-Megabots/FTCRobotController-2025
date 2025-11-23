package org.firstinspires.ftc.teamcode.Autos.CompAutos.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autos.PedroHelper;
import org.firstinspires.ftc.teamcode.ShooterSystem;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "FarSixPieceRed", group = "Red")
public class FarSixPieceRed extends OpMode {
    Follower follower;
    ShooterSystem shooter;
    public Path path1, path2, path3, path4, leave;
    public PathChain pickupChain;
    ElapsedTime timer;

    boolean initVar = false;
    enum State{
        firstPath,
        shoot1,
        toPickup,
        shoot2,
        leave
    }
    private State pathState;

    @Override
    public void init() {
        timer = new ElapsedTime();
        pathState = State.firstPath;
        shooter = new ShooterSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.Paths.FarScoreConst.farStart.mirror());
        PedroHelper.onRedAlliance();
        //insert bezier line or 2 poses
        path1 = PedroHelper.createLine(Constants.Paths.FarScoreConst.farStart,
                Constants.Paths.FarScoreConst.farScore);

        //put in paths in chain
        path2 = PedroHelper.createLine(Constants.Paths.FarScoreConst.farScore,
                Constants.Paths.GrabConst.PPGStart);

        path3 = PedroHelper.createLine(Constants.Paths.GrabConst.PPG.getFirstControlPoint(),
                Constants.Paths.GrabConst.PPG.getLastControlPoint());

        path4 = PedroHelper.createLine(Constants.Paths.GrabConst.PPG.getLastControlPoint(),
                Constants.Paths.FarScoreConst.farScore);

        pickupChain = new PathChain(path2, path3, path4);
        leave = PedroHelper.createLine(Constants.Paths.FarScoreConst.farScore, Constants.Paths.FarScoreConst.leave);
    }

    private void runPath() {
        switch (pathState) {
            case firstPath:
                follower.followPath(path1, true);
                pathState = State.shoot1;
                shooter.setShooterFast();
                break;
            case shoot1:
                //ready shooter
                if (!follower.isBusy()) {
                    //fire
                    shooter.nextState(true);
                    if (timer.seconds() > 4.5) {
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
                    if (timer.seconds() > 4.5) {
                        shooter.setStopState(true);
                        initVar = false;
                        pathState = State.leave;
                        follower.followPath(leave, false);
                    }
                } else {
                    timer.reset();
                }
                break;
            case leave:
                break;

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
