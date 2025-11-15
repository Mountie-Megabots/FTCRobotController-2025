package org.firstinspires.ftc.teamcode.Autos.CompAutos.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ShooterSystem;
import org.firstinspires.ftc.teamcode.Autos.PedroHelper;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "CloseSixPieceBlue", group = "Blue")
public class CloseSixPieceBlue extends OpMode {
    Follower follower;
    ShooterSystem shooter;
    public Path backupShoot, path2, path3, path4, leave;
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
        follower.setStartingPose(Constants.paths.CloseScoreConst.centerStart);
        PedroHelper.onBlueAlliance();

        backupShoot = PedroHelper.runPath(Constants.paths.CloseScoreConst.backupCenter);

        path2 = PedroHelper.runPath(Constants.paths.CloseScoreConst.centerEnd, Constants.paths.GrabConst.GPPStart);

        path3 = PedroHelper.runPath(Constants.paths.GrabConst.GPP);

        path4 = PedroHelper.runPath(Constants.paths.GrabConst.GPP.getLastControlPoint(), Constants.paths.CloseScoreConst.centerEnd);

        pickupChain = new PathChain(path2, path3, path4);

        leave = PedroHelper.runPath(Constants.paths.CloseScoreConst.centerEnd, Constants.paths.CloseScoreConst.launchLeave);
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
                        pathState = State.leave;
                    }
                } else {
                    timer.reset();
                }
                break;

            case leave:
                follower.followPath(leave, false);
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
