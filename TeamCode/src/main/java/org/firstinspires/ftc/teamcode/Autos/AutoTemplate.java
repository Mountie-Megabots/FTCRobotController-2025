package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ShooterSystem;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Disabled
@Autonomous(name = "DONOTUSE", group = "DONTUSE")
public class AutoTemplate extends OpMode {
    Follower follower;
    ShooterSystem shooter;
    public Path path1;
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
        follower.setStartingPose(Constants.Paths.CloseScoreConst.centerStart);
        //Choose one
        //PedroHelper.onBlueAlliance();
        //PedroHelper.onRedAlliance();

        //insert bezier line or two poses
        //path1 = PedroHelper.runPath();

        //put in paths in chain for pickup
        pickupChain = new PathChain();
    }

    private void runPath() {
        switch (pathState) {
            case firstPath:
                follower.followPath(path1, true);
                pathState = State.shoot1;
                break;

            case shoot1:
                //ready shooter
                shooter.nextState(true);
                if (!follower.isBusy()) {
                    //fire
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
                //Use if grabbing pieces
                /*
                if (follower.getCurrentPath() == Path3 && follower.getPathCompletion() > 0.4) {
                    shooter.setStopState(true);
                    shooter.nextState(false);
                    follower.setMaxPower(0.3);
                }

                if (follower.getCurrentPath() == Path4 && !initVar) {
                    initVar = true;
                    shooter.nextState(true);
                    shooter.setStopState(false);
                    follower.setMaxPower(1);
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
                */
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
