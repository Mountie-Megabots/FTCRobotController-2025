package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.ShooterSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "CloseScorePrePlusGPP", group = "Blue")
public class CloseScorePrePlusGPP extends OpMode {
    Follower follower;
    ShooterSystem shooter;
    public Path Path1, Path2, Path3, Path4, Path5;
    ElapsedTime timer;
    enum State{
        firstPath,
        firstPathInt,
        shoot1,
        toPickup,
        toPickupInt,
        pickup,
        pickupInt,
        reversePickup,
        reversePickupInt,
        returnToShoot,
        returnToShootInt,
        shoot2
    }
    private State pathState;

    @Override
    public void init() {
        timer = new ElapsedTime();
        pathState = State.firstPath;
        shooter = new ShooterSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.paths.CloseScoreConst.leftStart);

        Path1 = new Path(Constants.paths.CloseScoreConst.backupLeft);
        Path1.setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(155));

        Path2 = new Path(new BezierCurve(Constants.paths.CloseScoreConst.backupLeftEnd, Constants.paths.CloseScoreConst.curveControlPoint, Constants.paths.GrabConst.GPPStart));
        Path2.setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180));

        Path3 = new Path(Constants.paths.GrabConst.GPP);
        Path3.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));

        Path4 = new Path(Constants.paths.GrabConst.GPPRev);
        Path4.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));

        Path5 = new Path(new BezierCurve(Constants.paths.GrabConst.GPPStart, Constants.paths.CloseScoreConst.curveControlPoint, Constants.paths.CloseScoreConst.backupLeftEnd));
        Path5.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155));
    }

    private void runPath() {
        switch(pathState){
            case firstPath:
                shooter.nextState(true);
                follower.followPath(Path1, true);
                pathState = State.firstPathInt;
                break;
            case firstPathInt:
                if (!follower.isBusy()) {
                    pathState = State.shoot1;
                    timer.reset();
                }
                break;

            case shoot1:
                shooter.nextState(true);
                if (timer.seconds() > 5) {
                    shooter.setStopState(true);
                    shooter.nextState(false);
                    pathState = State.toPickup;
                }
                break;

            case toPickup:
                follower.followPath(Path2);
                pathState = State.toPickupInt;
                break;

            case toPickupInt:
                if (!follower.isBusy()) {
                    pathState = State.pickup;
                }
                break;

            case pickup:
                follower.followPath(Path3);
                pathState = State.pickupInt;
                follower.setMaxPower(0.2);
                break;

            case pickupInt:
                if (!follower.isBusy()) {
                    pathState = State.reversePickup;
                }
                break;
            case reversePickup:
                follower.followPath(Path4);
                pathState = State.reversePickupInt;
                follower.setMaxPower(1);
                break;

            case reversePickupInt:
                if (!follower.isBusy()) {
                    pathState = State.returnToShoot;
                }
                break;

            case returnToShoot:
                follower.followPath(Path5, true);
                pathState = State.returnToShootInt;
                break;

            case returnToShootInt:
                if (!follower.isBusy()) {
                    pathState = State.shoot2;
                    timer.reset();
                }
                break;

            case shoot2:
                shooter.nextState(true);
                if (timer.seconds() > 5) {
                    shooter.setStopState(true);
                    shooter.nextState(false);
                }
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
