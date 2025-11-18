package org.firstinspires.ftc.teamcode.Autos.ScrappedAutos;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autos.PedroHelper;
import org.firstinspires.ftc.teamcode.ShooterSystem;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "CloseThreePieceBlue", group = "Blue")
public class CloseThreePieceBlue extends OpMode {
    Follower follower;
    ShooterSystem shooter;
    public Path Path1;
    enum State{
        firstPath,
        firstPathInt,
        shoot1
    }
    private State pathState;

    @Override
    public void init() {
        pathState = State.firstPath;
        shooter = new ShooterSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Constants.Paths.CloseScoreConst.rightStart);
        Path1 = PedroHelper.createLine(Constants.Paths.CloseScoreConst.backupRight);


    }

    private void runPath() {
        switch(pathState){
            case firstPath:
                shooter.nextState(true);
                follower.followPath(Path1, false);
                pathState = State.firstPathInt;
                break;
            case firstPathInt:
                if (!follower.isBusy()) {
                    pathState = State.shoot1;
                }
                break;
            case shoot1:
                shooter.nextState(true);
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
