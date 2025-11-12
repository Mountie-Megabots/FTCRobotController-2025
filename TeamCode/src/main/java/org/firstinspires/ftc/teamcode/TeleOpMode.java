package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp
public class TeleOpMode extends OpMode {
    Follower follower;
    double slowModeMult = 1;
    boolean isRobotCentric = true;
    boolean automatedDrive = false;

    ShooterSystem shooter;

    Pose designatedPose;

    private Supplier<PathChain> pathToPose;

    @Override
    public void init() {
        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose((follower.getPose() == null) ? new Pose(0,0,0) : follower.getPose());
        shooter = new ShooterSystem(hardwareMap);
        pathToPose = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, designatedPose::getPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, designatedPose::getHeading, 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        if (gamepad1.back) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }
        if (gamepad1.rightBumperWasPressed()) {
            slowModeMult = (slowModeMult == 0.5) ? 1 : 0.5;
        }

        if (gamepad1.leftBumperWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }

        if (gamepad1.left_trigger > 0.2) {
            shooter.nextState(true);
            shooter.setShooterSlow();
        }
        else if (gamepad1.right_trigger > 0.2) {
            shooter.nextState(true);
            shooter.setShooterFast();
        } else {
            shooter.nextState(false);
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMult,
                -gamepad1.left_stick_x * slowModeMult,
                -gamepad1.right_stick_x/1.5 * slowModeMult,
                isRobotCentric
        );

        if (automatedDrive && (gamepad1.xWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.dpadUpWasPressed()) {
            shooter.changeShooterSpeed(50);
        }

        if (gamepad1.dpadDownWasPressed()) {
            shooter.changeShooterSpeed(-50);
        }

        if (gamepad1.startWasPressed()) {
            designatedPose = follower.getPose();
        }

        if (gamepad1.aWasPressed() && !follower.isBusy()) {
            automatedDrive = true;
            follower.followPath(pathToPose.get());
        }

        shooter.setStopState(gamepad1.b);

        shooter.functions();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        shooter.pushTelemetry(telemetry);
        telemetry.update();

    }
}
