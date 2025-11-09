package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TeleOpMode extends OpMode {
    Follower follower;
    double slowModeMult = 1;
    boolean isRobotCentric = true;

    ShooterSystem shooter;

    @Override
    public void init() {
        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose((follower.getPose() == null) ? new Pose(0,0,0) : follower.getPose());
        shooter = new ShooterSystem(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        if (gamepad1.back) {
            follower.setPose(new Pose(0,0,0));
        }
        if (gamepad1.rightBumperWasPressed()) {
            slowModeMult = (slowModeMult == 0.5) ? 1 : 0.5;
        }

        if (gamepad1.leftBumperWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMult,
                -gamepad1.left_stick_x * slowModeMult,
                -gamepad1.right_stick_x * slowModeMult,
                isRobotCentric
        );

        if (gamepad1.dpadUpWasPressed()) {
            shooter.changeShooterSpeed(50);
        }

        if (gamepad1.dpadDownWasPressed()) {
            shooter.changeShooterSpeed(-50);
        }

        shooter.nextState(gamepad1.right_trigger > 0.2);

        shooter.setStopState(gamepad1.b);

        shooter.functions();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        shooter.pushTelemetry(telemetry);
        telemetry.update();

    }
}
