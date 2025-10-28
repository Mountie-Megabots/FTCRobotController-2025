package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TeleOpMode extends OpMode {
    Follower follower;
    double slowModeMult = 1;
    boolean isRobotCentric = true;
    @Override
    public void init() {
        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(75, 120, Math.toRadians(0)));
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
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

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }
}
