package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class TeleOpMode extends NextFTCOpMode {
    {
        addComponents(new PedroComponent(Constants::createFollower),  new SubsystemComponent(Shooter.INSTANCE));
    }

    Follower follower;
    double slowModeMult = 1;
    boolean isRobotCentric = true;
    @Override
    public void onInit() {
        follower  = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(75, 120, Math.toRadians(0)));
    }

    @Override
    public void onStartButtonPressed() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void onUpdate() {
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
