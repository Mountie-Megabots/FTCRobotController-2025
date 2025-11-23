package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp
public class TeleOpMode extends OpMode {
    Follower follower;
    double slowModeMulti = 1;
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
        //generator to move to point
        pathToPose = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, designatedPose::getPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, designatedPose::getHeading, 0.8))
                .build();
    }

    //during the first loop of loop begin teleop drive
    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        // reset the pose for field centric
        if (gamepad1.back) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }


        //if dpad left is pressed, switch between field and robot centric
        if (gamepad1.dpadLeftWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }

        //if start is pressed, set the pose that the robot will go to automatically
        if (gamepad1.startWasPressed()) {
            designatedPose = follower.getPose();
        }

        //if a is pressed, begin driving automatically to the set pose
        if (gamepad1.aWasPressed() && !follower.isBusy()) {
            automatedDrive = true;
            follower.followPath(pathToPose.get());
        }

        shooter.runLifter(gamepad2.left_stick_y);

        //if automated drive finishes or the joystick is moved then cancel and begin teleop drive
        if (automatedDrive && (joystickMoved() || !follower.isBusy())) {
            follower.startTeleopDrive(true);
            automatedDrive = false;
        }

        //if right bumper is held, move at half speed
        slowModeMulti = (gamepad1.right_bumper) ? 0.5 : 1;

        //run the robot through gamepads, with modifiers from slow mode
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMulti,
                -gamepad1.left_stick_x * slowModeMulti,
                -gamepad1.right_stick_x/1.5 * slowModeMulti,
                isRobotCentric
        );

        //if left trigger was pressed, shoot from afar
        if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2) {
            shooter.nextState(true);
            shooter.setShooterFast();
        }
        //if right trigger was pressed, shoot from close
        else if (gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2) {
            shooter.nextState(true);
            shooter.setShooterSlow();
        }
        //if nothing is pressed, don't do anything in the current state
        else {
            shooter.nextState(false);
        }

        /*
        //increase or decrease the left trigger shoot speed
        if (gamepad1.dpadUpWasPressed()) {
            shooter.changeShooterSpeed(50);
        }

        if (gamepad1.dpadDownWasPressed()) {
            shooter.changeShooterSpeed(-50);
        }
         */

        //if b is pressed then go into the stop path of the current state
        shooter.setStopState(gamepad1.b || gamepad2.b);

        //if left bumper is pressed, attempt to back out all artifacts in the system
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            shooter.intakeBackwards(true);
        }
        //once left bumper is released, then stop, and return to intake state
        else if (gamepad1.leftBumperWasReleased() || gamepad2.leftBumperWasReleased()) {
            shooter.intakeBackwards(false);
        }

        //run all functions, such as the state machine
        shooter.functions();

        //push all telemetry
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        shooter.pushTelemetry(telemetry);
        telemetry.update();

    }

    public boolean joystickMoved() {
        return gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_y > 0.2 || gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_y > 0.2;
    }
}
