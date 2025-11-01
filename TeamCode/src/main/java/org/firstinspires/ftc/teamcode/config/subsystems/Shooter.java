package org.firstinspires.ftc.teamcode.config.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter(){}

    private ControlSystem velController = ControlSystem.builder()
            .velPid(.005,0,0)
            .build();

    private MotorEx shooterMotor = new MotorEx("shooter");
    private MotorEx indexerMotor = new MotorEx("holder");
    private MotorEx intakeMotor = new MotorEx("intake");

    public Command intakeMode = new SequentialGroup(
            new RunToVelocity(velController, 0).requires(shooterMotor),
            new SetPower(indexerMotor, 0),
            new SetPower(intakeMotor, 1));

    public Command retract = new SequentialGroup(
            new SetPower(indexerMotor, -1),
            new SetPower(intakeMotor, -1),
            new Delay(250),
            new SetPower(indexerMotor, 0),
            new SetPower(intakeMotor, 0)
    );

    public Command spinup = new ParallelGroup(
            new SetPower(indexerMotor, 0),
            new SetPower(intakeMotor, 0),
            new RunToVelocity(velController, 2200).requires(shooterMotor)
            );

    public Command shoot = new ParallelGroup(
            new SetPower(indexerMotor, 1),
            new SetPower(intakeMotor, 1)
    );
}
