package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSystem {

    private final DcMotor shooter;
    private final DcMotor holder;
    private final DcMotor intake;

    private boolean nextState;

    private boolean stopState;
    ElapsedTime timer;

    int slowShooterSpeedSet = 1825;
    int customShooterSpeedSet = 2400;

    int shooterSpeed = 0;

    int speedThreshold = 0;

    enum State {
        Nothing,
        intake,
        retract,
        spinup,
        armed,
        removeBall,
        stop
    }

    private State functionState;

    public ShooterSystem(HardwareMap hm) {
        shooter = hm.get(DcMotor.class, "shooter");
        holder = hm.get(DcMotor.class, "holder");
        intake = hm.get(DcMotor.class, "intake");
        functionState = State.Nothing;
        timer = new ElapsedTime();

    }

    //if the passed true, then move to the next state or part of state
    public void nextState(boolean state) {
        nextState = state;
    }

    //if at speed threshold, return true
    public boolean shooterAtSpeed() {
        return ((DcMotorEx) shooter).getVelocity() > speedThreshold;
    }

    public void setStopState(boolean state) {
        stopState = state;
    }

    //either set the speed of the shooter to shoot from afar or up close
    public void setShooterSlow() {
        shooterSpeed = slowShooterSpeedSet;
    }

    public void setShooterFast() {
        shooterSpeed = customShooterSpeedSet;
    }

    //set the speed of all motors to backwards, and attempt to remove artifacts
    public void intakeBackwards(boolean remove) {
        functionState = (remove) ? State.removeBall : State.intake;
    }

    public void functions() {
        speedThreshold = shooterSpeed - 25;
        switch (functionState) {
            case Nothing:
                functionState = State.intake;
                break;
            case intake:
                intake.setPower(1);
                shooter.setPower(-0.2);
                holder.setPower(0);
                //if right trigger pressed, begin retracting, otherwise stay intaking
                if ((nextState)) {
                    timer.reset();
                    functionState = State.retract;
                    nextState = false;
                }
                break;

            case retract:
                intake.setPower(-1);
                holder.setPower(-1);
                if (timer.seconds() > 0.2) {
                    functionState = State.spinup;
                    timer.reset();
                }
                //if the intake has passed the distance needed for retracting
                //begin spinning up the shooter otherwise keep moving to position
                break;
            case spinup:
                intake.setPower(0);
                holder.setPower(0);
                ((DcMotorEx) shooter).setVelocity(shooterSpeed);
                //if the shooter has reached the desired speed, set state to armed and hold velocity
                //otherwise, keep attempting to reach velocity or stop when the timer reaches two seconds/stop is pressed

                functionState = (shooterAtSpeed() || timer.seconds() > 2) ? State.armed : State.spinup;
                if (stopState) {
                    stopState = false;
                    functionState = State.intake;
                }
                break;
            case armed:
                intake.setPower(0);
                holder.setPower(0);
                //shoot button pressed, begin firing
                if (nextState && shooterAtSpeed()) {
                    intake.setPower(1);
                    holder.setPower(1);
                } else if (stopState) {
                    //if stop, then return to intake from stop
                    functionState = State.stop;
                } else {
                    //nothing done
                }
                break;
            case removeBall:
                //attempt to remove artifacts in the shooter system
                intake.setPower(-1);
                holder.setPower(-1);
                shooter.setPower(-1);
            case stop:
                //reset everything
                intake.setPower(0);
                holder.setPower(0);
                shooter.setPower(0);
                functionState = State.Nothing;
                stopState = false;
                nextState = false;
                break;

        }
    }

    //change shooter speed on left trigger by specified amount 
    public void changeShooterSpeed(int amount) {
        customShooterSpeedSet += amount;
    }

    public void pushTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter State", functionState);
        telemetry.addData("Shooter Velocity", ((DcMotorEx) shooter).getVelocity());
        telemetry.addData("Shooter Custom Velocity Setting", customShooterSpeedSet);
    }
}
