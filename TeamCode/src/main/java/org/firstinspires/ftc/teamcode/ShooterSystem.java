package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.wifi.DriverStationAccessPointAssistant;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ShooterSystem {

    private final DcMotor shooter;
    private final DcMotor holder;
    private final DcMotor intake;
    private final DcMotor lifter;
    private boolean nextState;
    private boolean initIntake = false;

    private final Servo light;

    private boolean stopState;
    ElapsedTime timer;

    int slowShooterSpeedSet = 1750 ;
    int customShooterSpeedSet = 2350;

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
        lifter = hm.get(DcMotor.class, "lifter");
        light = hm.get(Servo.class, "light");
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        holder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        holder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        functionState = State.Nothing;
        timer = new ElapsedTime();
        ((DcMotorEx) intake).setCurrentAlert(3000, CurrentUnit.MILLIAMPS);
        ((DcMotorEx) shooter).setCurrentAlert(1300, CurrentUnit.MILLIAMPS);
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
        if (functionState != State.armed) shooterSpeed = slowShooterSpeedSet;
    }

    public void runLifter(double value) {
        lifter.setPower(value);
    }

    public void setShooterFast() {
        if (functionState != State.armed) shooterSpeed = customShooterSpeedSet;
    }

    //set the speed of all motors to backwards, and attempt to remove artifacts
    public void intakeBackwards(boolean remove) {
        functionState = (remove) ? State.removeBall : State.intake;
    }

    public void functions() {
        speedThreshold = shooterSpeed - 75;
        switch (functionState) {
            case Nothing:
                functionState = State.intake;
                break;
            case intake:

                holder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                shooter.setPower(-0.2);
                if (((DcMotorEx) shooter).isOverCurrent()) {
                    holder.setPower(-0.35);
                } else {
                    holder.setPower(0.125);
                }
                if (!((DcMotorEx) intake).isOverCurrent()) {
                    timer.reset();
                }
                if (((((DcMotorEx) intake).isOverCurrent()  && timer.seconds() > 1) || initIntake) && ((DcMotorEx) shooter).isOverCurrent()) {
                    intake.setPower(0);
                    setLight(0.61);
                    initIntake = true;
                } else {
                    intake.setPower(1);
                    setLight(0.33);
                }
                //if right trigger pressed, begin retracting, otherwise stay intaking
                if ((nextState)) {
                    holder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    timer.reset();
                    functionState = State.retract;
                    nextState = false;
                    initIntake = false;
                }
                break;

            case retract:
                setLight(0.28);
                intake.setPower(-0.75);
                holder.setPower(-1);
                if (timer.seconds() > 0.3) {
                    functionState = State.spinup;
                    timer.reset();
                    ((DcMotorEx) shooter).setVelocity(shooterSpeed);
                }
                //if the intake has passed the distance needed for retracting
                //begin spinning up the shooter otherwise keep moving to position
                break;
            case spinup:
                setLight(0.28);
                intake.setPower(0);
                holder.setPower(0);
                //if the shooter has reached the desired speed, set state to armed and hold velocity
                //otherwise, keep attempting to reach velocity or stop when the timer reaches two seconds/stop is pressed
                functionState = (shooterAtSpeed() || timer.seconds() > 2) ? State.armed : State.spinup;
                if (stopState) {
                    stopState = false;
                    functionState = State.intake;
                }
                break;
            case armed:
                setLight(0.73);
                intake.setPower(0);
                holder.setPower(0);
                if (shooterAtSpeed() && !nextState) {
                   timer.reset();
                }
                //use this if on slow speed
                if (shooterSpeed == slowShooterSpeedSet &&
                        nextState && (shooterAtSpeed() || timer.seconds() < 0.1)) {
                    intake.setPower(0.75);
                    holder.setPower(1);
                }
                else if (shooterSpeed == customShooterSpeedSet &&
                            nextState && shooterAtSpeed()) {
                    intake.setPower(0.75);
                    holder.setPower(1);
                }
                else if (stopState) {
                    //if stop, then return to intake from stop
                    functionState = State.stop;
                } else {}
                break;
            case removeBall:
                //attempt to remove artifacts in the shooter system
                intake.setPower(1);
                holder.setPower(1);
                shooter.setPower(-1);
                setLight(1);
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
    private void setLight(double set){
        if (light.getPosition() == set){
            return;
        }
        light.setPosition(set);
    }

    //change shooter speed on left trigger by specified amount 
    public void changeShooterSpeed(int amount) {
        customShooterSpeedSet += amount;
    }

    public void pushTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter State", functionState);
        telemetry.addData("Shooter Velocity", ((DcMotorEx) shooter).getVelocity());
        //telemetry.addData("Shooter Custom Velocity Setting", customShooterSpeedSet);
        //telemetry.addData("Shooter At Speed", shooterAtSpeed());
        //telemetry.addData("Timer", timer.seconds());
        //telemetry.addData("NextState", nextState);
        //telemetry.addData("Intake MilliAmps",((DcMotorEx) intake).getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("initIntake", initIntake);
        telemetry.addData("Lifter Position", lifter.getCurrentPosition());
        telemetry.addData("Shooter Current", ((DcMotorEx) shooter).getCurrent(CurrentUnit.MILLIAMPS));
    }
}
