package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class ShooterSystem {

    private DcMotor shooter;
    private DcMotor holder;
    private DcMotor intake;

    private boolean nextState;

    private boolean stopState;

    ElapsedTime timer;
    int high_speed;
    int mid_speed = 2500;
    int no_speed;
    int low_speed;
    int target;
    int retractDistance;

    enum State{
        Nothing,
        intake,
        retract,
        spinup,
        armed,
        shooting,
        stop
    }

    private State functionState;

    ShooterSystem(HardwareMap hm){
        shooter = hm.get(DcMotor.class, "shooter");
        holder = hm.get(DcMotor.class, "holder");
        intake = hm.get(DcMotor.class, "intake");
        functionState = State.Nothing;
        timer = new ElapsedTime();

    }

    void nextState(boolean state) {
        nextState = state;
    }

    void setStopState(boolean state) {
        stopState = state;
    }


    void functions(){
        switch(functionState){
            case Nothing:
                functionState = State.intake;
                break;
            case intake:
                intake.setPower(1);
                shooter.setPower(0);
                holder.setPower(0);
                //if right trigger pressed, begin retracting, otherwise stay intaking
                if((nextState)){
                    timer.reset();
                    functionState = State.retract;
                    nextState = false;
                }
                break;

            case retract:
                intake.setPower(-1);
                holder.setPower(-1);
                if (timer.seconds() > 0.25) {
                    functionState = State.spinup;
                }
                //if the intake has passed the distance needed for retracting
                // begin spinning up the shooter otherwise keep moving to position
                break;
            case spinup:
                intake.setPower(0);
                holder.setPower(0);
                ((DcMotorEx) shooter).setVelocity(mid_speed);
                double currentSpeed = ((DcMotorEx) shooter).getVelocity();
                //if the shooter has reached the desired speed, set state to armed and hold velocity
                //otherwise, keep attempting to reach velocity

                functionState = (currentSpeed > 1000) ? State.armed : State.spinup;
                break;
            case armed:
                intake.setPower(0);
                holder.setPower(0);
                //shoot button pressed
                if (nextState) {
                    intake.setPower(1);
                    holder.setPower(1);
                } else if (stopState) {
                    // if stop shoot button pressed
                    functionState = State.stop;
                } else {
                    //nothing done
                }
                break;
            case stop:
                intake.setPower(0);
                holder.setPower(0);
                shooter.setPower(0);
                functionState = State.Nothing;
                stopState = false;
                nextState = false;
                break;

        }
    }
}
