package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class repeatButton {
    private boolean previousButtonState;
    private double delay;
    private double repeatPeriod;
    private ElapsedTime delayTimer;
    private ElapsedTime repeatTimer;
    public repeatButton(double delay, double repeatPeriod){
        this.delay = delay;
        this.repeatPeriod = repeatPeriod;
        this.delayTimer = new ElapsedTime();
        this.repeatTimer = new ElapsedTime();
    }
    public repeatButton(){
        this.delay = 0.75;
        this.repeatPeriod = 0.25;
        this.delayTimer = new ElapsedTime();
        this.repeatTimer = new ElapsedTime();
    }
    public void update(boolean buttonState){
        boolean trigger = !previousButtonState && buttonState;
        boolean hold = previousButtonState && buttonState;
        if(trigger){
            trigger();
            delayTimer.reset();
            repeatTimer.reset();
        }
        if(hold && delayTimer.time() > delay && repeatTimer.time() > repeatPeriod){
            repeatTimer.reset();
            trigger();
        }
        previousButtonState = buttonState;
    }
    public abstract void trigger();
}
