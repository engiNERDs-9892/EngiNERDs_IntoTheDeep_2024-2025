package org.firstinspires.ftc.teamcode;

public abstract class edgeButton {
    private boolean previousButtonState;
    public enum Edge {
            RISING,
            FALLING,
    };
    private Edge edge;
    public edgeButton(Edge edge){
        this.edge = edge;
    }
    public edgeButton(){
        edge = Edge.RISING;
    }
    public void update(boolean buttonState){
        boolean trigger = !previousButtonState && buttonState && edge == Edge.RISING ||
                previousButtonState && !buttonState && edge == Edge.FALLING;
        if(trigger){
            trigger();
        }
        previousButtonState = buttonState;
    }
    public abstract void trigger();
}
