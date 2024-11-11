package org.firstinspires.ftc.teamcode;

public abstract class toggleButton {

    private boolean toggleState;
    private boolean previousButtonState;
    enum Edge {
            RISING,
            FALLING,
    };
    private Edge _edge;
    public void toggleButton(Edge edge){
        _edge = edge;
    }
    public void update(boolean buttonState){
        boolean triggerToggle = !previousButtonState && buttonState && _edge == Edge.RISING ||
                previousButtonState && !buttonState && _edge == Edge.FALLING;
        if(triggerToggle){
            toggleState = !toggleState;
            if(toggleState){
                toggleOn();
            }else{
                toggleOff();
            }
        }
        previousButtonState = buttonState;
    }
    public abstract void toggleOn();
    public abstract void toggleOff();

}
