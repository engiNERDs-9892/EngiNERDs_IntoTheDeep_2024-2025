package org.firstinspires.ftc.teamcode;

public abstract class toggleButton {

    public boolean getToggleState() {
        return toggleState;
    }

    private boolean toggleState;
    private boolean previousButtonState;
    public enum Edge {
            RISING,
            FALLING,
    };
    private Edge edge;
    public toggleButton(Edge edge){
        this.edge = edge;
        toggleState = false;
    }
    public toggleButton(){
        edge = Edge.RISING;
        toggleState = false;
    }
    public void update(boolean buttonState){
        boolean triggerToggle = !previousButtonState && buttonState && edge == Edge.RISING ||
                previousButtonState && !buttonState && edge == Edge.FALLING;
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
