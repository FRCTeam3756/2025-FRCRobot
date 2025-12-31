package frc.robot.logic;

public class ControlManager {

    private boolean humanDriverControl = false;
    private boolean humanOperatorControl = false;

    public ControlManager() {
    }

    public void forceDrivingControl(boolean control) {
        humanDriverControl = control;
    }

    public void forceOperatorControl(boolean control) {
        humanOperatorControl = control;
    }

    public boolean isHumanDriverControl() {
        return humanDriverControl;
    }

    public boolean isHumanOperatorControl() {
        return humanOperatorControl;
    }

}
