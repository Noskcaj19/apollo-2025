package frc.robot.command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsytems.Arms;

public class DefaultClimb extends Command {

    private Joystick joy;
    private Arms climbSub;
    boolean status = false;

    public DefaultClimb(Joystick inJoy, Arms climb) {
        this.climbSub = climb;
        this.joy = inJoy;
    }

    @Override
    public void execute() {

        // double extendController;
        // if (secondaryController.getRightY() < 0.03 && secondaryController.getRightY()
        // > -0.03) {
        // extendController = 0;
        // } else {
        // extendController = secondaryController.getRightY() * 2;
        // }

        // var extendSet = -extendController + -clawSystem.getExtendSetPoint();
        // climbSub.set(extendSet);

        if (joy.getPOV() == 0) {
            status = !status;
            climbSub.upDown(status);
        }
        if (joy.getPOV() == 180) {
            climbSub.upDown(status);
        } else {
            climbSub.turnOff();
        }

    }

}