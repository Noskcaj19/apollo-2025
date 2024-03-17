package frc.robot.command;

import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultIntake extends Command {

    private final Joystick primaryController;
    private final XboxController secondaryController;
    Intake mouth;
    Shooter shooterSub;

    public DefaultIntake(Intake mouth, XboxController secondaryController, Shooter shooterSub, Joystick primaryController) {

        addRequirements(mouth);
        this.secondaryController = secondaryController;
        this.primaryController = primaryController;
        this.mouth = mouth;
        this.shooterSub = shooterSub;
    }

    // comment out which control method the drivers dont want
    // private final XboxController primaryController = new XboxController(0);
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (true) {
            // mouth.printshtuff();
        }

        if (secondaryController.getLeftBumper()) {
            mouth.smartIntake();
        } else {
            mouth.stopSmIntake();
        }
        // if (primaryJoystick.getRawButtonPressed(12)) {
        // mouth.smartIntake();
        // // System.out.println(mouth.get());
        // } else if (primaryJoystick.getRawButtonReleased(12)) {
        // mouth.stopSmIntake();
        // // System.out.println(mouth.get());
        // }

        // System.out.println(mouth.getTaking());

        if (secondaryController.getRightBumperPressed() || primaryController.getRawButtonPressed(7)) {
            mouth.feedOn();
        }
        if (secondaryController.getRightBumperReleased() || primaryController.getRawButtonReleased(7)) {
            mouth.feedOff();
            shooterSub.turnOff();
        }
        // if (primaryJoystick.getRawButton(4)) {
        // mouth.sing();
        // }
        // if (primaryJoystick.getRawButtonReleased(4)) {
        // mouth.intakeOff();
        // }
    }
}