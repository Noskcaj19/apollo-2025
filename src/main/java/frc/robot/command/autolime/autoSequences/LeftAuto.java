
package frc.robot.command.autolime.autoSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.command.autolime.AutoAlignTags;
import frc.robot.command.autolime.AutoDrive;
import frc.robot.command.autolime.AutoDriveAndTrackNote;
import frc.robot.command.autolime.AutoIntake;
import frc.robot.command.autolime.AutoIntakeDrive;
import frc.robot.command.autolime.AutoRotate;
import frc.robot.command.autolime.AutoShoot;
import frc.robot.command.autolime.AutoShootSmart;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class LeftAuto extends SequentialCommandGroup {

    SwerveSubsystem swerveSub;
    Shooter shooterSub;
    Intake intakeSub;

    public LeftAuto(SwerveSubsystem swerveSub, Shooter shooterSub, Intake intakeSub) {
        addRequirements(swerveSub);
        addRequirements(shooterSub);
        this.swerveSub = swerveSub;
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;
        addCommands(
            new WaitCommand(7),
           // new AutoAlignTags(swerveSub).until(AutoAlignTags::aligned).withTimeout(0.5),
            new AutoShootSmart(shooterSub, intakeSub),
            new AutoDrive(swerveSub, 0.2, 0.5),
            new AutoRotate(swerveSub, -30, 0.25),
            new AutoDrive(swerveSub, 2.1, 0.5)
        );


                // new AutoShoot(shooterSub, intakeSub).withTimeout(3),
                // new AutoDrive(swerveSub, 1, 0.2).withTimeout(2),
                // new AutoRotate(swerveSub, -10, 0.1).withTimeout(3),
                // new AutoIntakeDrive(swerveSub, intakeSub, 0.5, 0.3),
                // new AutoRotate(swerveSub, 10, 0.1),
                // new AutoDrive(swerveSub, 1, 0.2).withTimeout(2),
                // new AutoAlignTags(swerveSub, 1)
                // new AutoIntake(in)
//                new AutoShoot(shooterSub, intakeSub).withTimeout(3),
//                new AutoDrive(swerveSub, 0.3, 0.2).withTimeout(3),
//                new AutoRotate(swerveSub, 25, 0.1).withTimeout(3),
//                new AutoDrive(swerveSub, 5, 0.3), 
               // new InstantCommand(()->{swerveSub.zeroYaw();},swerveSub)
//        );
    }
}