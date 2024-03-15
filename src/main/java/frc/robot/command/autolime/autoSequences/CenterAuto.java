package frc.robot.command.autolime.autoSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.command.autolime.AutoDrive;
import frc.robot.command.autolime.AutoIntake;
import frc.robot.command.autolime.AutoIntakeEndless;
import frc.robot.command.autolime.AutoShootSmart;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class CenterAuto extends SequentialCommandGroup {

    SwerveSubsystem swerveSub;
    Shooter shooterSub;
    Intake intakeSub;

    public CenterAuto(SwerveSubsystem swerveSub, Shooter shooterSub, Intake intakeSub) {
        addRequirements(swerveSub);
        addRequirements(shooterSub);
        addRequirements(intakeSub);
        this.swerveSub = swerveSub;
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;

        addCommands(
                // new AutoShoot(shooterSub, intakeSub).until(intakeSub::doesntHaveNote).withTimeout(2), 
                new AutoShootSmart(shooterSub, intakeSub),
                Commands.race(
                    new AutoDrive(swerveSub, 3, 0.2),
                    new ParallelRaceGroup(
                        new AutoIntake(intakeSub),
                        new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.45))
                    )
                ),
                new AutoDrive(swerveSub, 5, -0.2).until(this::closeEnough),
                new AutoShootSmart(shooterSub, intakeSub)
        );
    }

    
    private boolean closeEnough() {
        if(LimelightHelpers.getTargetPose3d_CameraSpace(("limelight-back")).getZ() < 1.3){
            return true;
        }
        else{
            return false;
        }
    }
}