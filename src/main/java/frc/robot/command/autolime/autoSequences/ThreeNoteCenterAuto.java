package frc.robot.command.autolime.autoSequences;

import java.util.Optional;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.command.autolime.AutoAlignTags;
import frc.robot.command.autolime.AutoDrive;
import frc.robot.command.autolime.AutoDriveAndTrackNote;
import frc.robot.command.autolime.AutoIntake;
import frc.robot.command.autolime.AutoRotate;
import frc.robot.command.autolime.AutoShootSmart;
import frc.robot.command.autolime.NoteRotationAlign;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class ThreeNoteCenterAuto extends SequentialCommandGroup{

    SwerveSubsystem swerveSub;
    Shooter shooterSub;
    Intake intakeSub;

    public ThreeNoteCenterAuto(SwerveSubsystem swerveSub, Shooter shooterSub, Intake intakeSub) {
        addRequirements(swerveSub);
        addRequirements(shooterSub);
        addRequirements(intakeSub);
        this.swerveSub = swerveSub;
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;

        // var autoAlign = new AutoAlignTags(swerveSub);
        // var autoAlignSpeaker3 = new AutoAlignTags(swerveSub);
        
        

        
        addCommands(
                new AutoAlignTags(swerveSub).withTimeout(.5),
                new StopCommand(swerveSub),
                new AutoShootSmart(shooterSub, intakeSub),
                // get second note (center)
                new AutoDrive(swerveSub, 0.75, 0.7),
                Commands.race(
                    new AutoDriveAndTrackNote(swerveSub, 1, 0.4),
                    Commands.race(
                        new AutoIntake(intakeSub),
                        new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.15))
                    )
                ),
                new AutoDrive(swerveSub, 0.8, -0.6).withTimeout(4),
                // autoAlign.until(autoAlign::aligned),//.until(AutoAlignTags::aligned),
                new AutoAlignTags(swerveSub).until(AutoAlignTags::aligned),
                // this line is not a mistake, we might have overshot in the above line, so we run a bit longer
                new AutoAlignTags(swerveSub).withTimeout(.5), 
                new StopCommand(swerveSub),
                new AutoShootSmart(shooterSub, intakeSub).withTimeout(4),
                // end 2nd note
                new ConditionalCommand(new AutoRotate(swerveSub, 35, 0.08), new AutoRotate(swerveSub, -35, 0.08), this::isBlue),
                Commands.race(
                    new AutoDriveAndTrackNote(swerveSub, 2, 0.35),
                    Commands.race(
                        new AutoIntake(intakeSub),
                        new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.15))
                    )
                ),
                //new ConditionalCommand(new AutoRotate(swerveSub, 10, 0.6).withTimeout(1), new AutoRotate(swerveSub, -10, 0.6), this::isBlue),
                new AutoDrive(swerveSub, 1, -0.6).withTimeout(2.5),
               // new AutoRotate(swerveSub, 45, 0.5)),
                //new AutoAlignTags(swerveSub)
                new ConditionalCommand(new AutoRotate(swerveSub, -31, 0.25), new AutoRotate(swerveSub, 31, 0.25), this::isBlue)
                    .until(AutoAlignTags::speakerAimReady)
                    .withTimeout(4),
                // autoAlignSpeaker3.until(autoAlignSpeaker3::aligned),//.until(AutoAlignTags::aligned),
                new AutoAlignTags(swerveSub).until(AutoAlignTags::aligned),
                // this line is not a mistake, we might have overshot in the above line, so we run a bit longer
                new AutoAlignTags(swerveSub).withTimeout(1), 
                new StopCommand(swerveSub),

                new AutoShootSmart(shooterSub, intakeSub).withTimeout(2),

                // start note 4
                 new ConditionalCommand(new AutoRotate(swerveSub, -50, 0.2), new AutoRotate(swerveSub, 50, 0.2), this::isBlue),
                 Commands.race(
                     new AutoDriveAndTrackNote(swerveSub, 2.5, 0.35),
                     Commands.race(
                        new AutoIntake(intakeSub),
                        new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(0.5))
                    )
                ),
                new AutoDrive(swerveSub, 2.5, -0.7).withTimeout(2),
                new ConditionalCommand(new AutoRotate(swerveSub, 10, 0.25), new AutoRotate(swerveSub, -10, 0.25), this::isBlue)
                    .until(AutoAlignTags::speakerAimReady)
                    .withTimeout(4),
                new AutoAlignTags(swerveSub).until(AutoAlignTags::aligned),
                new AutoAlignTags(swerveSub).withTimeout(1),
                new StopCommand(swerveSub),
                new AutoShootSmart(shooterSub, intakeSub).withTimeout(2)

        );
        //         new AutoDrive(swerveSub, 1,#).withTimeout(1),
        //         new ConditionalCommand(new AutoRotate(swerveSub, #, #), new AutoRotate(swerveSub, #, #), this::isBlue)

        // );

                // new AutoRotate(swerveSub, 90, 0.5),
                //  Commands.race(
                //     new AutoDrive(swerveSub, 3, 0.2),
                //     Commands.race(
                //         new AutoIntake(intakeSub),
                //         new WaitUntilCommand(intakeSub::hasNote).andThen(new WaitCommand(.3))
                //     )
                // ),
                // new Auto
    }

    
    private MedianFilter zFilter = new MedianFilter(7);
    private boolean closeEnough() {
        if(LimelightHelpers.getTV("limelight-back")){
        var rawZ = LimelightHelpers.getTargetPose3d_RobotSpace(("limelight-back")).getZ();
        var z = zFilter.calculate(rawZ);
        if(z < 1.3){
            return true;
        }
        else{
            return false;
        }
        }
        else{
            return false;
        }

    }

    private boolean isBlue() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if(ally.isPresent()){
            if (ally.get() == Alliance.Blue) {
                System.out.println("Blue");
                return true;
            }
            else {
                System.out.println("Red");
                return false;
            }
        }
        else{
                System.out.println("error :(");
            return false;
        }
    }
}
