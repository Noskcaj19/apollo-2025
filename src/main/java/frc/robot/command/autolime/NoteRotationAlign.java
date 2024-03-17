package frc.robot.command.autolime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsytems.SwerveSubsystem;

public class NoteRotationAlign extends Command{
    private SwerveSubsystem swerveSub;
    private ProfiledPIDController rotationPid;
    double ku = .4;
    double tu = 1.1;
    double kuh = .3;
    double tuh = 1.6;

    final double getHorizontalOffset() {
        return (LimelightHelpers.getTX("limelight-front"));
        // return (x.getDouble(160)/160)-1;
        // whatever the distance is
        // returns the specific distance value we want so we can pid it???
        // why is everything so
    }
static int count = 0;
    public NoteRotationAlign(SwerveSubsystem swerveSub){
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);
        rotationPid = new ProfiledPIDController(kuh*.6, tuh*.5, tuh*.125,
        // horizontalPid = new ProfiledPIDController(kuh*.6, 0, tuh*.125,
        new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3/1.5));
        Shuffleboard.getTab("pid_debug").add("note_rotation_PID"+count, rotationPid);
        count+=1;
    }
    
    @Override
    public void initialize() {
        // forwardPid.reset(1.5);
        // horizontalPid.reset(0);
        
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight-front")) {
            var rot = rotationPid.calculate(getHorizontalOffset());
            rot = MathUtil.clamp(rot, -DriveConstants.MaxVelocityMetersPerSecond/5, DriveConstants.MaxVelocityMetersPerSecond/5);

            swerveSub.drive(0, 0, rot/DriveConstants.MaxAngularVelocityRadiansPerSecond, false);
        }else {
            swerveSub.drive(0, 0, -0.175, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(0, 0, 0, false, 0, 0);
    }
    
}
