package frc.robot.Commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class DriveToYaw extends Command{
    private CommandSwerveDrivetrain drivetrain;
    private double yaw;

    public DriveToYaw(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute(){
        yaw = Math.atan((158.5 - drivetrain.getState().Pose.getY())/(182.1 - drivetrain.getState().Pose.getX()));
        drivetrain.driveToPoseWithConstraints(new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), new Rotation2d(yaw)), new PathConstraints(100, 100, 100, 100));
    }
}
