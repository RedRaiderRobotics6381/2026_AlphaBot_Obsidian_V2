package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class DriveToYaw extends Command{
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentricFacingAngle drive;
    private CommandXboxController joystick;
    private double MaxSpeed;
    private double MaxAngularRate;


    public DriveToYaw(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle drive, CommandXboxController joystick, double MaxSpeed, double MaxAngularRate) {
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.joystick = joystick;
        this.MaxSpeed = MaxSpeed;
        this.MaxAngularRate = MaxAngularRate;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        // drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
        //             .withTargetDirection(new Rotation2d(drivetrain.yaw)).withHeadingPID(6.0, 0, 0) // Drive counterclockwise with negative X (left)
        // );
        drivetrain.setControl(drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                     .withTargetDirection(new Rotation2d(drivetrain.yaw)).withHeadingPID(6.0, 0, 0));
    }
}
