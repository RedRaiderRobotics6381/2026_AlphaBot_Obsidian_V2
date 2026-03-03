package frc.robot.Commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.Secondary.Rotation;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoShooter extends Command {
    private Indexer m_indexer;
    private Rotation m_rotate;
    private Outtake m_outtake;
    private CommandSwerveDrivetrain drivetrain;
    private double angle;
    private double yaw;
    private OuttakeRun m_outtakeRun;
    private AutoAimer m_autoAimer;
    private IndexerControl m_indexerControl;
    private DriveToYaw m_driveToYaw;

    public AutoShooter(Outtake m_outtake, Rotation m_rotate, Indexer m_indexer, CommandSwerveDrivetrain drivetrain, OuttakeRun m_outtakeRun, AutoAimer m_autoAimer, IndexerControl m_indexerControl, DriveToYaw m_driveToYaw) {
        this.m_indexer = m_indexer;
        this.m_outtake = m_outtake;
        this.m_rotate = m_rotate;
        this.drivetrain = drivetrain;
        this.m_outtakeRun = m_outtakeRun;
        this.m_autoAimer = m_autoAimer;
        this.m_indexerControl = m_indexerControl;
        this.m_driveToYaw = m_driveToYaw;
    }

    @Override
    public void initialize(){   
        CommandScheduler.getInstance().schedule(m_outtakeRun, m_autoAimer, m_indexerControl, m_driveToYaw);
    }

    // @Override
    // public void execute(){
        
        
    //     yaw = Math.atan((158.5 - drivetrain.getState().Pose.getY())/(182.1 - drivetrain.getState().Pose.getX()));
    //     //drivetrain.driveToPoseWithConstraints(new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), new Rotation2d(yaw)), new PathConstraints(100, 100, 100, 100));
    //     //if(Math.abs(m_rotate.sliderLdrMtr.getPosition().getValueAsDouble() - angle) <= 1 && Math.abs(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPM) <= 30 && Math.abs(m_turret.turretAngMtr.getPosition().getValueAsDouble() - yaw) <= 1){
        
    // }

    @Override
    public void end(boolean interrupted){
        CommandScheduler.getInstance().cancel(m_outtakeRun, m_autoAimer, m_indexerControl, m_driveToYaw);
    }
}