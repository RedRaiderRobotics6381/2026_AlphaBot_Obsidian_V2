package frc.robot.Commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.Secondary.RotateSubsystem;
import frc.robot.subsystems.Secondary.Turret;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoShooter extends Command {
    private Indexer m_indexer;
    private RotateSubsystem m_rotate;
    private Outtake m_outtake;
    private Turret m_turret;
    private CommandSwerveDrivetrain drivetrain;
    private double distance;
    private double lowerBoundAngle;
    private double upperBoundAngle;
    private double angle;
    private double yaw;

    public AutoShooter(Outtake m_outtake, RotateSubsystem m_rotate, Indexer m_indexer, CommandSwerveDrivetrain drivetrain, Turret m_turret) {
        this.m_indexer = m_indexer;
        this.m_outtake = m_outtake;
        this.m_rotate = m_rotate;
        this.drivetrain = drivetrain;
        this.m_turret = m_turret;
    }

    @Override
    public void initialize(){
    
    m_outtake.setVelocity(ConstantValues.SHOOTER_RPM);                    

    }

    @Override
    public void execute(){
        distance =  Math.sqrt(Math.pow(182.1 - drivetrain.getState().Pose.getX(), 2) + Math.pow(158.5 - drivetrain.getState().Pose.getY(), 2));
        lowerBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED,4) - 
                          384 * (384 * Math.pow(distance - FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED, 2)))) / 
                          384 * (distance - FieldConstants.SMALLEST_RADIUS_OF_HOLE));
    

                          
        upperBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED,4) - 
                          384 * (384 * Math.pow(distance + FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED, 2)))) / 
                          384 * (distance + FieldConstants.SMALLEST_RADIUS_OF_HOLE));
    
        angle = (lowerBoundAngle + upperBoundAngle) / 2;
        m_rotate.setRotateAngleCmd(angle);
        
        yaw = Math.atan((158.5 - drivetrain.getState().Pose.getY())/(182.1 - drivetrain.getState().Pose.getX()));
        // m_turret.setTurretCmd(yaw);
        drivetrain.driveToPoseWithConstraints(new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), new Rotation2d(yaw)), new PathConstraints(100, 100, 100, 100));
        //if(Math.abs(m_rotate.armAngMtr.getPosition().getValueAsDouble() - angle) <= 1 && Math.abs(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPM) <= 30 && Math.abs(m_turret.turretAngMtr.getPosition().getValueAsDouble() - yaw) <= 1){
            if(Math.abs(m_rotate.armAngMtr.getPosition().getValueAsDouble() - angle) <= 1 && Math.abs(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPM) <= 30 && Math.abs(drivetrain.getState().Pose.getRotation().getRadians() - yaw) <= 0.5){
            m_indexer.indexMtrLdr.setVoltage(10);;
        } else {
            m_indexer.indexMtrLdr.setVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_outtake.setVelocity(0);
        m_indexer.indexMtrLdr.set(0);
    }
} 
