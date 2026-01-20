package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAutoShooter extends Command {
    private Indexer m_indexer;
    private Outtake m_outtake;
    private CommandSwerveDrivetrain drivetrain;
    private double distance;
    private double lowerBoundAngle;
    private double upperBoundAngle;
    private double angle;

    public AutoAutoShooter(Outtake m_outtake, Indexer m_indexer, CommandSwerveDrivetrain drivetrain) {
        this.m_indexer = m_indexer;
        this.m_outtake = m_outtake;
        this.drivetrain = drivetrain;
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
        
            if(Math.abs(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPM) <= 30){
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
