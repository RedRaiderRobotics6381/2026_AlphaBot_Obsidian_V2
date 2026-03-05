package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class IndexerControl extends Command {
    private Indexer m_indexer;
    private Outtake m_outtake;
    private CommandSwerveDrivetrain drivetrain;
    private AutoAimer m_autoAimer;

    public IndexerControl(Outtake m_outtake, Indexer m_indexer, CommandSwerveDrivetrain drivetrain, AutoAimer m_autoAimer) {
        this.m_indexer = m_indexer;
        this.m_outtake = m_outtake;
        this.drivetrain = drivetrain;
        this.m_autoAimer = m_autoAimer;
    }

    @Override
    public void execute(){
        if(drivetrain.distanceToHub < ConstantValues.DISTANCE_TO_SHOOT){
        if(Math.abs(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPS_NEAR) < 1 && m_autoAimer.atAngle && Math.abs(drivetrain.yaw - drivetrain.getState().Pose.getRotation().getRadians()) < 0.05){
            m_indexer.setVoltage(5);

        } else {
            m_indexer.setVoltage(0);
        }
    } else {
        if(Math.abs(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPS_FAR) < 1 && m_autoAimer.atAngle && Math.abs(drivetrain.yaw - drivetrain.getState().Pose.getRotation().getRadians()) < 0.05){
            m_indexer.setVoltage(5);

        } else {
            m_indexer.setVoltage(0);
        }
    }
    }

    @Override
    public void end(boolean interrupted){
        m_indexer.setVoltage(0);
    }
} 

