package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;

public class StupidIndexer extends Command {
    private Indexer m_indexer;
    private Outtake m_outtake;

    public StupidIndexer(Outtake m_outtake, Indexer m_indexer) {
        this.m_indexer = m_indexer;
        this.m_outtake = m_outtake;
    }

    @Override
    public void execute(){
       if(Math.abs(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPS_FAR) < 1){
        m_indexer.setVoltage(5);
       } else {
        m_indexer.setVoltage(0);
       }
    }

    @Override
    public void end(boolean interrupted){
        m_indexer.setVoltage(0);
    }
} 

