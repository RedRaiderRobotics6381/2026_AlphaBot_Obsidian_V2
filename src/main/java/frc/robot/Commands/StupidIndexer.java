package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;

public class StupidIndexer extends Command {
    private Indexer m_indexer;

    public StupidIndexer(Indexer m_indexer) {
        this.m_indexer = m_indexer;
    }

    @Override
    public void execute(){
       if(Outtake.atSpeed){
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

