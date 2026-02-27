package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;

public class IndexerControl extends Command {
    private Indexer m_indexer;
    private Outtake m_outtake;

    public IndexerControl(Outtake m_outtake, Indexer m_indexer) {
        this.m_indexer = m_indexer;
        this.m_outtake = m_outtake;
    }

    @Override
    public void initialize(){

    m_indexer.setVoltage(5);

    }
    @Override
    public void execute(){
        if(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() < 29){
            m_indexer.setVoltage(0);

        } else {
            m_indexer.setVoltage(5);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_indexer.setVoltage(0);
    }
} 

