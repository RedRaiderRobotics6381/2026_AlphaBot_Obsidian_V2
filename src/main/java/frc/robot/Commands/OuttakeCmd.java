package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;

public class OuttakeCmd extends Command {
    private Indexer m_indexer;
    private Outtake m_outtake;

    public OuttakeCmd(Outtake m_outtake, Indexer m_indexer) {
        this.m_indexer = m_indexer;
        this.m_outtake = m_outtake;
    }

    @Override
    public void initialize(){
    
    m_outtake.setVelocity(10);
    m_indexer.setVoltage(3);
    }

    @Override
    public void execute(){
        if(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() < 22.5){
            m_indexer.setVoltage(0);
        } else {
            m_indexer.setVoltage(1);
        }
    }

    @Override
    public void end(boolean interrupted){
        // m_outtake.setVelocity(0);
        // m_indexer.setVoltage(0);
    }
} 
