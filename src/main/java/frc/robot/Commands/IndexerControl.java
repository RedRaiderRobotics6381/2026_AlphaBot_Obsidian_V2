package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class IndexerControl extends Command {
    private Indexer m_indexer;
    public static boolean override;

    public IndexerControl(Indexer m_indexer) {
        this.m_indexer = m_indexer;
    }

    @Override
    public void execute(){
        if(Outtake.atSpeed && AutoAimer.atAngle){
            if(!override){
                if(CommandSwerveDrivetrain.isAligned){
                    m_indexer.setVoltage(5);
                } else {
                    m_indexer.setVoltage(0);
                }
            } else {
                m_indexer.setVoltage(5);
            }
        } else {
            m_indexer.setVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_indexer.setVoltage(0);
        override = false;
    }
} 

