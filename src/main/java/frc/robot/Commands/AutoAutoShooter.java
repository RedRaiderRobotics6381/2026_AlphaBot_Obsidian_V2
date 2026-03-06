package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoAutoShooter extends Command {
    private OuttakeRun m_outtakeRun;
    private AutoAimer m_autoAimer;
    private IndexerControl m_indexerControl;

    public AutoAutoShooter(OuttakeRun m_outtakeRun, AutoAimer m_autoAimer, IndexerControl m_indexerControl) {
        this.m_outtakeRun = m_outtakeRun;
        this.m_autoAimer = m_autoAimer;
        this.m_indexerControl = m_indexerControl;
    }

    @Override
    public void initialize(){   
        CommandScheduler.getInstance().schedule(m_outtakeRun, m_autoAimer, m_indexerControl);
    }

    @Override
    public void end(boolean interrupted){
        CommandScheduler.getInstance().cancel(m_outtakeRun, m_autoAimer, m_indexerControl);
        System.out.println("Hello");
    }
}