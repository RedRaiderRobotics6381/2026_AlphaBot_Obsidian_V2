package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class StupidShooter extends Command {
    private OuttakeRun m_outtakeRun;
    private StupidIndexer m_stupidIndexer;

    public StupidShooter(OuttakeRun m_outtakeRun, StupidIndexer m_stupidIndexer) {
        this.m_outtakeRun = m_outtakeRun;
        this.m_stupidIndexer = m_stupidIndexer;
    }

    @Override
    public void initialize(){
        CommandScheduler.getInstance().schedule(m_outtakeRun, m_stupidIndexer);
    }
    
    @Override
    public void end(boolean interrupted){
        CommandScheduler.getInstance().cancel(m_outtakeRun, m_stupidIndexer);
    }
}