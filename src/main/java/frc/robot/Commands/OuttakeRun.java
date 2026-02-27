package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Intake;
import frc.robot.subsystems.Secondary.IntakeSlider;
import frc.robot.subsystems.Secondary.Outtake;

public class OuttakeRun extends Command {
    private Outtake m_outtake;
    private Intake m_intake;
    private IntakeSlider m_intakeSlider;
    private boolean on;

    public OuttakeRun(Outtake m_outtake, Intake m_intake, IntakeSlider m_intakeSlider) {
        this.m_intake = m_intake;
        this.m_outtake = m_outtake;
        this.m_intakeSlider = m_intakeSlider;
        on = false;
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    @Override
    public void initialize(){
        m_outtake.setVelocity();
        if(m_intake.intakeOn) {
            m_intake.runIntake();
        } else if(m_intake.reverseIntakeOn) {
            m_intake.runReverseIntake();
        } else if(m_intakeSlider.out) {
             m_intake.runIntake();
        }
    }

} 
