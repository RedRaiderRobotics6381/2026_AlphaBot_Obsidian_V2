package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.Intake;
import frc.robot.subsystems.Secondary.IntakeSlider;

public class IntakeRun extends Command {
    private Intake m_intake;
    private IntakeSlider m_intakeSlider;

    public IntakeRun(Intake m_intake, IntakeSlider m_intakeSlider) {
        this.m_intake = m_intake;
        this.m_intakeSlider = m_intakeSlider;
    }

    @Override
    public void initialize(){
        m_intakeSlider.setRotateAngle();
        m_intake.runIntake();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
} 
