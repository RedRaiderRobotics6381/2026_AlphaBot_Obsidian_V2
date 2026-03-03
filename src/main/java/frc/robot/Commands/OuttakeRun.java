package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Intake;
import frc.robot.subsystems.Secondary.IntakeSlider;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class OuttakeRun extends Command {
    private Outtake m_outtake;
    private Intake m_intake;
    private IntakeSlider m_intakeSlider;
    CommandSwerveDrivetrain drivetrain;
    double distance;

    public OuttakeRun(Outtake m_outtake, Intake m_intake, IntakeSlider m_intakeSlider, CommandSwerveDrivetrain drivetrain) {
        this.m_intake = m_intake;
        this.m_outtake = m_outtake;
        this.m_intakeSlider = m_intakeSlider;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize(){
        if(m_intake.intakeOn) {
            m_intake.runIntake();
        } else if(m_intake.reverseIntakeOn) {
            m_intake.runReverseIntake();
        } else if(m_intakeSlider.out) {
             m_intake.runIntake();
        }
    }

    @Override
    public void execute(){
        if(drivetrain.distanceToHub < 70){
            m_outtake.setVelocity(ConstantValues.SHOOTER_RPS_NEAR);
        }else {
            m_outtake.setVelocity(ConstantValues.SHOOTER_RPS_FAR);
        }
    }
    @Override
    public void end(boolean interrupted){
        m_outtake.setVelocity(ConstantValues.OUTTAKE_IDLE_SPEED);
    }
} 
