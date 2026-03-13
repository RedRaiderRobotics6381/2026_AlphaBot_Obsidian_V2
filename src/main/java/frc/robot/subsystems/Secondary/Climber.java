package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {   
    public TalonFX climberMtr;
    private TalonFXConfiguration climberMtrCfg;
    private VoltageOut voltageCntrl;
    private MotionMagicVoltage motionMagicVoltage;
    double distance;
    double distanceMtr;
    public boolean out;

    private double kP = 0.0, kI = 0.0, kD = 0.0;
    public boolean close;

    public Climber() {
        climberMtr = new TalonFX(ClimberConstants.CLIMBER_MOTOR_PORT, TunerConstants.kCANBus);

        voltageCntrl = new VoltageOut(0.0);
        climberMtrCfg = new TalonFXConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

        climberMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMtrCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climberMtrCfg.Feedback.SensorToMechanismRatio = 45/1;

        climberMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        climberMtrCfg.CurrentLimits.SupplyCurrentLimit = 20.0;
        climberMtrCfg.CurrentLimits.StatorCurrentLimit = 80.0;

        climberMtrCfg.Slot0.kP = kP;
        climberMtrCfg.Slot0.kI = kI;
        climberMtrCfg.Slot0.kD = kD;

        climberMtrCfg.MotionMagic.MotionMagicAcceleration = IntakeSliderConstants.SLIDER_ACCELERATION_CONSTRAINT;
        climberMtrCfg.MotionMagic.MotionMagicCruiseVelocity = IntakeSliderConstants.SLIDER_VELOCITY_CONSTRAINT;

        climberMtr.getConfigurator().apply(climberMtrCfg);
    }

    public void setRotateAngle() {
        if (!out) {
            out = true;
            climberMtr.setControl(motionMagicVoltage.withPosition(distToRev(13.5)));
        } else {
            out = false;
            climberMtr.setControl(motionMagicVoltage.withPosition(0));
        }
    }

    public void setVoltage(double volt) {
        climberMtr.setControl(voltageCntrl.withOutput(volt));
    }

    public double revToDist(double revolutions){
        return revolutions * Math.PI * 1.5; 
    }

    public double distToRev(double distance){
        return distance / (Math.PI * 1.5);
    }

  public Command runClimberUp(){
    return Commands.runEnd(
      () -> setVoltage(-8), 
      () -> setVoltage(0), 
      this);
  }
  public Command runClimberDown(){
    return Commands.runEnd(
      () -> setVoltage(4), 
      () -> setVoltage(0), 
      this);
  }
}