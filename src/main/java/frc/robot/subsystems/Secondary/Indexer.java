package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  public TalonFX indexMtr;
  private TalonFXConfiguration indexMtrCon;
  private VoltageOut voltageCntrl;

  // public DigitalInput FuelSensor;

  public Indexer() {
    indexMtr = new TalonFX(IndexerConstants.INDEXER_MOTOR_PORT);

    indexMtrCon = new TalonFXConfiguration();
    voltageCntrl = new VoltageOut(0.0);

      //coralSensor = new DigitalInput(CoralConstants.BEAM_BREAK_SENSOR_PORT);

    indexMtrCon.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexMtrCon.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    indexMtrCon.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexMtrCon.CurrentLimits.StatorCurrentLimitEnable = true;
    indexMtrCon.CurrentLimits.SupplyCurrentLimit = 30.0;
    indexMtrCon.CurrentLimits.StatorCurrentLimit = 50.0;

    indexMtr.getConfigurator().apply(indexMtrCon);
  }


    public FunctionalCommand setVoltageCmd(double volt) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setVoltage(volt), interrupted -> {
                },
                () -> (Math.abs(- indexMtr.getMotorVoltage().getValueAsDouble()) <= 0.25),
                this);
    }

  public void setVoltage(double volt) {
    indexMtr.setControl(voltageCntrl.withOutput(volt));
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Outtake Speed", indexMtr.getVelocity().getValueAsDouble());
      // SmartDashboard.putBoolean("CoralSensor", FuelSensor.get());
  }
}