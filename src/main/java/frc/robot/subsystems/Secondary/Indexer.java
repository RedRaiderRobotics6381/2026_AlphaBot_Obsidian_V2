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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  public TalonFX indexMtrLdr;
  private TalonFX indexMtrFlw;
  private TalonFXConfiguration indexMtrLdrCon;
  private VoltageOut voltageCntrl;

  public DigitalInput FuelSensor;

  public Indexer() {
    indexMtrLdr = new TalonFX(IndexerConstants.INDEXER_MOTOR_PORT_LDR);
    indexMtrFlw = new TalonFX(IndexerConstants.INDEXER_MOTOR_PORT_FLW);
    indexMtrFlw.setControl(new Follower(IndexerConstants.INDEXER_MOTOR_PORT_LDR, MotorAlignmentValue.Aligned));

    indexMtrLdrCon = new TalonFXConfiguration();
    voltageCntrl = new VoltageOut(0.0);

      //coralSensor = new DigitalInput(CoralConstants.BEAM_BREAK_SENSOR_PORT);

    indexMtrLdrCon.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexMtrLdrCon.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    indexMtrLdrCon.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexMtrLdrCon.CurrentLimits.StatorCurrentLimitEnable = true;
    indexMtrLdrCon.CurrentLimits.SupplyCurrentLimit = 30.0;
    indexMtrLdrCon.CurrentLimits.StatorCurrentLimit = 50.0;

    indexMtrLdrCon.Slot0.kP = 5.0;
    indexMtrLdrCon.Slot0.kI = 0;
    indexMtrLdrCon.Slot0.kD = 0;

    indexMtrLdr.getConfigurator().apply(indexMtrLdrCon);
    indexMtrFlw.getConfigurator().apply(indexMtrLdrCon);
  }

  public void setVoltage(double volt) {
    indexMtrLdr.setControl(voltageCntrl.withOutput(volt));
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Outtake Speed", indexMtrLdr.getVelocity().getValueAsDouble());
      SmartDashboard.putBoolean("CoralSensor", FuelSensor.get());
  }
}