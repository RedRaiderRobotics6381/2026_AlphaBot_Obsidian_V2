package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.generated.TunerConstants;

public class Indexer extends SubsystemBase {
  public TalonFX indexMtr;
  public TalonFX uptakeWheelMtr;
  public TalonFX uptakeBeltMtr;
  private TalonFXConfiguration indexMtrCon;
  private VoltageOut voltageCntrl;

  // public DigitalInput FuelSensor;

  public Indexer() {
    indexMtr = new TalonFX(IndexerConstants.INDEXER_MOTOR_PORT, TunerConstants.kCANBus);
    uptakeWheelMtr = new TalonFX(IndexerConstants.UPTAKE_WHEELS_PORT, TunerConstants.kCANBus);
    uptakeBeltMtr = new TalonFX(IndexerConstants.UPTAKE_BELT_PORT, TunerConstants.kCANBus);
    uptakeWheelMtr.setControl(new Follower(IndexerConstants.INDEXER_MOTOR_PORT, MotorAlignmentValue.Aligned));
    uptakeBeltMtr.setControl(new Follower(IndexerConstants.INDEXER_MOTOR_PORT, MotorAlignmentValue.Opposed));

    indexMtrCon = new TalonFXConfiguration();
    voltageCntrl = new VoltageOut(0.0);

    indexMtrCon.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexMtrCon.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    indexMtrCon.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexMtrCon.CurrentLimits.StatorCurrentLimitEnable = true;
    indexMtrCon.CurrentLimits.SupplyCurrentLimit = 80.0;
    indexMtrCon.CurrentLimits.StatorCurrentLimit = 80.0;

    indexMtr.getConfigurator().apply(indexMtrCon);
    uptakeWheelMtr.getConfigurator().apply(indexMtrCon);
    uptakeBeltMtr.getConfigurator().apply(indexMtrCon);
  }
/** Sets the  indexer's voltage.
 * @param volt how many volts you want to set the indexer to.
*/
  public void setVoltage(double volt) {
    indexMtr.setControl(voltageCntrl.withOutput(volt));
  }

      public FunctionalCommand setVoltageCmd(double volt) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setVoltage(volt),
                interrupted -> {
                },
                () -> (Math.abs(- indexMtr.getMotorVoltage().getValueAsDouble()) <= 5000),
                this);
    }
/**Returns a command that starts the indexer with 8 volts and stops the indexer when the command ends
 * @return Command to stop and start the indexer
 */
  public Command runIndexer(){
    return Commands.runEnd(
      () -> setVoltage(10), 
      () -> setVoltage(0), 
      this);
  }
}