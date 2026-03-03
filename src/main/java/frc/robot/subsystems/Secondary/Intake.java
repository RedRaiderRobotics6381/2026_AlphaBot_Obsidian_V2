package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class Intake extends SubsystemBase {
    public TalonFXS intMtrFrnt;
    private TalonFXSConfiguration intVelMtrLdrCfg;
    private VoltageOut voltageCntrl;
    public boolean intakeOn;
    public boolean reverseIntakeOn;

public Intake() {
    intakeOn = false;
    intMtrFrnt = new TalonFXS(IntakeConstants.INTAKE_MOTOR_PORT_1, TunerConstants.kCANBus);
    
    //intMtrBck.setControl(new Follower(IntakeConstants.INTAKE_MOTOR_PORT_1, MotorAlignmentValue.Aligned));

    intVelMtrLdrCfg = new TalonFXSConfiguration();
    intVelMtrLdrCfg.Commutation.MotorArrangement = MotorArrangementValue.VORTEX_JST;
    voltageCntrl = new VoltageOut(0.0);

    intVelMtrLdrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intVelMtrLdrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intVelMtrLdrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    intVelMtrLdrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    intVelMtrLdrCfg.CurrentLimits.SupplyCurrentLimit = 400.0;
    intVelMtrLdrCfg.CurrentLimits.StatorCurrentLimit = 800.0;


    intMtrFrnt.getConfigurator().apply(intVelMtrLdrCfg);
}

/** Functional Command that returns another command that sets the voltage
 * @param volt how many volts it's set to
 * @return Functional Command
 */
    public FunctionalCommand setVoltageCmd(double volt) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setVoltage(volt),
                interrupted -> {
                },
                () -> (Math.abs(- intMtrFrnt.getMotorVoltage().getValueAsDouble()) <= 5000),
                this);
    }

    
/** sets the indexer's voltage
 * @param volt how many volts it's set to
 */
public void setVoltage(double volt) {
    intMtrFrnt.setControl(voltageCntrl.withOutput(Math.signum(volt) * (Math.abs(volt) + 3)));
}

/** runs the intake, when intake is off, it uses zero volts, and when intake is on, it uses 9 volts while the reverse intake turns off
 */
public void runIntake(){
    if(intakeOn){
        intakeOn = false;
        setVoltage(0);
    } else {
        intakeOn = true;
        reverseIntakeOn = false;
        setVoltage(9);
    }
}
/** runs the reverse intake, uses no volts whens the reverse intake is off, and uses 9 volts and runs in reverse when reverse intake is on and the intake is off 
 */
public void runReverseIntake(){
    if(reverseIntakeOn){
        reverseIntakeOn = false;
        setVoltage(0);
    } else {
        reverseIntakeOn = true;
        intakeOn = false;
        setVoltage(-9);
    }
}
}