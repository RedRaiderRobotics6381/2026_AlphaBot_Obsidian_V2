package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public TalonFXS intMtrFrnt;
    public TalonFXS intMtrBck;
    private TalonFXSConfiguration intVelMtrLdrCfg;
    private VoltageOut voltageCntrl;

    public boolean close;

public Intake() {
    intMtrFrnt = new TalonFXS(IntakeConstants.INTAKE_MOTOR_PORT_FRONT);
    intMtrBck = new TalonFXS(IntakeConstants.INTAKE_MOTOR_PORT_BACK);
    intMtrBck.setControl(new Follower(IntakeConstants.INTAKE_MOTOR_PORT_FRONT, MotorAlignmentValue.Aligned));


    intVelMtrLdrCfg = new TalonFXSConfiguration();
    voltageCntrl = new VoltageOut(0.0);

    intVelMtrLdrCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intVelMtrLdrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intVelMtrLdrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    intVelMtrLdrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    intVelMtrLdrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    intVelMtrLdrCfg.CurrentLimits.StatorCurrentLimit = 50.0;


    intMtrFrnt.getConfigurator().apply(intVelMtrLdrCfg);
    intMtrBck.getConfigurator().apply(intVelMtrLdrCfg);
}

    public FunctionalCommand setVoltageCmd(double volt) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setVoltage(volt), interrupted -> {
                },
                () -> (Math.abs(- intMtrFrnt.getMotorVoltage().getValueAsDouble()) <= 50),
                this);
    }

public void setVoltage(double volt) {
    intMtrFrnt.setControl(voltageCntrl.withOutput(volt));
}
}