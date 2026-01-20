package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public TalonFX intVelMtrLdr;
    public TalonFX intVelMtrFlw;
    private TalonFXConfiguration intVelMtrLdrCfg;
    private VoltageOut voltageCntrl;

    private double kP = 1.00, kI = 0.0, kD = 0.00;
    public boolean close;

public Intake() {
    intVelMtrLdr = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT_LDR);
    intVelMtrFlw = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT_FLW);
    intVelMtrFlw.setControl(new Follower(IntakeConstants.INTAKE_MOTOR_PORT_LDR, MotorAlignmentValue.Aligned));


    intVelMtrLdrCfg = new TalonFXConfiguration();
    voltageCntrl = new VoltageOut(0.0);

    intVelMtrLdrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intVelMtrLdrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intVelMtrLdrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    intVelMtrLdrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    intVelMtrLdrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    intVelMtrLdrCfg.CurrentLimits.StatorCurrentLimit = 50.0;

    intVelMtrLdrCfg.Slot0.kP = kP;
    intVelMtrLdrCfg.Slot0.kI = kI;
    intVelMtrLdrCfg.Slot0.kD = kD;

    intVelMtrLdr.getConfigurator().apply(intVelMtrLdrCfg);
    intVelMtrFlw.getConfigurator().apply(intVelMtrLdrCfg);
}

public void setVoltage(double volt) {
    intVelMtrLdr.setControl(voltageCntrl.withOutput(volt));
}
}