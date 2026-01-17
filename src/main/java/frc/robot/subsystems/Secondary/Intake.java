package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public TalonFX intVelMtr;
    private TalonFXConfiguration intVelMtrCfg;
    private VoltageOut voltageCntrl;

    private double kP = 1.00, kI = 0.0, kD = 0.00;
    public boolean close;

public Intake() {
    intVelMtr = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT);

    intVelMtrCfg = new TalonFXConfiguration();
    voltageCntrl = new VoltageOut(0.0);

    intVelMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intVelMtrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intVelMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    intVelMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    intVelMtrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    intVelMtrCfg.CurrentLimits.StatorCurrentLimit = 50.0;

    intVelMtrCfg.Slot0.kP = kP;
    intVelMtrCfg.Slot0.kI = kI;
    intVelMtrCfg.Slot0.kD = kD;

    intVelMtr.getConfigurator().apply(intVelMtrCfg);
}

public void setVoltage(double volt) {
    intVelMtr.setControl(voltageCntrl.withOutput(volt));
}
}