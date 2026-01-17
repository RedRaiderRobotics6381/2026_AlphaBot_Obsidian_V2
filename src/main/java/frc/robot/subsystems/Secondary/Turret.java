
package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Turret extends SubsystemBase{  
    public TalonFX turretAngMtr;
    private TalonFXConfiguration turretAngMtrCfg;
    private MotionMagicVoltage motionMagicVoltage;

    private double kP = 0.010, kI = 0.0, kD = 0.15;
    public boolean close;

    public Turret() {
        turretAngMtr = new TalonFX(TurretConstants.TURRET_MOTOR_PORT);
        turretAngMtrCfg = new TalonFXConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

        turretAngMtrCfg.Slot0.kP = kP;
        turretAngMtrCfg.Slot0.kP = kI;
        turretAngMtrCfg.Slot0.kD = kD;

        turretAngMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretAngMtrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        turretAngMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretAngMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        turretAngMtrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
        turretAngMtrCfg.CurrentLimits.StatorCurrentLimit = 50.0;

        turretAngMtrCfg.MotionMagic.MotionMagicAcceleration = TurretConstants.TURRET_ACCELERATION_CONSTRAINT;
        turretAngMtrCfg.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.TURRET_VELOCITY_CONSTRAINT;

        turretAngMtr.getConfigurator().apply(turretAngMtrCfg);
    }

    public void setTurret(double angle) {
        turretAngMtr.setControl(motionMagicVoltage.withPosition(angle));
    }

    public FunctionalCommand setTurretCmd(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setTurret(pos), interrupted -> {
                },
                () -> (Math.abs(pos - turretAngMtr.getPosition().getValueAsDouble()) <= 2.0 || (Math.abs(pos - turretAngMtr.getPosition().getValueAsDouble()) <= 4.0 && Math.abs(turretAngMtr.getVelocity().getValueAsDouble()) <= 5.0)),
                this);
    }
}