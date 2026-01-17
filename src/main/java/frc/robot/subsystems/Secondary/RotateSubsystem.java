package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RotateSubsystem extends SubsystemBase {   
    public TalonFX armAngMtr;
    private TalonFXConfiguration armAngMtrCfg;
    private MotionMagicVoltage motionMagicVoltage;

    private double kP = 0.010, kI = 0.0, kD = 0.15,  kFF = 0.0;
    public boolean close;

    public RotateSubsystem() {
        armAngMtr = new TalonFX(RotateConstants.ROTATE_MOTOR_PORT);
        armAngMtrCfg = new TalonFXConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

        armAngMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armAngMtrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        armAngMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        armAngMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        armAngMtrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
        armAngMtrCfg.CurrentLimits.StatorCurrentLimit = 50.0;

        armAngMtrCfg.Slot0.kP = kP;
        armAngMtrCfg.Slot0.kI = kI;
        armAngMtrCfg.Slot0.kD = kD;

        armAngMtrCfg.MotionMagic.MotionMagicAcceleration = RotateConstants.ROTATE_ACCELERATION_CONSTRAINT;
        armAngMtrCfg.MotionMagic.MotionMagicCruiseVelocity = RotateConstants.ROTATE_VELOCITY_CONSTRAINT;

        armAngMtr.getConfigurator().apply(armAngMtrCfg);
    }

    public void setRotateAngle(double angle) {
        armAngMtr.setControl(motionMagicVoltage.withPosition(angle));
    }

    public FunctionalCommand setRotateAngleCmd(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setRotateAngle(pos), interrupted -> {
                },
                () -> (Math.abs(pos - armAngMtr.getPosition().getValueAsDouble()) <= 2.0 || (Math.abs(pos - armAngMtr.getPosition().getValueAsDouble()) <= 4.0 && Math.abs(armAngMtr.getVelocity().getValueAsDouble()) <= 5.0)),
                this);
    }

    public void periodic() {
            SmartDashboard.putNumber("Fuel Arm Position", armAngMtr.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Fuel Arm Speed", armAngMtr.getVelocity().getValueAsDouble());
    }
}

