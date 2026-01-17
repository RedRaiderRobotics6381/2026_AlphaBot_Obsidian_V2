package frc.robot.subsystems.Secondary;

import frc.robot.Constants.OuttakeConstants;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Outtake extends SubsystemBase {
    public TalonFX wheelSpeedMtr;
    private TalonFXConfiguration wheelSpeedMtrCfg;
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage;

    private double kP = 0.010, kI = 0.0, kD = 0.15;
    public boolean close;

    public Outtake() {
        wheelSpeedMtr = new TalonFX(OuttakeConstants.OUTTAKE_MOTOR_PORT);

        wheelSpeedMtrCfg = new TalonFXConfiguration();
        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withSlot(0);

        wheelSpeedMtrCfg.Slot0.kP = kP;
        wheelSpeedMtrCfg.Slot0.kP = kI;
        wheelSpeedMtrCfg.Slot0.kD = kD;

        wheelSpeedMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelSpeedMtrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        wheelSpeedMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        wheelSpeedMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        wheelSpeedMtrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
        wheelSpeedMtrCfg.CurrentLimits.StatorCurrentLimit = 50.0;

        wheelSpeedMtrCfg.MotionMagic.MotionMagicAcceleration = OuttakeConstants.OUTTAKE_ACCELERATION_CONSTRAINT;

        wheelSpeedMtr.getConfigurator().apply(wheelSpeedMtrCfg);
    }

    public FunctionalCommand setVelocityCmd(double vel) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setVelocity(vel), interrupted -> {
                },
                () -> (Math.abs(vel - wheelSpeedMtr.getVelocity().getValueAsDouble()) <= 50),
                this);
    }

    /** @param velocity in rotations per second */

    public void setVelocity(double velocity) {
        wheelSpeedMtr.setControl(motionMagicVelocityVoltage.withVelocity(velocity));

    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Cool Wheel Velocity", wheelSpeedMtr.getVelocity().getValueAsDouble());
    }
}