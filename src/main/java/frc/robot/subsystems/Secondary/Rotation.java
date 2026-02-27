package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class Rotation extends SubsystemBase {   
    public TalonFX rotationMtr;
    public CANcoder rotationEncoder;
    public CANcoderConfiguration rotEncCfg;
    private TalonFXConfiguration rotationMtrCfg;
    private VoltageOut voltageCntrl;
    private MotionMagicVoltage motionMagicVoltage;
    double angle;

    private double kP = 10.00, kI = 0.0, kD = 0.0,  kFF;
    public boolean close;

    public Rotation() {
        rotationMtr = new TalonFX(RotationConstants.ROTATION_MOTOR_PORT, TunerConstants.kCANBus);
        rotationEncoder = new CANcoder(5, TunerConstants.kCANBus);
        voltageCntrl = new VoltageOut(0.0);
        rotationMtrCfg = new TalonFXConfiguration();
        rotEncCfg = new CANcoderConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);
        rotEncCfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        rotationMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotationMtrCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rotationMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotationMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        rotationMtrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
        rotationMtrCfg.CurrentLimits.StatorCurrentLimit = 50.0;

        rotationMtrCfg.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(angToRot(93))
                .withReverseSoftLimitThreshold(angToRot(30)));

        rotationMtrCfg.Feedback.SensorToMechanismRatio = 12 / 35;

        rotationMtrCfg.Slot0.kP = kP;
        rotationMtrCfg.Slot0.kI = kI;
        rotationMtrCfg.Slot0.kD = kD;
        rotationMtrCfg.Slot0.kG = kFF;
        rotationMtrCfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        rotationMtrCfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        rotationMtrCfg.MotionMagic.MotionMagicAcceleration = RotationConstants.ROTATION_ACCELERATION_CONSTRAINT;
        rotationMtrCfg.MotionMagic.MotionMagicCruiseVelocity = RotationConstants.ROTATION_VELOCITY_CONSTRAINT;
        rotationMtrCfg.MotionMagic.MotionMagicJerk = 9999;

        rotationMtr.getConfigurator().apply(rotationMtrCfg);

        rotationEncoder.getConfigurator().apply(rotEncCfg);
        
    }

    public void setRotateAngle(double angle) {
        angle = angToRot(angle);
        rotationMtr.setControl(motionMagicVoltage.withPosition(angle));
    }

    public void setVoltage(double volt) {
        rotationMtr.setControl(voltageCntrl.withOutput(volt));
    }

  public Command runRotation(){
    return Commands.runEnd(
      () -> setVoltage(-3), 
      () -> setVoltage(0), 
      this);
  }

    public Command runRotationReverse(){
    return Commands.runEnd(
      () -> setVoltage(3), 
      () -> setVoltage(0), 
      this);
  }


    public FunctionalCommand setRotateAngleCmd(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setRotateAngle(pos), interrupted -> {
                },
                () -> (Math.abs(pos - rotationMtr.getPosition().getValueAsDouble()) <= 2.0 || (Math.abs(pos - rotationMtr.getPosition().getValueAsDouble()) <= 4.0 && Math.abs(rotationMtr.getVelocity().getValueAsDouble()) <= 5.0)),
                this);
    }

    public double rotToAng(double rotations){
        return (rotations * 360 * 12/35) + RotationConstants.ROTATION_INITIAL_ANGLE;
    }

    public double angToRot(double angle){
        return (angle - RotationConstants.ROTATION_INITIAL_ANGLE) / 360
        * 35/12;
    }

    @Override
    public void periodic(){
        angle = rotToAng(rotationMtr.getPosition().getValueAsDouble());
        //rotationMtr.setPosition(rotationEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("relative Postion", rotationMtr.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("absolute Postion", angToRot(45));
        SmartDashboard.putNumber("angle", angle);

    }
}

