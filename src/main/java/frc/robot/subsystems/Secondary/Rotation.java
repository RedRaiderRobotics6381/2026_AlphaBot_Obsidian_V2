package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;

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

    private double kP = 34.0, kI = 0.0, kD = 0.0, kS = 0.4, kFF;
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
        rotationMtrCfg.CurrentLimits.SupplyCurrentLimit = 100.0;
        rotationMtrCfg.CurrentLimits.StatorCurrentLimit = 120.0;

        rotationMtrCfg.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(angToRot(110)) // Larger
                .withReverseSoftLimitThreshold(angToRot(30)));

        rotationMtrCfg.Feedback.SensorToMechanismRatio = 12 / 35;

        rotationMtrCfg.Slot0.kP = kP;
        rotationMtrCfg.Slot0.kI = kI;
        rotationMtrCfg.Slot0.kD = kD;
        rotationMtrCfg.Slot0.kG = kFF;
        rotationMtrCfg.Slot0.kS = kS;
        rotationMtrCfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        rotationMtrCfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        rotationMtrCfg.MotionMagic.MotionMagicAcceleration = RotationConstants.ROTATION_ACCELERATION_CONSTRAINT;
        rotationMtrCfg.MotionMagic.MotionMagicCruiseVelocity = RotationConstants.ROTATION_VELOCITY_CONSTRAINT;
        rotationMtrCfg.MotionMagic.MotionMagicJerk = 9999;

        rotationMtr.getConfigurator().apply(rotationMtrCfg);

        rotationEncoder.getConfigurator().apply(rotEncCfg);
        
    }
    /** sets what angle it's rotated to
     * @param angle the angle that sets how much it rotates
     */
    public void setRotateAngle(double angle) {
        angle = angToRot(angle);
        rotationMtr.setControl(motionMagicVoltage.withPosition(angle));
    }

    /** gets the angle needed to rotate
     * @return the value in rotations
     */
    public double getAngle(){
        return rotToAng(rotationMtr.getPosition().getValueAsDouble());
    }

    /** sets the voltage
     * @param volt the amount of volts to use
     */
    public void setVoltage(double volt) {
        rotationMtr.setControl(voltageCntrl.withOutput(volt));
    }

   /** runs the rotations
    * @return the runEnd Command that runs or ends the rotation 
    */ 
  public Command runRotation(){
    return Commands.runEnd(
      () -> setVoltage(-3), 
      () -> setVoltage(0), 
      this);
  }

  /** runs a command that reverses the rotation
   * @return the runEnd Command that runs or ends the rotation
   */
    public Command runRotationReverse(){
    return Commands.runEnd(
      () -> setVoltage(3), 
      () -> setVoltage(0), 
      this);
  }

/** a command to rotate to a a certain angle
 * @param pos the position that the angle should be in
 * @return a Funtional Command that shows what happens when ran and when interrupted
 */
    public FunctionalCommand setRotateAngleCmd(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setRotateAngle(pos), interrupted -> {
                },
                () -> (Math.abs(pos - rotationMtr.getPosition().getValueAsDouble()) <= 2.0 || (Math.abs(pos - rotationMtr.getPosition().getValueAsDouble()) <= 4.0 && Math.abs(rotationMtr.getVelocity().getValueAsDouble()) <= 5.0)),
                this);
    }

    /** converts rotations to an angle
     * @param rotations rotations in it
     * @return rotations multiplied by the value needed to convert added to the initial angle value
     */
    public double rotToAng(double rotations){
        return (rotations * 360 * 12/35) + RotationConstants.ROTATION_INITIAL_ANGLE;
    }

    /** converts from angles to rotations
     * @param angle angle it wants to be
     * @return angle subtracted by the inital angle and then divided by a converting value
     */
    public double angToRot(double angle){
        return (angle - RotationConstants.ROTATION_INITIAL_ANGLE) / 360
        * 35/12;
    }

    @Override
    public void periodic(){
        angle = rotToAng(rotationMtr.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("angle", angle);

    }
}

