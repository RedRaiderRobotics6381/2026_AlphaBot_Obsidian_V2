package frc.robot.subsystems.Secondary;

import frc.robot.Constants.ConstantValues;
import frc.robot.Constants.IntakeSliderConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Outtake extends SubsystemBase {
    public TalonFX wheelSpeedUppMtr;
    public TalonFX wheelSpeedLowMtr;
    private TalonFXConfiguration wheelSpeedMtrCfg;
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private VoltageOut voltageCntrl;
    public boolean run;
    public static boolean atSpeed;

    private double kP = 1.0, kI = 0.0, kD = 0.0, kS = 0.23, kV = 0.24;
    public boolean close;

    public Outtake() {
        wheelSpeedUppMtr = new TalonFX(OuttakeConstants.OUTTAKE_MOTOR_UPPER_PORT, TunerConstants.kCANBus);
        wheelSpeedLowMtr = new TalonFX(OuttakeConstants.OUTTAKE_MOTOR_LOWER_PORT, TunerConstants.kCANBus);
        wheelSpeedLowMtr.setControl(new Follower(OuttakeConstants.OUTTAKE_MOTOR_UPPER_PORT, MotorAlignmentValue.Aligned));

        wheelSpeedMtrCfg = new TalonFXConfiguration();
        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withSlot(0);
        voltageCntrl = new VoltageOut(0.0);

        wheelSpeedMtrCfg.Slot0.kP = kP;
        wheelSpeedMtrCfg.Slot0.kI = kI;
        wheelSpeedMtrCfg.Slot0.kD = kD;
        wheelSpeedMtrCfg.Slot0.kS = kS;
        wheelSpeedMtrCfg.Slot0.kV = kV;

        wheelSpeedMtrCfg.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2));

        wheelSpeedMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelSpeedMtrCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        wheelSpeedMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        wheelSpeedMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        wheelSpeedMtrCfg.CurrentLimits.SupplyCurrentLimit = 100.0;
        wheelSpeedMtrCfg.CurrentLimits.StatorCurrentLimit = 200.0;

        wheelSpeedMtrCfg.MotionMagic.MotionMagicAcceleration = OuttakeConstants.OUTTAKE_ACCELERATION_CONSTRAINT;

        wheelSpeedUppMtr.getConfigurator().apply(wheelSpeedMtrCfg);
        wheelSpeedLowMtr.getConfigurator().apply(wheelSpeedMtrCfg);
    }


    /** makes the outtake run when true, makes it not run when false by using 0 volts
     * @param speed in rotations per second 
     */
    public void runOuttake(double speed) {
        if (!run) {
            run = true;
            wheelSpeedUppMtr.setControl(motionMagicVelocityVoltage.withVelocity(speed));
        } else {
            run = false;
            wheelSpeedUppMtr.setControl(voltageCntrl.withOutput(0));
        }

    }

    /** sets the outtake velocity using speed
     * @param speed the amount of speed used
     */
    public void setVelocity(double speed){
        wheelSpeedUppMtr.setControl(motionMagicVelocityVoltage.withVelocity(speed/2));
    }
    public void setVoltage(double speed){
        wheelSpeedUppMtr.setControl(voltageCntrl.withOutput(speed));
    }
    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Outtake On", wheelSpeedMtr.getVelocity().getValueAsDouble() > 1);
        if(CommandSwerveDrivetrain.distanceToHub < ConstantValues.DISTANCE_TO_SHOOT){
            atSpeed = Math.abs(wheelSpeedUppMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPS_NEAR/2) < 1;
        } else {
            atSpeed = Math.abs(wheelSpeedUppMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOTER_RPS_FAR/2) < 1;
        }
        SmartDashboard.putNumber("Cool Wheel Velocity", wheelSpeedUppMtr.getVelocity().getValueAsDouble());
    }
}