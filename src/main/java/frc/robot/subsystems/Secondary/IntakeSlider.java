package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class IntakeSlider extends SubsystemBase {   
    public TalonFX sliderLdrMtr;
    public TalonFX sliderFlwMtr;
    public CANcoder sliderEncoder;
    private CANcoderConfiguration sliEncCfg;
    private TalonFXConfiguration sliderLdrMtrCfg;
    private VoltageOut voltageCntrl;
    private MotionMagicVoltage motionMagicVoltage;
    double distance;
    double distanceMtr;
    public boolean out;

    private double kP = 6.38, kI = 0.0, kD = 0.0;
    public boolean close;

    public IntakeSlider() {
        sliderLdrMtr = new TalonFX(IntakeSliderConstants.INTAKE_SLIDER_MOTOR_PORT_LDR, TunerConstants.kCANBus);
        sliderFlwMtr = new TalonFX(IntakeSliderConstants.INTAKE_SLIDER_MOTOR_PORT_FLW, TunerConstants.kCANBus);
        sliderFlwMtr.setControl(new Follower(IntakeSliderConstants.INTAKE_SLIDER_MOTOR_PORT_LDR, MotorAlignmentValue.Opposed));
        sliderEncoder = new CANcoder(4, TunerConstants.kCANBus);

        voltageCntrl = new VoltageOut(0.0);
        sliderLdrMtrCfg = new TalonFXConfiguration();
        sliEncCfg = new CANcoderConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

        sliderLdrMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        sliderLdrMtrCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//         sliderLdrMtrCfg.withDifferentialSensors(
//             new DifferentialSensorsConfigs()
//             .withDifferentialRemoteSensorID(4)
//             .withDifferentialSensorSource(DifferentialSensorSourceValue.RemoteCANcoder)
// );
        sliderLdrMtrCfg.withFeedback(new FeedbackConfigs().withRemoteCANcoder(sliderEncoder).withRotorToSensorRatio(3));

        sliderLdrMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        sliderLdrMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        sliderLdrMtrCfg.CurrentLimits.SupplyCurrentLimit = 20.0;
        sliderLdrMtrCfg.CurrentLimits.StatorCurrentLimit = 80.0;

        sliEncCfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        sliderLdrMtrCfg.Slot0.kP = kP;
        sliderLdrMtrCfg.Slot0.kI = kI;
        sliderLdrMtrCfg.Slot0.kD = kD;

        sliderLdrMtrCfg.MotionMagic.MotionMagicAcceleration = IntakeSliderConstants.SLIDER_ACCELERATION_CONSTRAINT;
        sliderLdrMtrCfg.MotionMagic.MotionMagicCruiseVelocity = IntakeSliderConstants.SLIDER_VELOCITY_CONSTRAINT;

        sliderLdrMtr.getConfigurator().apply(sliderLdrMtrCfg);
        sliderFlwMtr.getConfigurator().apply(sliderLdrMtrCfg);
        sliderEncoder.getConfigurator().apply(sliEncCfg);
    }

    public void setRotateAngle() {
        if (!out) {
            out = true;
            sliderLdrMtr.setControl(motionMagicVoltage.withPosition(distToRev(13)));
        } else {
            out = false;
            sliderLdrMtr.setControl(motionMagicVoltage.withPosition(0));
        }
        
    }

    public void setVoltage(double volt) {
        sliderLdrMtr.setControl(voltageCntrl.withOutput(volt));
    }

    public double revToDist(double revolutions){
        return revolutions * Math.PI * 1.5; 
    }

    public double distToRev(double distance){
        return distance / (Math.PI * 1.5);
    }


  public Command runSlider(){
    return Commands.runEnd(
      () -> setVoltage(1), 
      () -> setVoltage(0), 
      this);
  }

    // public FunctionalCommand setRotateAngleCmd(double pos) {
    //     return new FunctionalCommand(
    //             () -> {
    //             },
    //             () -> setRotateAngle(pos), interrupted -> {
    //             },
    //             () -> (Math.abs(pos - sliderLdrMtr.getPosition().getValueAsDouble()) <= 2.0 || (Math.abs(pos - sliderLdrMtr.getPosition().getValueAsDouble()) <= 4.0 && Math.abs(sliderLdrMtr.getVelocity().getValueAsDouble()) <= 5.0)),
    //             this);
    // }
    @Override
    public void periodic(){
        // distance = revToDist(sliderEncoder.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("distance", distance);
    }
}

