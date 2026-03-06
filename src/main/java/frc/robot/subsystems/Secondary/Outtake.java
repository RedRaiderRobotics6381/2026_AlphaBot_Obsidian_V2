package frc.robot.subsystems.Secondary;

import frc.robot.Constants.OuttakeConstants;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Outtake extends SubsystemBase {
    public TalonFX wheelSpeedMtr;
    private TalonFXConfiguration wheelSpeedMtrCfg;
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage;
    private VoltageOut voltageCntrl;
    public boolean run;

    private double kP = 1.0, kI = 0.0, kD = 0.0, kS = 0.42, kV = 0.13;
    public boolean close;

    public Outtake() {
        wheelSpeedMtr = new TalonFX(OuttakeConstants.OUTTAKE_MOTOR_PORT, TunerConstants.kCANBus);

        wheelSpeedMtrCfg = new TalonFXConfiguration();
        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withSlot(0);
        voltageCntrl = new VoltageOut(0.0);

        wheelSpeedMtrCfg.Slot0.kP = kP;
        wheelSpeedMtrCfg.Slot0.kI = kI;
        wheelSpeedMtrCfg.Slot0.kD = kD;
        wheelSpeedMtrCfg.Slot0.kS = kS;
        wheelSpeedMtrCfg.Slot0.kV = kV;

        wheelSpeedMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wheelSpeedMtrCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        wheelSpeedMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        wheelSpeedMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        wheelSpeedMtrCfg.CurrentLimits.SupplyCurrentLimit = 100.0;
        wheelSpeedMtrCfg.CurrentLimits.StatorCurrentLimit = 200.0;

        wheelSpeedMtrCfg.MotionMagic.MotionMagicAcceleration = OuttakeConstants.OUTTAKE_ACCELERATION_CONSTRAINT;

        wheelSpeedMtr.getConfigurator().apply(wheelSpeedMtrCfg);
    }


    /** makes the outtake run when true, makes it not run when false by using 0 volts
     * @param speed in rotations per second 
     */
    public void runOuttake(double speed) {
        if (!run) {
            run = true;
            wheelSpeedMtr.setControl(motionMagicVelocityVoltage.withVelocity(speed));
        } else {
            run = false;
            wheelSpeedMtr.setControl(voltageCntrl.withOutput(0));
        }

    }

    /** sets the outtake velocity using speed
     * @param speed the amount of speed used
     */
    public void setVelocity(double speed){
        wheelSpeedMtr.setControl(motionMagicVelocityVoltage.withVelocity(speed));
    }
    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Outtake On", wheelSpeedMtr.getVelocity().getValueAsDouble() > 1);
        SmartDashboard.putNumber("Cool Wheel Velocity", wheelSpeedMtr.getVelocity().getValueAsDouble());
    }
}