package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Rotation;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAimer extends Command {
    private Rotation m_rotate;
    private double lowerBoundAngle;
    private double upperBoundAngle;
    private double angle;
    public static boolean atAngle;

    public AutoAimer(Rotation m_rotate) {
        this.m_rotate = m_rotate;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(CommandSwerveDrivetrain.distanceToHub < ConstantValues.DISTANCE_TO_SHOOT){
            lowerBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED_NEAR, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED_NEAR,4) - 
                          384 * (384 * Math.pow(CommandSwerveDrivetrain.distanceToHub - FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED_NEAR, 2)))) / 
                          (384 * (CommandSwerveDrivetrain.distanceToHub - FieldConstants.SMALLEST_RADIUS_OF_HOLE))) * 180 / Math.PI;
    

              
            upperBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED_NEAR, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED_NEAR,4) - 
                          384 * (384 * Math.pow(CommandSwerveDrivetrain.distanceToHub + FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED_NEAR, 2)))) / 
                          (384 * (CommandSwerveDrivetrain.distanceToHub + FieldConstants.SMALLEST_RADIUS_OF_HOLE))) * 180 / Math.PI;
        } else {
            lowerBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED_FAR, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED_FAR,4) - 
                          384 * (384 * Math.pow(CommandSwerveDrivetrain.distanceToHub - FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED_FAR, 2)))) / 
                          (384 * (CommandSwerveDrivetrain.distanceToHub - FieldConstants.SMALLEST_RADIUS_OF_HOLE))) * 180 / Math.PI;
    

            
            upperBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED_FAR, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED_FAR,4) - 
                          384 * (384 * Math.pow(CommandSwerveDrivetrain.distanceToHub + FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED_FAR, 2)))) / 
                          (384 * (CommandSwerveDrivetrain.distanceToHub + FieldConstants.SMALLEST_RADIUS_OF_HOLE))) * 180 / Math.PI;
        }
        angle = (lowerBoundAngle + upperBoundAngle) / 2;
        m_rotate.setRotateAngle(angle);
        atAngle = Math.abs(m_rotate.getAngle() - angle) < 0.5;
        SmartDashboard.putNumber("set angle", angle);
        SmartDashboard.putBoolean("atAngle", atAngle);
    }

    @Override
    public void end(boolean interrupted){
        m_rotate.setVoltage(0);
    }
}