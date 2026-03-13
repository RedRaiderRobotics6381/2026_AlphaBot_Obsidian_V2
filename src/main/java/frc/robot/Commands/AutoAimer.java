package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Rotation;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAimer extends Command {
    private Rotation m_rotate;
    private CommandSwerveDrivetrain drivetrain;
    private double lowerBoundAngle;
    private double upperBoundAngle;
    private double angle;
    public boolean atAngle;

    public AutoAimer(Rotation m_rotate, CommandSwerveDrivetrain drivetrain) {
        this.m_rotate = m_rotate;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(drivetrain.distanceToHub < ConstantValues.DISTANCE_TO_SHOOT){
            lowerBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED_NEAR, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED_NEAR,4) - 
                          384 * (384 * Math.pow(drivetrain.distanceToHub - FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED_NEAR, 2)))) / 
                          (384 * (drivetrain.distanceToHub - FieldConstants.SMALLEST_RADIUS_OF_HOLE))) * 180 / Math.PI;
    

              
            upperBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED_NEAR, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED_NEAR,4) - 
                          384 * (384 * Math.pow(drivetrain.distanceToHub + FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED_NEAR, 2)))) / 
                          (384 * (drivetrain.distanceToHub + FieldConstants.SMALLEST_RADIUS_OF_HOLE))) * 180 / Math.PI;
        } else {
            lowerBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED_FAR, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED_FAR,4) - 
                          384 * (384 * Math.pow(drivetrain.distanceToHub - FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED_FAR, 2)))) / 
                          (384 * (drivetrain.distanceToHub - FieldConstants.SMALLEST_RADIUS_OF_HOLE))) * 180 / Math.PI;
    

            
            upperBoundAngle = Math.atan((Math.pow(ConstantValues.SHOOTER_SPEED_FAR, 2) + 
                          Math.sqrt(Math.pow(ConstantValues.SHOOTER_SPEED_FAR,4) - 
                          384 * (384 * Math.pow(drivetrain.distanceToHub + FieldConstants.SMALLEST_RADIUS_OF_HOLE, 2) + 
                          2 * (FieldConstants.HEIGHT_OF_HOLE - PhysicalConstants.SHOOTER_HEIGHT) * 
                          Math.pow(ConstantValues.SHOOTER_SPEED_FAR, 2)))) / 
                          (384 * (drivetrain.distanceToHub + FieldConstants.SMALLEST_RADIUS_OF_HOLE))) * 180 / Math.PI;
        }
        angle = (lowerBoundAngle + upperBoundAngle) / 2;
        m_rotate.setRotateAngle(angle);
        if(Math.abs(m_rotate.getAngle() - angle) < 1){
            atAngle = true;
        } else {
            atAngle = false;
        }
        SmartDashboard.putBoolean("atAngle", atAngle);
    }

    @Override
    public void end(boolean interrupted){
        m_rotate.setVoltage(0);
    }
}