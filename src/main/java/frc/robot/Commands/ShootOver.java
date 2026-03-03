package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.Secondary.Rotation;

public class ShootOver extends Command {
    private Indexer m_indexer;
    private Rotation m_rotate;
    private Outtake m_outtake;

    public ShootOver(Outtake m_outtake, Rotation m_rotate, Indexer m_indexer) {
        this.m_indexer = m_indexer;
        this.m_outtake = m_outtake;
        this.m_rotate = m_rotate;
    }

    @Override
    public void initialize(){   
        m_rotate.setRotateAngle(ConstantValues.SHOOT_OVER_ANGLE);
        m_outtake.setVelocity(ConstantValues.SHOOT_OVER_SPEED);
    }

    @Override
    public void execute(){
        if(Math.abs(m_rotate.getAngle() - ConstantValues.SHOOT_OVER_ANGLE) < 0.5 && Math.abs(m_outtake.wheelSpeedMtr.getVelocity().getValueAsDouble() - ConstantValues.SHOOT_OVER_SPEED) <0.5){
            m_indexer.setVoltage(5);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_indexer.setVoltage(0);
        m_rotate.setRotateAngle(0);
        m_outtake.setVelocity(ConstantValues.OUTTAKE_IDLE_SPEED);
    }
}