// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Vision.BackVision;
import frc.robot.subsystems.drive.Vision.FrontVision;
import frc.robot.subsystems.drive.Vision.OuttakeVision;
import frc.robot.subsystems.drive.Vision.RadioVision;
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private FrontVision frontVision;
  private BackVision backVision;
  private OuttakeVision outtakeVision;
  private RadioVision radioVision;

  public Robot() {
    m_robotContainer = new RobotContainer();
    frontVision = new FrontVision(m_robotContainer.drivetrain::addVisionMeasurement);
    backVision = new BackVision(m_robotContainer.drivetrain::addVisionMeasurement);
    outtakeVision = new OuttakeVision(m_robotContainer.drivetrain::addVisionMeasurement);
    radioVision = new RadioVision(m_robotContainer.drivetrain::addVisionMeasurement);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    frontVision.periodic();
    backVision.periodic();
    outtakeVision.periodic();
    radioVision.periodic();

  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_rotation.rotationEncoder.setPosition(0);
       m_robotContainer.m_rotation.rotationMtr.setPosition(0);
    m_robotContainer.m_intakeSlider.sliderEncoder.setPosition(0);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
