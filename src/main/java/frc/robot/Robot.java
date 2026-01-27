// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.drive.Vision.backVision;
// import frc.robot.subsystems.drive.Vision.frontVision;
// import frc.robot.subsystems.drive.Vision.leftVision;
// import frc.robot.subsystems.drive.Vision.rightVision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // private frontVision frontVision;
  // private backVision backVision;
  // private leftVision leftVision;
  // private rightVision rightVision;

  public Robot() {
    m_robotContainer = new RobotContainer();
  
    // frontVision = new frontVision(m_robotContainer.drivetrain::addVisionMeasurement);
    // backVision = new backVision(m_robotContainer.drivetrain::addVisionMeasurement);
    // leftVision = new leftVision(m_robotContainer.drivetrain::addVisionMeasurement);
    // rightVision = new rightVision(m_robotContainer.drivetrain::addVisionMeasurement);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // frontVision.periodic();
    // backVision.periodic();
    // leftVision.periodic();
    // rightVision.periodic();

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
      m_autonomousCommand.schedule();
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
