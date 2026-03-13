// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Vision.FrontVision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private FrontVision frontVision;
  // private BackVision backVision;
  // private OuttakeVision outtakeVision;
  // private RadioVision radioVision;

  public Robot() {
    m_robotContainer = new RobotContainer();
    frontVision = new FrontVision(m_robotContainer.drivetrain::addVisionMeasurement);
    // backVision = new BackVision(m_robotContainer.drivetrain::addVisionMeasurement);
    // outtakeVision = new OuttakeVision(m_robotContainer.drivetrain::addVisionMeasurement);
    // radioVision = new RadioVision(m_robotContainer.drivetrain::addVisionMeasurement);

  }

  @Override
  public void robotInit() {
  SignalLogger.enableAutoLogging(false);
    LiveWindow.disableAllTelemetry();
     try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(1);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(1);

    // Silence Rotation2d warnings
    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    frontVision.periodic();
    // backVision.periodic();
    // outtakeVision.periodic();
    // radioVision.periodic();

    if(!m_robotContainer.m_intakeSlider.out && m_robotContainer.m_intake.intakeOn){
      m_robotContainer.m_intake.runIntake();
    }

  }


  // @Override
  // public void disabledInit() {

  // }

  // @Override
  // public void disabledPeriodic() {}

  // @Override
  // public void disabledExit() {}

  @Override
  public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    m_robotContainer.m_rotation.rotationMtr.setPosition(0);
    m_robotContainer.m_intakeSlider.sliderEncoder.setPosition(0);
    if(DriverStation.getAlliance().get() == Alliance.Red){
            m_robotContainer.drivetrain.xDistanceToHub = 11.9; // was 11.9
            m_robotContainer.drivetrain.rotOffset = Math.PI;
            System.out.println("Bello");
        } else {
            m_robotContainer.drivetrain.xDistanceToHub = 4.925; // was 4.625
            m_robotContainer.drivetrain.rotOffset = 0;
            System.out.println("hello");
        }
  }

  @Override
  public void autonomousPeriodic() {}

  // @Override
  // public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.m_rotation.rotationMtr.setPosition(0);
    // m_robotContainer.m_intakeSlider.sliderEncoder.setPosition(0);
    if(DriverStation.getAlliance().get() == Alliance.Red){
            m_robotContainer.drivetrain.xDistanceToHub = 11.9; // was 11.9
            m_robotContainer.drivetrain.rotOffset = Math.PI;
            System.out.println("Bello");
        } else {
            m_robotContainer.drivetrain.xDistanceToHub = 4.925; // was 4.625
            m_robotContainer.drivetrain.rotOffset = 0;
            System.out.println("hello");
        }
  }

  // @Override
  // public void teleopPeriodic() {}

  // @Override
  // public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // @Override
  // public void testPeriodic() {}

  // @Override
  // public void testExit() {}

  // @Override
  // public void simulationPeriodic() {}
}
