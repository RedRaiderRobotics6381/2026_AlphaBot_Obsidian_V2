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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Vision.BackVision;
import frc.robot.subsystems.drive.Vision.FrontVision;
// import frc.robot.subsystems.drive.Vision.OuttakeVision;
import frc.robot.subsystems.drive.Vision.RadioVision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private FrontVision frontVision;
  private BackVision backVision;
  // private OuttakeVision outtakeVision;
  private RadioVision radioVision;
  private String allianceColor;

  public Robot() {
    m_robotContainer = new RobotContainer();
    frontVision = new FrontVision(m_robotContainer.drivetrain::addVisionMeasurement);
    backVision = new BackVision(m_robotContainer.drivetrain::addVisionMeasurement);
    // outtakeVision = new OuttakeVision(m_robotContainer.drivetrain::addVisionMeasurement);
    radioVision = new RadioVision(m_robotContainer.drivetrain::addVisionMeasurement);

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
    backVision.periodic();
    // outtakeVision.periodic();
    radioVision.periodic();

    SmartDashboard.putBoolean("Match Data/InShift", currentShiftIsYours());
    SmartDashboard.putNumber(
                    "Match Data/TimeLeftInShift",
                    timeLeftInShiftSeconds(DriverStation.getMatchTime()));

    // if(!m_robotContainer.m_intakeSlider.out && m_robotContainer.m_intake.intakeOn){
    //   m_robotContainer.m_intake.runIntake();
    // }

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
    m_robotContainer.m_intakeSlider.sliderLdrMtr.setPosition(0);
    if(DriverStation.getAlliance().get() == Alliance.Red){
            m_robotContainer.drivetrain.xDistanceToHub = 11.92; // was 11.9
            m_robotContainer.drivetrain.rotOffset = Math.PI;
            System.out.println("Bello");
        } else {
            m_robotContainer.drivetrain.xDistanceToHub = 4.625; // was 4.625
            m_robotContainer.drivetrain.rotOffset = 0;
            System.out.println("hello");
        }
  }

  @Override
  public void autonomousPeriodic() {
          SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void autonomousExit() {
        m_robotContainer.m_intakeSlider.setVoltage(0);
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_rotation.rotationMtr.setPosition(0);
    m_robotContainer.m_intakeSlider.sliderLdrMtr.setPosition(0); //TODO uncomment this while testing!!!!!
    if(DriverStation.getAlliance().get() == Alliance.Red){
            m_robotContainer.drivetrain.xDistanceToHub = 11.92; // was 11.9
            m_robotContainer.drivetrain.rotOffset = Math.PI;
            System.out.println("Bello");
        } else {
            m_robotContainer.drivetrain.xDistanceToHub = 4.625; // was 4.625
            m_robotContainer.drivetrain.rotOffset = 0;
            System.out.println("hello");
        }

        
  //       DriverStation.getGameSpecificMessage();
  //       if(DriverStation.getAlliance().get() == Alliance.Red){
  //         allianceColor = "R";
  //       } else {
  //         allianceColor = "B";
  //       }
  //       if(DriverStation.getGameSpecificMessage() == allianceColor){
  //         DriverStation.getMatchTime();
  //         if (DriverStation.getMatchTime() == 55.0 || DriverStation.getMatchTime() == 105.0) {
  //           // our alliance
  //         }
  //         if (DriverStation.getMatchTime() == 80.0 || DriverStation.getMatchTime() == 130.0) {
  //           // other alliance
  //         }

  //       }
  //       if(DriverStation.getGameSpecificMessage() != allianceColor){
  //         DriverStation.getMatchTime();
  //         if (DriverStation.getMatchTime() > 55.0 || DriverStation.getMatchTime() > 105.0) {
  //           // other alliance
  //         }
  //         if (DriverStation.getMatchTime() > 80.0 || DriverStation.getMatchTime() > 130.0) {
  //           // our alliance
  //         }

  //       }

  

   }

  @Override
  public void teleopPeriodic() {
      SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void teleopExit() {
    m_robotContainer.m_intakeSlider.setVoltage(0);
  }

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


public static boolean blueWonAuto() {
        String matchInfo = DriverStation.getGameSpecificMessage();
        if (matchInfo != null && matchInfo.length() > 0) {
            return matchInfo.charAt(0) == 'B';
        }
        // Safe default if data isn't ready yet
        return false;
    }

    public static int timeLeftInShiftSeconds(double currentMatchTime) {
        if (currentMatchTime >= 130) {
            return (int) (currentMatchTime - 130);
        } else if (currentMatchTime >= 105 && currentMatchTime < 130) {
            return (int) (currentMatchTime - 105);
        } else if (currentMatchTime >= 80 && currentMatchTime < 105) {
            return (int) (currentMatchTime - 80);
        } else if (currentMatchTime >= 55 && currentMatchTime < 80) {
            return (int) (currentMatchTime - 55);
        } else if (currentMatchTime >= 30 && currentMatchTime < 55) {
            return (int) (currentMatchTime - 30);
        } else {
            return (int) currentMatchTime;
        }
    }

    public static boolean isCurrentShiftBlue(double currentMatchTime) {
        if (currentMatchTime >= 105 && currentMatchTime < 130) {
            return blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 80 && currentMatchTime < 105) {
            return blueWonAuto() ? true : false;
        } else if (currentMatchTime >= 55 && currentMatchTime < 80) {
            return blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 30 && currentMatchTime < 55) {
            return blueWonAuto() ? true : false;
        } else {
            return true;
        }
    }

    public static boolean isCurrentShiftRed(double currentMatchTime) {
        if (currentMatchTime >= 105 && currentMatchTime < 130) {
            return blueWonAuto() ? true : false;
        } else if (currentMatchTime >= 80 && currentMatchTime < 105) {
            return blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 55 && currentMatchTime < 80) {
            return blueWonAuto() ? true : false;
        } else if (currentMatchTime >= 30 && currentMatchTime < 55) {
            return blueWonAuto() ? false : true;
        } else {
            return true;
        }
    }

    public static boolean currentShiftIsYours() {
        double currentMatchTime = DriverStation.getMatchTime();
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue)) {
            return isCurrentShiftBlue(currentMatchTime);
        } else {
            return isCurrentShiftRed(currentMatchTime);
        }
      }
    }
