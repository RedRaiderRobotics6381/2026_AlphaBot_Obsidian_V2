// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Commands.*;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Secondary.Climber;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Intake;
import frc.robot.subsystems.Secondary.IntakeSlider;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.Secondary.Rotation;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.autos.AutoPicker;

public class RobotContainer {

    public final Intake m_intake = new Intake();
    public final Indexer m_indexer = new Indexer();
    public final Outtake m_outtake = new Outtake();
    public final IntakeSlider m_intakeSlider = new IntakeSlider();
    // public final Climber m_climber = new Climber();
    
    public final Rotation m_rotation = new Rotation();
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle driveToAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController joystick_HID = joystick.getHID();
    private final CommandGenericHID engineer = new CommandGenericHID(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
                   
    private final AutoPicker autoChooser2;



    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    // private final Turret m_turret = new Turret();
    // private final Climber m_climber = new Climber();

    public RobotContainer() {
        NamedCommands.registerCommand("Outtake", new OuttakeRun(m_outtake, m_intake, m_intakeSlider));
        NamedCommands.registerCommand("Aim", new AutoAimer(m_rotation));
        NamedCommands.registerCommand("Index", new IndexerControl(m_indexer));
        NamedCommands.registerCommand("Yaw", new DriveToYaw(drivetrain, driveToAngle, joystick, MaxSpeed));
        NamedCommands.registerCommand("Intake Out", Commands.runOnce(() -> m_intakeSlider.setRotateAngle()));
        NamedCommands.registerCommand("Intake Run", Commands.runOnce(() -> m_intake.runIntake()));
        NamedCommands.registerCommand("AutoShooter", 
            Commands.parallel(
                    new OuttakeRun(m_outtake, m_intake, m_intakeSlider),
                    new IndexerControl(m_indexer),
                    new AutoAimer(m_rotation),
                    new DriveToYaw(drivetrain, driveToAngle, joystick, MaxSpeed)
            ));
        autoChooser2 = new AutoPicker();
        autoChooser = AutoBuilder.buildAutoChooser();
        for(int i = 0; i < autoChooser2.getAutoList().length; i++){
            autoChooser.addOption(autoChooser2.getAutoList()[i].getName(), autoChooser2.getAutoList()[i].getPath());
        }

        configureBindings();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Warmup PathPlanner to avoid Java pauses
        Commands.sequence(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick_HID.getLeftY() * MaxSpeed * 0.75) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick_HID.getLeftX() * MaxSpeed * 0.75) // Drive left with negative X (left)
                    .withRotationalRate(-joystick_HID.getRightX() * MaxAngularRate * 0.75) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick_HID.getLeftY() * MaxSpeed * 1.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick_HID.getLeftX() * MaxSpeed * 1.5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick_HID.getRightX() * MaxAngularRate * 1.5) // Drive counterclockwise with negative X (left)
        ));

        joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick_HID.getLeftY() * MaxSpeed * 0.15) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick_HID.getLeftX() * MaxSpeed * 0.15) // Drive left with negative X (left)
                    .withRotationalRate(-joystick_HID.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
        ));

        joystick.y().whileTrue(new ShootOver(m_outtake, m_rotation, m_indexer));
        //Auto Shooter
        joystick.rightBumper().whileTrue(
            Commands.parallel(
                    new OuttakeRun(m_outtake, m_intake, m_intakeSlider),
                    new IndexerControl(m_indexer),
                    new AutoAimer(m_rotation),
                    new DriveToYaw(drivetrain, driveToAngle, joystick, MaxSpeed)
            )
        );
        //Auto Shooter no yaw
        // joystick.rightBumper()
        //     .whileTrue(
        //         Commands.parallel(
        //             Commands.runOnce(() -> IndexerControl.override = true),
        //             new OuttakeRun(m_outtake, m_intake, m_intakeSlider),
        //             new IndexerControl(m_indexer),
        //             new AutoAimer(m_rotation)
        // ));
        //Auto Shooter no yaw no pitch
        // joystick.povLeft().whileTrue(
        //     Commands.parallel(
        //             new OuttakeRun(m_outtake, m_intake, m_intakeSlider),
        //             new StupidIndexer(m_indexer)
        //     )
        // );
        joystick.leftBumper().onTrue(m_intake.runOnce(() -> m_intake.runIntake()));
        joystick.x().onTrue(m_intake.runOnce(() -> m_intake.runReverseIntake()));
        // joystick.povUp().whileTrue(m_climber.runClimberUp());
        // joystick.povDown().whileTrue(m_climber.runClimberDown());
        joystick.a().onTrue(Commands.runOnce(() -> m_intakeSlider.setRotateAngle()).andThen(new WaitCommand(2)).andThen(Commands.runOnce(() -> m_intakeSlider.setVoltage(0))));

        // engineer.button(1).whileTrue(m_autoShooter);
        // engineer.button(2).onTrue(m_intakeRun);

        // engineer.button(4).whileTrue(m_shootOver);
        // engineer.button(5).onTrue(Commands.runOnce(() -> m_intake.runReverseIntake()));
        // engineer.button(3).whileTrue(m_climber.runClimberUp());
        // engineer.button(6).whileTrue(m_climber.runClimberDown());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // reset the field-centric heading on start press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}