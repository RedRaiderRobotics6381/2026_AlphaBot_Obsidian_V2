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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.Commands.AutoShooter;
import frc.robot.Commands.IndexerControl;
import frc.robot.Commands.IntakeRun;
import frc.robot.Commands.OuttakeCmd;
import frc.robot.Commands.OuttakeRun;
import frc.robot.Constants.ConstantValues;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Secondary.Indexer;
//import frc.robot.subsystems.Secondary.Climber;
//import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Intake;
import frc.robot.subsystems.Secondary.IntakeSlider;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.Secondary.Rotation;
//import frc.robot.subsystems.Secondary.Outtake;
//import frc.robot.subsystems.Secondary.RotateSubsystem;
//import frc.robot.subsystems.Secondary.Turret;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.autos.AutoPicker;

public class RobotContainer {

    //private final RotateSubsystem m_RotateSubsystem = new RotateSubsystem();    

    public final Intake m_intake = new Intake();
    public final Indexer m_indexer = new Indexer();
    public final Outtake m_outtake = new Outtake();
    public final IntakeSlider m_intakeSlider = new IntakeSlider();
    
    public final Rotation m_rotation = new Rotation();
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandGenericHID engineer = new CommandGenericHID(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
                   
    private final AutoPicker autoChooser2;



    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    //public DriveToYaw driveToYaw = new DriveToYaw(drivetrain);

    // private final Turret m_turret = new Turret();
    // private final Climber m_climber = new Climber();
    // private final AutoShooter m_autoShooter = new AutoShooter(m_outtake, m_RotateSubsystem, m_indexer, drivetrain, m_turret);
    private final OuttakeRun m_outtakeRun = new OuttakeRun(m_outtake, m_intake, m_intakeSlider);
    private final IntakeRun m_intakeRun = new IntakeRun(m_intake, m_intakeSlider);
    private final IndexerControl m_indexerControl = new IndexerControl(m_outtake, m_indexer);
    private final OuttakeCmd m_outtakeCmd = new OuttakeCmd(m_outtake, m_indexer);

    public RobotContainer() {
        NamedCommands.registerCommand("OuttakeCmd", m_outtakeCmd);
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
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.75) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.75) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.75) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 1.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 1.5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 1.5) // Drive counterclockwise with negative X (left)
        ));
        joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
        ));
        //joystick.rightBumper().onTrue(Commands.runOnce(() -> m_intake.runIntake()));
        // joystick.x().whileTrue(m_indexer.runIndexer());
        joystick.x().whileTrue(m_indexerControl);
        joystick.a().onTrue(m_outtakeRun);
        joystick.b().onTrue(Commands.runOnce(() -> m_intake.runReverseIntake()));
        // joystick.leftTrigger().whileTrue(m_rotation.runRotation());
        // joystick.rightTrigger().whileTrue(m_rotation.runRotationReverse());
        joystick.leftBumper().onTrue(m_intakeRun);

        // engineer.button(2).onTrue(Commands.runOnce(() -> m_intakeSlider.setRotateAngle(8)));
        // engineer.button(3).whileTrue(m_outtakeRun);
        // engineer.button(8).whileTrue(m_intakeRun);
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

  //      engineer.().whileTrue(driveToYaw);
      //  engineer.button(1).whileTrue(m_intake.reverseIntake());
       // engineer.button(2).onTrue(Commands.runOnce(() -> m_intake.setVoltage(3)));
       // engineer.button(5).onTrue(m_autoShooter);
       // engineer.button(6).onTrue(m_climber.ClimberHeightCmd(0));//TODO change value
       // engineer.button(10).onTrue(m_climber.ClimberHeightCmd(0)); // TODO change value

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.y().whileTrue(Commands.runOnce(() -> {
        //     Pose2d currentPose = drivetrain.getState().Pose;
      
        //     Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        //     Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

        //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
        //         PathPlannerPath path = new PathPlannerPath(
        //         waypoints, 
        //         new PathConstraints(
        //             4.0, 4.0, 
        //             Units.degreesToRadians(360), Units.degreesToRadians(540)
        //     ),
        //     null, // Ideal starting state can be null for on-the-fly paths
        //     new GoalEndState(0.0, currentPose.getRotation())
        //     );

        //     // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        //     path.preventFlipping = true;

        //     AutoBuilder.followPath(path).schedule();
        // }));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}