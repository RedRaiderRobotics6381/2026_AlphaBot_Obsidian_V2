package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants {
  public static class TurretConstants {
    public static final int TURRET_MOTOR_PORT = 0; //TODO Change Later!
    public static final int TURRET_ACCELERATION_CONSTRAINT = 0; //TODO Change Later!
    public static final int TURRET_VELOCITY_CONSTRAINT = 0; //TODO Change Later!
  }

  public static class IndexerConstants {
    public static final int INDEXER_MOTOR_PORT_LDR = 0; //TODO Change Later!
    public static final int INDEXER_MOTOR_PORT_FLW = 0; //TODO Change Later!
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_PORT = 0; //TODO Change Later!
  }

  public static class ClimberConstants {
    public static final int LEFT_CLIMBER_MOTOR_PORT = 0; //TODO Change Later!
    public static final int RIGHT_CLIMBER_MOTOR_PORT = 0; //TODO Change Later!
    public static final int CLIMBER_ACCELERATION_CONSTRAINT = 0; //TODO Change Later!
    public static final int CLIMBER_VELOCITY_CONSTRAINT = 0; //TODO Change Later!
  }

  public static class RotateConstants {
    public static final int ROTATE_MOTOR_PORT = 0; //TODO Change Later!
    public static final int ROTATE_ACCELERATION_CONSTRAINT = 0; //TODO Change Later!
    public static final int ROTATE_VELOCITY_CONSTRAINT = 0; //TODO Change Later!
  }

  public static class OuttakeConstants {
    public static final int OUTTAKE_MOTOR_PORT = 0; //TODO Change Later!
    public static final int OUTTAKE_ACCELERATION_CONSTRAINT = 0; //TODO Change Later!
  }
    public static class Vision {
        public static final String kFrontCameraName = "camFront";
        public static final String kBackCameraName = "camBack";
        public static final String kLeftCameraName = "camLeft";
        public static final String kRightCameraName = "camRight";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kFrontRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
        public static final Transform3d kBackRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
        public static final Transform3d kLeftRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
        public static final Transform3d kRightRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class AutonConstants
  {
    public static final double LINEAR_VELOCITY = 2.0; //MPS
    public static final double LINEAR_ACELERATION = 2.0; //Meters per second squared
    public static final double ANGULAR_VELOCITY = 720; //Degrees
    public static final double ANGULAR_ACCELERATION = 360; //Degrees per second squared
  
  }


  public static class PhysicalConstants {
    public static double SHOOTER_HEIGHT = 0; //TODO Change Later!
    
  }
  public static class ConstantValues {
    public static double SHOOTER_RPM = 700; //TODO Change Later
    public static double SHOOTER_SPEED = 0; //TODO Change Later
    
  }
  public static class FieldConstants {
    public static double SMALLEST_RADIUS_OF_HUB = 25.5; //inches
    public static double HEIGHT_OF_HUB = 72; //inches
    public static double SMALLEST_RADIUS_OF_HOLE = 11.7; //inches
    public static double HEIGHT_OF_HOLE = 56; //inches
    
  }
  public static class AprilTagConstants {
    public static int HumanPlayerLeft = 4; // 1 red, 13 blue
    public static int HumanPlayerRight = 0; // 2 red, 12 blue
    public static int Processor = 0; // 3 red, 11 blue
    public static int BargeFront = 0; // 5, 14 both blue and red are on the same side of the field
    public static int BargeBack = 0; // 15, 4 both blue and red are on the same side of the field
    public static int Reef0 = 0; // 7 red, 18 Blue
    public static int Reef60 = 0; // 8 red, 17 Blue
    public static int Reef120 = 0; // 9 red, 22 Blue
    public static int Reef180 = 0; // 10 red, 21 Blue
    public static int Reef240 = 0; // 11 red, 20 Blue
    public static int Reef300 = 0; // 6 red, 19
  }
}
