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

  public static class IndexerConstants {
    public static final int INDEXER_MOTOR_PORT = 25;
    public static final int UPTAKE_WHEELS_PORT = 27;
    public static final int UPTAKE_BELT_PORT = 30;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_PORT_1 = 36;
    public static final int INTAKE_MOTOR_PORT_2 = 37;
  }

  public static class IntakeSliderConstants {
    public static final int INTAKE_SLIDER_MOTOR_PORT_LDR = 35;
    public static final int INTAKE_SLIDER_MOTOR_PORT_FLW = 26;
    public static final int SLIDER_ACCELERATION_CONSTRAINT = 9999; 
    public static final int SLIDER_VELOCITY_CONSTRAINT = 9999; 
  }

  public static class RotationConstants {
    public static final int ROTATION_MOTOR_PORT = 28;
    public static final int ROTATION_ACCELERATION_CONSTRAINT = 9999; 
    public static final int ROTATION_VELOCITY_CONSTRAINT = 9999; 
    public static final double ROTATION_INITIAL_ANGLE = 89;
  }

  public static class OuttakeConstants {
    public static final int OUTTAKE_MOTOR_PORT = 29;
    public static final int OUTTAKE_ACCELERATION_CONSTRAINT = 9999;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR_PORT = 38; //TODO Change!!!
    public static final int CLIMBER_VELOCITY_CONSTRAINT = 9999;
    public static final int CLIMBER_ACCELERATION_CONSTRAINT = 9999;
  }

    public static class Vision {
        public static final String kFrontCameraName = "Front";
        public static final String kBackCameraName = "Back";
        public static final String kOuttakeCameraName = "OuttakeSide";
        public static final String kRadioCameraName = "RadioSide";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kFrontRobotToCam =
                new Transform3d(new Translation3d(0.34, 0.267, 0.512), new Rotation3d(0, Math.PI/6, 0));
        public static final Transform3d kBackRobotToCam =
                new Transform3d(new Translation3d(-0.34, -0.111, 0.415), new Rotation3d(0, Math.PI/6, Math.PI));
        public static final Transform3d kOuttakeRobotToCam =
                new Transform3d(new Translation3d(0.175, 0.34, 0.52), new Rotation3d(0, Math.PI/6, 3 * Math.PI/2));
        public static final Transform3d kRadioRobotToCam =
                new Transform3d(new Translation3d(0.175, -0.34, 0.52), new Rotation3d(0, Math.PI/6, Math.PI / 2));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

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
    public static double SHOOTER_HEIGHT = 24;
  }

  public static class ConstantValues {
    public static double DISTANCE_TO_SHOOT = 85;
    public static double SHOOTER_RPS_NEAR = 25;
    public static double SHOOTER_SPEED_NEAR = 298;
    public static double SHOOTER_RPS_FAR = 30;
    public static double SHOOTER_SPEED_FAR = 320;
    public static double SHOOT_OVER_ANGLE = 80;
    public static double SHOOT_OVER_SPEED = 30;
    public static double OUTTAKE_IDLE_SPEED = 10;
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
