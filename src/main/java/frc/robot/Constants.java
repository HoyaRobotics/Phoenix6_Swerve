// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int kDriverControllerPort = 0;
  public static final int kOperatorControllerPort = 1;

  public static final int DRIVETRAIN_PIGEON_ID = 2; // Pigeon ID
  public static final int CANDLE_ID = 3;
  public static final String DRIVETRAIN_CANBUS = "canivore";

  public static final double DRIVE_SPEED = 0.8;
  public static final double BOOST_SPEED = 1.0;
  public static final double PERCISION_SPEED = 0.25;

  public static final class ModuleConstants {

    // Swerve Current Limiting
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    // Angle Motor PID Values
    public static final double angleKP = 0.0; 
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0; 

    // Drive Motor PID Values
    public static final double driveKP = 0.0;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    // Drive Motor Characterization Values
    public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
    public static final double driveKV = (2.44 / 12);
    public static final double driveKA = (0.27 / 12);

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    // Angle Encoder Invert
    public static final boolean canCoderInvert = false;

    // Motor Inverts
    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = false;

    // Neutral Modes
    public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
    public static final double angleNeutralDeadband = 0.1;

    public static final double wheelDiameter = Units.inchesToMeters(3.94);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double driveGearRatio = (((50.0/14.0)*(17.0/27.0)*(45.0/15.0)) / 1.0); //6.75:1
    public static final double angleGearRatio = (((32.0/15.0)*(60.0/10.0)) / 1.0); //12.8:1
  }

  public static final class SwerveConstants {
    public static final double TRACKWIDTH_METERS = Units.inchesToMeters(17.5);
    public static final double WHEELBASE_METERS = Units.inchesToMeters(26.5);

    public static final double MAX_VOLTAGE = 12.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)) * 0.10033 * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0), // Front Left
        new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0), // Front Right
        new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0), // Back Left
        new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)); // Back Right

    public static final int FRONT_LEFT_DRIVE_MOTOR = 19; // Front left module drive motor ID
    public static final int FRONT_LEFT_STEER_MOTOR = 20; // Front left module steer motor ID
    public static final int FRONT_LEFT_STEER_ENCODER = 21; // Front left steer encoder ID
    public static final double FRONT_LEFT_STEER_OFFSET = 38.95+90; // Front left steer offset

    public static final int FRONT_RIGHT_DRIVE_MOTOR = 14; // Front right drive motor ID
    public static final int FRONT_RIGHT_STEER_MOTOR = 13; // Front right steer motor ID
    public static final int FRONT_RIGHT_STEER_ENCODER = 15; // Front right steer encoder ID
    public static final double FRONT_RIGHT_STEER_OFFSET = -21.88+90; // Front right steer offset

    public static final int BACK_LEFT_DRIVE_MOTOR = 16; // Back left drive motor ID
    public static final int BACK_LEFT_STEER_MOTOR = 17; // Back left steer motor ID
    public static final int BACK_LEFT_STEER_ENCODER = 18; // Back left steer encoder ID 
    public static final double BACK_LEFT_STEER_OFFSET = -3.69+270; // Back left steer offset

    public static final int BACK_RIGHT_DRIVE_MOTOR = 10; // Back right drive motor ID
    public static final int BACK_RIGHT_STEER_MOTOR = 11; // Back right steer motor ID
    public static final int BACK_RIGHT_STEER_ENCODER = 12; // Back right steer encoder ID
    public static final double BACK_RIGHT_STEER_OFFSET = 94.66-90; // Back right steer offset
  }

  public static final class AutoConstants {
    public static final double kPXController = 3.0; //1.5
    public static final double kPYController = 3.0; //1.5
    public static final double kPThetaController = 3.0;
  }

  public static final class BalanceConstants {
    public static final double pitchMaxLimit = 0.5;
    public static final double pitchMinLimit = -0.5;
    public static final double rollMaxLimit = 0.5;
    public static final double rollMinLimit = -0.5;
  }

  public static final class IntakeConstants {
    public static final int FRONT_INTAKE_ROLLER = 22;
    public static final int BACK_INTAKE_ROLLER = 23;
    public static final int INTAKE_RETRACTION = 24;
    public static final double rotationRatio = (32.0/14.0) * (56.0/16.0) * (50/8);
  }

  public static final class ConveyorConstants {
    public static final int BOTTOM_CONVEYOR_CONVEYOR = 25;
    public static final int LEFT_CONVEYOR_CONVEYOR = 26;
    public static final int RIGHT_CONVEYOR_CONVEYOR = 27;
  }

  public static final class ArmConstants {
    public static final int LEFT_ARM_MOTOR = 28;
    public static final int RIGHT_ARM_MOTOR = 29;
    public static final int EXTENSION_MOTOR = 30;
  }

  public static final class PincherConstants {
    public static final int PNUMATICS_MODULE_ID = 4;
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
    public static final int GRIPPER_CLOSE = 0;
    public static final int GRIPPER_OPEN = 1;
  }

  public static final List<Double> ARM_POSITIONS = Collections.unmodifiableList(List.of(
    -7.0,
    -20.0,
    -25.5 //Original -27.0
  ));

  public static final List<Double> EXTENSION_POSITIONS = Collections.unmodifiableList(List.of(
    50.0,
    66.0,
    3.0
  ));

  public static final List<Double> RELEASE_ARM_POSITIONS = Collections.unmodifiableList(List.of(
    -7.0,
    -16.0,
    -20.0
  ));

  public static final List<Double> RELEASE_EXTENSION_POSITIONS = Collections.unmodifiableList(List.of(
    50.0,
    50.0,
    3.0
  ));

  public static final List<Pose2d> NODE_POSE_BLUE = Collections.unmodifiableList(List.of(
    new Pose2d(new Translation2d(1.89, 0.5), Rotation2d.fromDegrees(0)),//1.89
    new Pose2d(new Translation2d(1.89, 1.07), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 1.62), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 2.19), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 2.75), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 3.31), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 3.86), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 4.43), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 4.98), Rotation2d.fromDegrees(0))
  ));

  public static final List<Pose2d> NODE_POSE_RED = Collections.unmodifiableList(List.of(
    new Pose2d(new Translation2d(1.89, 7.52), Rotation2d.fromDegrees(0)),//1.89
    new Pose2d(new Translation2d(1.89, 6.95), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 6.39), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 5.83), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 5.27), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 4.72), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 4.16), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 3.59), Rotation2d.fromDegrees(0)),
    new Pose2d(new Translation2d(1.89, 3.04), Rotation2d.fromDegrees(0))
  ));

  public static final List<GAME_OBJECT> GAME_OBJECT_STRING = Collections.unmodifiableList(List.of(
    GAME_OBJECT.Cone,
    GAME_OBJECT.Cube,
    GAME_OBJECT.Cone,
    GAME_OBJECT.Cone,
    GAME_OBJECT.Cube,
    GAME_OBJECT.Cone,
    GAME_OBJECT.Cone,
    GAME_OBJECT.Cube,
    GAME_OBJECT.Cone
  ));

  public static enum GAME_OBJECT {Cone, Cube};

  public static final double pickupConeExtensionPosition = 25.5;
  public static final double pickupCubeExtensionPosition = 40;
  public static final double pickupConeArmPosition = 0.5; //was 0.5 at north bay, 0.75 works ok but sometimes get stuck on conveyor
  public static final double pickupCubeArmPosition = 0.1;
  public static final double holdExtensionPosition = 80;
  public static final double holdArmPosition = -2;
  public static final double placeExtensionPosition  = 100;
  public static final double placeArmPGain = 0.05;
  public static final double returnArmPGain = 0.07;

  public static final List<AprilTag> TAG_POSES = Collections.unmodifiableList(List.of(
    new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)))),
    new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)))),
    new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)))),
    new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)))),
    new AprilTag(5, new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)))),
    new AprilTag(6, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)))),
    new AprilTag(7, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)))),
    new AprilTag(8, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))))
  ));

  public static final class VisionConstants{

    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.15; //was 0.2
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
      .fill(
        // if these numbers are less than one, multiplying will do bad things
        0.1, // x
        0.1, // y
        0.1);
    
  }
}