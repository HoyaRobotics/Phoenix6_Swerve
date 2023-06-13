// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.SwerveConstants;

public class PoseEstimator extends SubsystemBase {

  private final SwerveSubsystem swerveSubsystem;
  private final Pigeon2Subsystem pigeon2Subsystem;
  

  PhotonCamera gridCamera = new PhotonCamera("OV5647");
  //PhotonCamera loadingCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  //Transform3d robotToCam = new Transform3d(new Translation3d(-0.408069, 0.194006, 1.257285), new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(180.0)));
  Transform3d robotToGridCam = new Transform3d(new Translation3d(-0.354094, 0.227661, 1.073770), new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(180.0)));
  //Transform3d robotToLoadingCam = new Transform3d(new Translation3d(-0.287274, -0.193675, 1.255828), new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(0.0)));
  PhotonPoseEstimator photonPoseEstimatorGrid;
  //PhotonPoseEstimator photonPoseEstimatorLoading;

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. 
  // Smaller numbers will cause the filter to "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence on the final pose estimate.
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); //was 0.05, 0.05, deg to rad 5
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9); //was 0.02, 0.02, 5
  private static SwerveDrivePoseEstimator poseEstimator;
  
  private final Field2d field2d = new Field2d();

  public PoseEstimator(SwerveSubsystem swerveSubsystem, Pigeon2Subsystem pigeon2Subsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.pigeon2Subsystem = pigeon2Subsystem;

    poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.KINEMATICS, 
      pigeon2Subsystem.getGyroRotation(), 
      swerveSubsystem.getPositions(), 
      new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0)), 
      stateStdDevs, 
      visionMeasurementStdDevs);

    SmartDashboard.putData("Field", field2d);

    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // Create pose estimator
      if(DriverStation.getAlliance() == Alliance.Red){
        fieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      }
      photonPoseEstimatorGrid = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, gridCamera,  robotToGridCam);
      //photonPoseEstimatorLoading = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, loadingCamera, robotToLoadingCam);
      photonPoseEstimatorGrid.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      //photonPoseEstimatorLoading.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (IOException e) {
      // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
      // where the tags are.
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimatorGrid = null;
      //photonPoseEstimatorLoading = null;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!GlobalVariables.hasConnectedToDS) {
      if(DriverStation.isDSAttached() && DriverStation.getAlliance().equals(Alliance.Blue)) {
        GlobalVariables.isBlue = true;
        System.out.println("Blue Side");
        try {
          AprilTagFieldLayout field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
          field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
          photonPoseEstimatorGrid.setFieldTags(field);
          //photonPoseEstimatorLoading.setFieldTags(field);
          System.out.println("Blue Side Set");
        } catch (IOException e) {
          DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
          photonPoseEstimatorGrid = null;
          //photonPoseEstimatorLoading = null;
          System.out.println("Error");
        }
        GlobalVariables.hasConnectedToDS = true;
      }else if(DriverStation.isDSAttached() && DriverStation.getAlliance().equals(Alliance.Red)) {
        GlobalVariables.isBlue = false;
        System.out.println("Red Side");
        try {
          AprilTagFieldLayout field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
          field.setOrigin(OriginPosition.kRedAllianceWallRightSide);
          photonPoseEstimatorGrid.setFieldTags(field);
          //photonPoseEstimatorLoading.setFieldTags(field);
          System.out.println("Red Side Set");
        } catch (IOException e) {
          DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
          photonPoseEstimatorGrid = null;
          //photonPoseEstimatorLoading = null;
          System.out.println("Error");
        }
        GlobalVariables.hasConnectedToDS = true;
      }
    }
    

    Optional<EstimatedRobotPose> gridPose = getCameraLatestResults(gridCamera, photonPoseEstimatorGrid, getCurrentPose());
    if(gridPose.isPresent()){
      Boolean isGridPoseCorrect = isPoseWithinView(gridPose.get().estimatedPose.toPose2d(), 1.69, 0.32, 16.13, 7.7);
      if(isGridPoseCorrect){
        var confidence = confidenceCalculator(gridPose.get());
        poseEstimator.setVisionMeasurementStdDevs(confidence);
        SmartDashboard.putString("Estimated Grid Pose", gridPose.get().estimatedPose.toString());
        poseEstimator.addVisionMeasurement(gridPose.get().estimatedPose.toPose2d(), gridPose.get().timestampSeconds);
        //Logger.getInstance().recordOutput("GridPose", gridPose.get().estimatedPose.toPose2d());
        //Logger.getInstance().recordOutput("GridConfidence", confidence.toString());
        //Logger.getInstance().recordOutput("GridConfidence", confidence.getData());
      }
    }

    /*Optional<EstimatedRobotPose> loadingPose = getCameraLatestResults(loadingCamera, photonPoseEstimatorLoading, getCurrentPose());
    if(loadingPose.isPresent()){
      Boolean isLoadingPoseCorrect = isPoseWithinView(loadingPose.get().estimatedPose.toPose2d(), 1.69, 0.32, 16.13, 7.7);
      if(isLoadingPoseCorrect){
        var confidence = confidenceCalculator(loadingPose.get());
        poseEstimator.setVisionMeasurementStdDevs(confidence);
        SmartDashboard.putString("Estimated Loading Pose", loadingPose.get().estimatedPose.toString());
        poseEstimator.addVisionMeasurement(loadingPose.get().estimatedPose.toPose2d(), loadingPose.get().timestampSeconds);
        Logger.getInstance().recordOutput("LoadingPose", loadingPose.get().estimatedPose.toPose2d());
        Logger.getInstance().recordOutput("loadingConfidence", confidence.toString());
      }
    }*/

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), pigeon2Subsystem.getGyroRotation(), swerveSubsystem.getPositions());
    field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    // This method will be called once per scheduler run
  }

  public static Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getPoseX() {
    return poseEstimator.getEstimatedPosition().getX();
  }

  public double getPoseY() {
    return poseEstimator.getEstimatedPosition().getY();
  }

  public double getPoseTheta() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public Rotation2d getPoseRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(pigeon2Subsystem.getGyroRotation(), swerveSubsystem.getPositions(), pose);
  }

  public void setTrajectoryField2d(Trajectory trajectory) {
    field2d.getObject("traj").setTrajectory(trajectory);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimatorGrid == null) {
        // The field layout failed to load, so we cannot estimate poses.
        return Optional.empty();
    }
    photonPoseEstimatorGrid.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimatorGrid.update();
  }

  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance) {
        smallestDistance = distance;
      }
    }
    double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
        ? 1
        : Math.max(
          1,
          (estimation.targetsUsed.get(0).getPoseAmbiguity()
              + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
              * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
    double confidenceMultiplier = Math.max(
        1,
        (Math.max(
            1,
            Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
                * Constants.VisionConstants.DISTANCE_WEIGHT)
            * poseAmbiguityFactor)
            / (1
                + ((estimation.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));
    
    return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
  }

  private Optional<EstimatedRobotPose> getCameraLatestResults(PhotonCamera suppliedCamera, PhotonPoseEstimator photonPoseEstimator, Pose2d prevEstimatedRobotPose) {
    if(photonPoseEstimator != null && suppliedCamera != null) {
      var photonResults = suppliedCamera.getLatestResult();
      if(photonResults.hasTargets() && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)) {
        return photonPoseEstimator.update(photonResults);
      }else{
        return Optional.empty();
      }
    }else{
      return Optional.empty();
    }
  }

  private boolean isPoseWithinView(Pose2d pose, double minX, double minY, double maxX, double maxY) {
    if(pose.getX() > minX && pose.getY() > minY && pose.getX() < maxX && pose.getY() < maxY){
      return true;
    }else{
      return false;
    }
  }
}