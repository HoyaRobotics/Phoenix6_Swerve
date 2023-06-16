// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.SwerveModuleConstants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveModules;
  BaseStatusSignal[] signals;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, new SwerveModuleConstants(
      SwerveConstants.FRONT_LEFT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_ENCODER, 
      SwerveConstants.FRONT_LEFT_STEER_OFFSET)),

      new SwerveModule(1, new SwerveModuleConstants(
      SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_ENCODER, 
      SwerveConstants.FRONT_RIGHT_STEER_OFFSET)),

      new SwerveModule(2, new SwerveModuleConstants(
      SwerveConstants.BACK_LEFT_DRIVE_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_ENCODER, 
      SwerveConstants.BACK_LEFT_STEER_OFFSET)),

      new SwerveModule(3, new SwerveModuleConstants(
      SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_ENCODER, 
      SwerveConstants.BACK_RIGHT_STEER_OFFSET))
    };

    signals = new BaseStatusSignal[16];
    for(int i = 0; i<4; i++){
      BaseStatusSignal[] tempSignals = swerveModules[i].getSignals();
      signals[i*4+0] = tempSignals[0];
      signals[i*4+1] = tempSignals[1];
      signals[i*4+2] = tempSignals[2];
      signals[i*4+3] = tempSignals[3];
    }

    if(DriverStation.getAlliance() == Alliance.Blue) {
      GlobalVariables.isBlue = true;
    }else{
      GlobalVariables.isBlue = false;
    }
  }

  //periodic() runs once per command scheduler loop
  @Override
  public void periodic() {}

  public void drive(ChassisSpeeds chassisSpeeds) {
      SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
      setModuleStates(states);
    }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    swerveModules[0].setDesiredState(states[0]);
    swerveModules[1].setDesiredState(states[1]);
    swerveModules[2].setDesiredState(states[2]); 
    swerveModules[3].setDesiredState(states[3]); 
  }
  
  //Calls methods in SwerveModule.java to stop motor movement for each module
  public void stop(){
    swerveModules[0].stop();
    swerveModules[1].stop();
    swerveModules[2].stop();
    swerveModules[3].stop();
  }

  public void lock(){
    swerveModules[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    swerveModules[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    swerveModules[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    swerveModules[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
  }

  public SwerveModulePosition[] getPositions(boolean refresh) {
    return new SwerveModulePosition[] {
      swerveModules[0].getPosition(refresh), 
      swerveModules[1].getPosition(refresh), 
      swerveModules[2].getPosition(refresh), 
      swerveModules[3].getPosition(refresh)
    };
  }

  public BaseStatusSignal[] getSignals() {
    return signals;
  }

  public SwerveModuleState[] getStates(boolean refresh) {
    return new SwerveModuleState[] {
      swerveModules[0].getState(refresh), 
      swerveModules[1].getState(refresh), 
      swerveModules[2].getState(refresh), 
      swerveModules[3].getState(refresh)
    };
  }

  public double getCurrentChassisSpeeds(boolean refresh) {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates(refresh));
    double linearVelocity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVelocity;
  }

  public Rotation2d getCurrentChassisHeading(Pose2d currentPose, boolean refresh) {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates(refresh));
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    Rotation2d currentHeading = robotHeading.plus(currentPose.getRotation());
    return currentHeading;
  }
}
