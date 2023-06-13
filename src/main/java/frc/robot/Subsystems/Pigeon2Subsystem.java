// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BalanceConstants;

public class Pigeon2Subsystem extends SubsystemBase {

  private final Pigeon2 pigeon2 = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID, "canivore");

  /** Creates a new Pigeon2Subsystem. */
  public Pigeon2Subsystem() {
    pigeon2.configFactoryDefault();
    pigeon2.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
  }

  /**
   * zeroGyroscope
   * Sets the angle to 0 in degrees
   */
  public void zeroGyroscope() {
    pigeon2.setYaw(0);
  }

  public void zeroPitch() {}

  /**
   * getGyroRotation - this is the Yaw value (rotate around...)
   * @return Rotation2d
   */
  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(pigeon2.getYaw());
  }


  public double getPigeonPitch(){
    return pigeon2.getPitch();
  }


  public double getPigeonRoll(){
    return pigeon2.getRoll();
  }


  public double getPigeonYaw(){
    return pigeon2.getYaw();
  }
  

  public int evaluatePitch(){
    if(getPigeonPitch() > BalanceConstants.pitchMaxLimit){
      return 1;
    }
    else if(getPigeonPitch() < BalanceConstants.pitchMinLimit){
      return 2;
    } 
    else {
      return 0;
    }
  }


  public int evaluateRoll(){
    if(getPigeonRoll() > BalanceConstants.rollMaxLimit){
      return 1;
    }
    else if(getPigeonRoll() < BalanceConstants.rollMinLimit){
      return 2;
    } 
    else {
      return 0;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch", getPigeonPitch());
    SmartDashboard.putNumber("Roll", getPigeonRoll());
    SmartDashboard.putNumber("Yaw", getPigeonYaw());
  }
}
