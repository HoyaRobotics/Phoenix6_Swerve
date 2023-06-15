// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pigeon2Subsystem extends SubsystemBase {

  private final Pigeon2 pigeon2 = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID, "canivore");

  private StatusSignal<Double> yawPosition;
  private StatusSignal<Double> yawVelocity;
  private StatusSignal<Double> pitchPosition;
  private StatusSignal<Double> rollPosition;
  private BaseStatusSignal[] signals;

  /** Creates a new Pigeon2Subsystem. */
  public Pigeon2Subsystem() {
    pigeon2.getConfigurator().apply(new Pigeon2Configuration());
    var pigeon2Configs = new Pigeon2Configuration();
    pigeon2Configs.MountPose.MountPosePitch = 0;
    pigeon2Configs.MountPose.MountPoseRoll = 0;
    pigeon2Configs.MountPose.MountPoseYaw = 0;
    pigeon2.getConfigurator().apply(pigeon2Configs);

    yawPosition = pigeon2.getYaw();
    yawVelocity = pigeon2.getAngularVelocityZ();
    pitchPosition = pigeon2.getPitch();
    rollPosition = pigeon2.getRoll();

    signals = new BaseStatusSignal[2];
    signals[0] = yawPosition;
    signals[1] = yawVelocity;
  }

  @Override
  public void periodic() {}

  /**
   * getGyroRotation - this is the Yaw value (rotate around...)
   * @return Rotation2d
   */
  public Rotation2d getGyroRotation(boolean refresh) {
    if(refresh){
      yawPosition.refresh();
      yawVelocity.refresh();
    }

    double yawRotation = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return Rotation2d.fromDegrees(yawRotation);
  }


  public double getPigeonPitch(){
    pitchPosition.refresh();
    return pitchPosition.getValue();
  }


  public double getPigeonRoll(){
    rollPosition.refresh();
    return rollPosition.getValue();
  }


  public double getPigeonYaw(boolean refresh){
    if(refresh){
      yawPosition.refresh();
      yawVelocity.refresh();
    }

    double yawRotation = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return yawRotation;
  }

  public BaseStatusSignal[] getSignals() {
    return signals;
  }
}
