// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {  
  public int moduleNumber;
  private double angleOffset;
  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANcoder angleEncoder;
  
  private StatusSignal<Double> drivePosition;
  private StatusSignal<Double> driveVelocity;
  private StatusSignal<Double> anglePosition;
  private StatusSignal<Double> angleVelocity;
  private BaseStatusSignal[] signals;

  private PositionVoltage angleSetter = new PositionVoltage(0);
  private VelocityTorqueCurrentFOC driveSetter = new VelocityTorqueCurrentFOC(0);

  private SwerveModulePosition internalState = new SwerveModulePosition();

  //Creates the Swerve modules Motors and Encoders
  //Relies on the moduleConstants class found in SwerveModuleConstants.java
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Configuration */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, Constants.DRIVETRAIN_CANBUS);
    configAngleEncoder();

    /* Angle Motor Configuration */
    angleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.DRIVETRAIN_CANBUS);
    configAngleMotor(moduleConstants.cancoderID);

    /* Drive Motor Configuration */
    driveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.DRIVETRAIN_CANBUS);
    configDriveMotor();

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    anglePosition = angleMotor.getPosition();
    angleVelocity = angleMotor.getVelocity();

    signals = new BaseStatusSignal[4];
    signals[0] = drivePosition;
    signals[1] = driveVelocity;
    signals[2] = anglePosition;
    signals[3] = angleVelocity;

  }


  public void setDesiredState(SwerveModuleState desiredState){
    var optimized = SwerveModuleState.optimize(desiredState, internalState.angle);
    double angleToSet = optimized.angle.getRotations();
    angleMotor.setControl(angleSetter.withPosition(angleToSet));
    double velocityToSet = optimized.speedMetersPerSecond * ModuleConstants.rotationsPerMeter;
    driveMotor.setControl(driveSetter.withVelocity(velocityToSet));
  }

  public void stop() {
    driveMotor.stopMotor();
    angleMotor.stopMotor();
  }

  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    var angleEncoderConfigs = new CANcoderConfiguration();
    angleEncoderConfigs.MagnetSensor.MagnetOffset = angleOffset;
    angleEncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    angleEncoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    angleEncoder.getConfigurator().apply(angleEncoderConfigs);
  }

  private void configAngleMotor(int cancoderID) {
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0.kV = 0.0;
    talonfxConfigs.Slot0.kS = 0.0;
    talonfxConfigs.Slot0.kP = ModuleConstants.angleKP;
    talonfxConfigs.Slot0.kI = ModuleConstants.angleKI;
    talonfxConfigs.Slot0.kD = ModuleConstants.angleKD;
    talonfxConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    talonfxConfigs.CurrentLimits.SupplyCurrentLimit = ModuleConstants.angleContinuousCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.angleEnableCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyCurrentThreshold = ModuleConstants.anglePeakCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyTimeThreshold = ModuleConstants.anglePeakCurrentDuration;
    talonfxConfigs.Voltage.PeakForwardVoltage = 10;
    talonfxConfigs.Voltage.PeakReverseVoltage = -10;
    talonfxConfigs.Feedback.FeedbackRemoteSensorID = cancoderID;
    talonfxConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonfxConfigs.Feedback.RotorToSensorRatio = ModuleConstants.angleGearRatio;
    talonfxConfigs.MotorOutput.NeutralMode = ModuleConstants.angleNeutralMode;
    talonfxConfigs.MotorOutput.Inverted = ModuleConstants.angleMotorInvert;
    angleMotor.getConfigurator().apply(talonfxConfigs);
  }

  private void configDriveMotor() {
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0.kV = 0.0;
    talonfxConfigs.Slot0.kS = 0.0;
    talonfxConfigs.Slot0.kP = ModuleConstants.driveKP;
    talonfxConfigs.Slot0.kI = ModuleConstants.driveKI;
    talonfxConfigs.Slot0.kD = ModuleConstants.driveKD;
    talonfxConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    talonfxConfigs.CurrentLimits.SupplyCurrentLimit = ModuleConstants.driveContinuousCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = ModuleConstants.driveEnableCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyCurrentThreshold = ModuleConstants.drivePeakCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyTimeThreshold = ModuleConstants.drivePeakCurrentDuration;
    talonfxConfigs.MotorOutput.NeutralMode = ModuleConstants.driveNeutralMode;
    talonfxConfigs.MotorOutput.Inverted = ModuleConstants.driveMotorInvert;
    driveMotor.getConfigurator().apply(talonfxConfigs);
  }
  
  public SwerveModuleState getState(boolean refresh) {
    if(refresh){
      driveVelocity.refresh();
      anglePosition.refresh();
    }
    return new SwerveModuleState(driveVelocity.getValue(), Rotation2d.fromRotations(anglePosition.getValue()));
  }

  public SwerveModulePosition getPosition(boolean refresh) {
    if(refresh){
      drivePosition.refresh();
      driveVelocity.refresh();
      anglePosition.refresh();
      angleVelocity.refresh();
    }

    double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
    double angleRotations = BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

    double distance = driveRotations / ModuleConstants.rotationsPerMeter;
    internalState.distanceMeters = distance;
    Rotation2d angle = Rotation2d.fromRotations(angleRotations);
    internalState.angle = angle;

    return internalState;
  }

public BaseStatusSignal[] getSignals() {
  return signals;
}

}