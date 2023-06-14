// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {  
  public int moduleNumber;
  private double angleOffset;
  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANcoder angleEncoder;
  private double lastAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.driveKS, ModuleConstants.driveKV, ModuleConstants.driveKA);

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

    lastAngle = getPosition().angle.getDegrees();
    stop();
  }


  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

    if(isOpenLoop){
        double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    else {
        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.ModuleConstants.wheelCircumference, Constants.ModuleConstants.driveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.ModuleConstants.angleGearRatio)); 
    lastAngle = angle;
}

public void setDesiredStateAbs(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    
    if(isOpenLoop){
        double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    else {
        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.ModuleConstants.wheelCircumference, Constants.ModuleConstants.driveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(desiredState.angle.getDegrees(), Constants.ModuleConstants.angleGearRatio)); 
    lastAngle = desiredState.angle.getDegrees();
}

  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    angleMotor.set(ControlMode.PercentOutput, 0);
  }


  private void resetToAbsolute() {
    double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.ModuleConstants.angleGearRatio);
    angleMotor.setSelectedSensorPosition(absolutePosition);
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
  
  //Should not use anymore use getPosition() instead
  public SwerveModuleState getState() {
    double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.ModuleConstants.wheelCircumference, Constants.ModuleConstants.driveGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.ModuleConstants.angleGearRatio));
    return new SwerveModuleState(velocity, angle);
  }


  public SwerveModulePosition getPosition() {
    double distance = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.ModuleConstants.wheelCircumference, Constants.ModuleConstants.driveGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.ModuleConstants.angleGearRatio));
    return new SwerveModulePosition(distance, angle);
  }

}