// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.Constants.CTREConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule implements Sendable {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  // private final CANCoder m_driveEncoder;
  private final CANCoder m_turningEncoder;
  private double m_turningEncoderOffset = 0;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.4, 2.2); // 0.4, 1.93
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.34, 0.32); // 0.1,0.25

  private SwerveModuleState m_desiredState = new SwerveModuleState();

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorCanId,
      int turningMotorCanId,
      int turningEncoderCanId,
      double turningEncoderOffset) {
    // m_driveMotor = new WPI_TalonFX(driveMotorCanId);
    // m_turningMotor = new WPI_TalonFX(turningMotorCanId);

    // this.m_turningEncoder = new CANCoder(turningEncoderCanId);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.ssssssssssetDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    // m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    // m_turningEncoder.setReverseDirection(turningEncoderReversed);
    m_driveMotor = new WPI_TalonFX(driveMotorCanId, CTREConstants.kCanivoreName);
    m_turningMotor = new WPI_TalonFX(turningMotorCanId, CTREConstants.kCanivoreName);

    m_driveMotor.configFactoryDefault(100);
    m_turningMotor.configFactoryDefault(100);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    m_driveMotor.setInverted(false);
    m_turningMotor.setInverted(true);

    m_driveMotor.setSensorPhase(false);
    m_turningMotor.setSensorPhase(false);

    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.closedloopRamp = 0.00;
    baseConfig.neutralDeadband = 0.005;
    baseConfig.nominalOutputForward = 0.0;
    baseConfig.nominalOutputReverse = 0.0;
    baseConfig.openloopRamp = 0.00;
    baseConfig.peakOutputForward = 1.0;
    baseConfig.peakOutputReverse = -1.0;
    baseConfig.statorCurrLimit.enable = true;
    baseConfig.statorCurrLimit.currentLimit = 40;
    baseConfig.statorCurrLimit.triggerThresholdCurrent = 40;
    baseConfig.statorCurrLimit.triggerThresholdTime = 0.5;
    baseConfig.supplyCurrLimit.enable = true;
    baseConfig.supplyCurrLimit.currentLimit = 30;
    baseConfig.supplyCurrLimit.triggerThresholdCurrent = 30;
    baseConfig.supplyCurrLimit.triggerThresholdTime = 1;
    baseConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    baseConfig.voltageCompSaturation = 11;
    baseConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

    TalonFXConfiguration turnConfig = baseConfig;
    turnConfig.remoteFilter0.remoteSensorDeviceID = turningEncoderCanId;
    turnConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    turnConfig.slot0.allowableClosedloopError = 50;
    turnConfig.slot0.closedLoopPeakOutput = 1.0;
    turnConfig.slot0.closedLoopPeriod = 20;
    turnConfig.slot0.integralZone = 100;
    turnConfig.slot0.kP = 0.0;
    turnConfig.slot0.kI = 0.0;
    turnConfig.slot0.kD = 0.0;
    turnConfig.slot0.kF = 0.0;
    turnConfig.motionAcceleration = ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared;
    turnConfig.motionCruiseVelocity = ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;
    turnConfig.motionCurveStrength = 2;

    m_turningMotor.configAllSettings(turnConfig, 100);
    m_turningMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    m_turningMotor.selectProfileSlot(0, 0);

    TalonFXConfiguration driveConfig = baseConfig;
    driveConfig.slot0.allowableClosedloopError = 0;
    driveConfig.slot0.closedLoopPeakOutput = 1;
    driveConfig.slot0.closedLoopPeriod = 10;
    driveConfig.slot0.integralZone = 100;
    driveConfig.slot0.kP = 0;
    driveConfig.slot0.kI = 0;
    driveConfig.slot0.kD = 0;
    driveConfig.slot0.kF = 0;

    m_driveMotor.configAllSettings(driveConfig, 100);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_driveMotor.selectProfileSlot(0, 0);
    m_driveMotor.configSelectedFeedbackCoefficient(1);

    m_driveMotor.setSafetyEnabled(false);
    m_turningMotor.setSafetyEnabled(false);

    // encoder for turning module
    m_turningEncoder = new CANCoder(turningEncoderCanId, CTREConstants.kCanivoreName);
    m_turningEncoder.configFactoryDefault(100);
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turningEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    m_turningEncoder.setPositionToAbsolute();
    m_turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 50);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_turningEncoderOffset = turningEncoderOffset;

    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveRatePerSecond(), new Rotation2d(getTurningPositionRadians()));
  }

  private double getDriveRatePerSecond() {
    // return m_driveMotor.getSelectedSensorVelocity();
    return m_driveMotor.getSelectedSensorVelocity() * 10 * ModuleConstants.kDriveEncoderDistancePerPulse;
  }

  private double getTurningPositionRadians() {
    double angle = m_turningEncoder.getPosition() - Units.radiansToDegrees(m_turningEncoderOffset);
    angle = Math.IEEEremainder(angle, 360);
    return Units.degreesToRadians(angle);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);
    m_desiredState = state;

    // if requested state is of ~0 speed, then don't move wheels back to zero,
    // just stop them
    if (Math.abs(state.speedMetersPerSecond) < 0.005) {
      stop();
      return;
    }

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController
        .calculate(getDriveRatePerSecond(), state.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController
        .calculate(getTurningPositionRadians(), state.angle.getRadians());
    final double turnFeedforward = m_turnFeedforward
        .calculate(m_turningPIDController.getSetpoint().velocity);

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public void stop() {
    m_turningMotor.set(ControlMode.PercentOutput, 0);
    m_driveMotor.set(ControlMode.PercentOutput, 0);
  }

  /** Zeros all the SwerveModule encoders. */
  // public void resetEncoders() {
  // m_driveEncoder.reset();
  // m_turningEncoder.reset();
  // }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module");
    builder.addDoubleProperty("Actual Drive m-s", () -> this.getDriveRatePerSecond(), null);
    builder.addDoubleProperty("Drive Position Meters",
        () -> (ModuleConstants.kDriveEncoderDistancePerPulse *
            m_driveMotor.getSelectedSensorPosition()),
        null);
    builder.addDoubleProperty("Actual Angle deg", () -> m_turningEncoder.getPosition(), null);
    builder.addDoubleProperty("Actual Angle with offset deg",
        () -> Units.radiansToDegrees(getTurningPositionRadians()), null);
    builder.addDoubleProperty("Desired Drive m-s", () -> m_desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Desired Angle deg", () -> m_desiredState.angle.getDegrees(), null);
  }
}
