// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.CTREConstants;

public class Turret extends SubsystemBase {

  WPI_TalonFX m_turnTableTalon = new WPI_TalonFX(TurretConstants.kTurretMotorCanId, CTREConstants.kCanivoreName);

  CANCoder m_turntableEncoder = new CANCoder(TurretConstants.kTurretEncoderCanId, CTREConstants.kCanivoreName);

  private ProfiledPIDController m_turretPIDController = new ProfiledPIDController(0.08, 0, 0,
      new Constraints(TurretConstants.kMaxAngularVelocity, TurretConstants.kMaxAngularAcceleration));

  /** Creates a new Turret. */
  public Turret() {
    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.closedloopRamp = 0.0;
    baseConfig.neutralDeadband = 0.005;
    baseConfig.nominalOutputForward = 0.0;
    baseConfig.nominalOutputReverse = 0.0;
    baseConfig.openloopRamp = 0.00;
    baseConfig.peakOutputForward = 1;
    baseConfig.peakOutputReverse = -1;
    baseConfig.statorCurrLimit.enable = true;
    baseConfig.statorCurrLimit.currentLimit = 30;
    baseConfig.statorCurrLimit.triggerThresholdCurrent = 30;
    baseConfig.statorCurrLimit.triggerThresholdTime = 1;
    baseConfig.supplyCurrLimit.enable = true;
    baseConfig.supplyCurrLimit.currentLimit = 30;
    baseConfig.supplyCurrLimit.triggerThresholdCurrent = 30;
    baseConfig.supplyCurrLimit.triggerThresholdTime = 1;
    baseConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    baseConfig.voltageCompSaturation = 10;
    baseConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    baseConfig.slot0.allowableClosedloopError = 20;
    baseConfig.slot0.closedLoopPeakOutput = 1.0;
    baseConfig.slot0.closedLoopPeriod = 20;
    baseConfig.slot0.integralZone = 100;

    TalonFXConfiguration turnTableConfig = baseConfig;
    turnTableConfig.peakOutputForward = .33;
    turnTableConfig.peakOutputReverse = -.33;
    // 0.1 @ 10 degree error
    // (0.1 * 1023) / (deg error * ticks / degree)
    turnTableConfig.slot0.kP = (1 * 1023) / (230 * TurretConstants.kEncoderTicksPerDegree);
    turnTableConfig.slot0.kI = 0;
    turnTableConfig.slot0.kD = 0;
    turnTableConfig.slot0.kF = 0;
    turnTableConfig.slot0.allowableClosedloopError = 0.2 * TurretConstants.kEncoderTicksPerDegree;
    turnTableConfig.supplyCurrLimit.currentLimit = 10;
    turnTableConfig.supplyCurrLimit.enable = true;
    turnTableConfig.supplyCurrLimit.triggerThresholdCurrent = 10;
    turnTableConfig.supplyCurrLimit.triggerThresholdTime = 0.5;
    turnTableConfig.forwardSoftLimitEnable = true;
    turnTableConfig.forwardSoftLimitThreshold = TurretConstants.kMaxAngle * TurretConstants.kEncoderTicksPerDegree;
    turnTableConfig.reverseSoftLimitEnable = true;
    turnTableConfig.reverseSoftLimitThreshold = TurretConstants.kMinAngle * TurretConstants.kEncoderTicksPerDegree;
    // // [deg / s*s] * [ticks / deg] * [s / 100ms] = [tick / (100ms * s)]
    // ticks per 100ms per sec
    turnTableConfig.motionAcceleration = TurretConstants.kMaxAngularAcceleration
        * TurretConstants.kEncoderTicksPerDegree * (1.0 / 10.0);
    // // [deg / s] * [tick / deg] * [s / 100ms] = [tick / 100ms]
    turnTableConfig.motionCruiseVelocity = TurretConstants.kMaxAngularVelocity * TurretConstants.kEncoderTicksPerDegree
        * (1.0 / 10.0);
    turnTableConfig.motionCurveStrength = 5;

    m_turnTableTalon.configFactoryDefault(100);
    m_turnTableTalon.setNeutralMode(NeutralMode.Brake);
    m_turnTableTalon.setInverted(false);
    m_turnTableTalon.configAllSettings(turnTableConfig, 100);

    m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_turnTableTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

    // initialize Turn Table CanCoder
    // set units of the CANCoder to radians, with velocity being radians per second

    var config = new CANCoderConfiguration();
    // config.sensorCoefficient = 2 * Math.PI / 4096.0;
    // config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

    m_turntableEncoder.configFactoryDefault(100);
    m_turntableEncoder.configAllSettings(config);

    m_turntableEncoder.setPositionToAbsolute(50);
    m_turntableEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 50);

    m_turretPIDController.disableContinuousInput();
    m_turretPIDController.setTolerance(0.5);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        if ((m_turntableEncoder.getPosition() - TurretConstants.kZeroOffsetDegrees) < TurretConstants.kMinAngle) {
          m_turntableEncoder.setPosition(m_turntableEncoder.getPosition() + 360, 50);
        } else if ((m_turntableEncoder.getPosition()
            - TurretConstants.kZeroOffsetDegrees) > TurretConstants.kMaxAngle) {
          m_turntableEncoder.setPosition(m_turntableEncoder.getPosition() - 360, 50);
        }
        Thread.sleep(100);
        m_turnTableTalon.setSelectedSensorPosition(getTurretAngleDegrees() * TurretConstants.kEncoderTicksPerDegree);
      } catch (Exception e) {

      }
    }).start();

    setDefaultCommand(new RunCommand(() -> stop(), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // TurnTable Funtions.
  public void TurnTable(boolean rightSideTurnTable,
      boolean leftSideTurnTable) {
    double turntableSpeed = 0;
    if (rightSideTurnTable) {
      turntableSpeed = 0.07;
    }
    if (leftSideTurnTable) {
      turntableSpeed = -0.07;
    }
    m_turnTableTalon.set(ControlMode.PercentOutput, turntableSpeed);
  }

  public void aimTurret(double angleYawDegreesOffset) {
    if (Math.abs(angleYawDegreesOffset) < 0.5) {
      m_turnTableTalon.setVoltage(0);
      return;
    }
    double targetAngle = getTurretAngleDegrees() + angleYawDegreesOffset;
    setTurretAngle(targetAngle);
  }

  public void setTurretAngle(double angleTargetDegrees) {
    double newAngle = angleTargetDegrees % 360;
    if (newAngle < TurretConstants.kMinAngle) {
      newAngle += 360;
    } else if (newAngle > TurretConstants.kMaxAngle) {
      newAngle -= 360;
    }
    double targetAngleTicks = angleDegreesToEncoderTicks(newAngle);
    // kS to overcome friction
    double kS = Math.copySign(0.02, targetAngleTicks -
        m_turnTableTalon.getSelectedSensorPosition());
    m_turnTableTalon.set(ControlMode.MotionMagic, targetAngleTicks,
        DemandType.ArbitraryFeedForward, kS);
  }

  public double angleDegreesToEncoderTicks(double degrees) {
    return (degrees) * TurretConstants.kEncoderTicksPerDegree;
  }

  public double getTurretAngleDegrees() {
    double angleDegrees = (m_turntableEncoder.getPosition() - TurretConstants.kZeroOffsetDegrees);
    return angleDegrees;
  }

  public double getTurretAngleCANcoder() {
    return m_turntableEncoder.getPosition();
  }

  public void stop() {
    m_turnTableTalon.stopMotor();
  }
}
