// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CTREConstants;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.distanceRPMPoint;

public class Shooter extends SubsystemBase {

  WPI_TalonFX m_shootTalonTop = new WPI_TalonFX(ShooterConstants.kTopMotorCanId, CTREConstants.kCanivoreName);
  WPI_TalonFX m_shootTalonBot = new WPI_TalonFX(ShooterConstants.kBottomMotorCanId, CTREConstants.kCanivoreName);

  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_interpolatingSpeeds_top = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();
  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> m_interpolatingSpeeds_bot = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

  ShuffleboardTab debugTab = Shuffleboard.getTab("debug");

  /** Creates a new Shooter. */
  public Shooter() {
    distanceRPMPoint[] distanceRPMlist = {
        new distanceRPMPoint(8.0, 2290, 2520),
        new distanceRPMPoint(8.5, 2300, 2600),
        new distanceRPMPoint(9.25, 2320, 2660),
        new distanceRPMPoint(10, 2320, 2770),
        new distanceRPMPoint(10.5, 2420, 2850),
        new distanceRPMPoint(11, 2480, 2890),
        new distanceRPMPoint(11.5, 2530, 2920),
        new distanceRPMPoint(12, 2640, 3010),
        new distanceRPMPoint(12.5, 2730, 3020),
        new distanceRPMPoint(13, 2910, 3070),
        new distanceRPMPoint(13.2, 2950, 3080),
        // see github commit for changes
        new distanceRPMPoint(13.5, 3030, 3120),
        new distanceRPMPoint(14, 3130, 3170),
        new distanceRPMPoint(14.5, 3200, 3170),
        new distanceRPMPoint(15, 3530, 3220),
        new distanceRPMPoint(15.5, 3830, 3220),
        new distanceRPMPoint(16, 4160, 3270),
    };

    for (distanceRPMPoint point : distanceRPMlist) {
      m_interpolatingSpeeds_bot.put(new InterpolatingDouble(point.distance),
          new InterpolatingDouble(point.botRPM));
      m_interpolatingSpeeds_top.put(new InterpolatingDouble(point.distance),
          new InterpolatingDouble(point.topRPM));
    }

    debugTab.addNumber("Actual Top RPM", () -> getTopMotorRPM()).withPosition(4, 1).withSize(4, 3);
    // .withWidget(BuiltInWidgets.kGraph);
    debugTab.addNumber("Actual Bot RPM", () -> getBotMotorRPM()).withPosition(4, 4).withSize(4, 3);
    // .withWidget(BuiltInWidgets.kGraph);
    debugTab.addNumber("Top Setpoint", () -> getTopClosedLoopTarget());
    // .withWidget(BuiltInWidgets.kGraph);
    debugTab.addNumber("Bot Setpoint", () -> getBotClosedLoopTarget());
    // .withWidget(BuiltInWidgets.kGraph);

    TalonFXConfiguration baseConfig = new TalonFXConfiguration();
    baseConfig.closedloopRamp = 0.0;
    baseConfig.neutralDeadband = 0.005;
    baseConfig.nominalOutputForward = 0.0;
    baseConfig.nominalOutputReverse = 0.0;
    baseConfig.openloopRamp = 0.01;
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

    m_shootTalonBot.configFactoryDefault(100);
    m_shootTalonTop.configFactoryDefault(100);

    // TalonFXConfiguration topConfig = baseConfig;
    // top settings
    baseConfig.slot0.kI = 0.0;
    baseConfig.slot0.kD = 0.0;
    baseConfig.slot0.kF = 0.052;
    baseConfig.slot0.kP = 0.059; // 0.03
    m_shootTalonTop.configAllSettings(baseConfig, 100);

    // TalonFXConfiguration botConfig = baseConfig;
    // bot settings
    baseConfig.slot0.kI = 0.0;
    baseConfig.slot0.kD = 0.0;
    baseConfig.slot0.kF = 0.049; // after distance tuning, was 0.051
    baseConfig.slot0.kP = 0.3; // after distance tuning, was 0.03
    m_shootTalonBot.configAllSettings(baseConfig, 100);

    m_shootTalonBot.setNeutralMode(NeutralMode.Coast);
    m_shootTalonTop.setNeutralMode(NeutralMode.Coast);

    m_shootTalonBot.setInverted(false);
    m_shootTalonTop.setInverted(false);

    m_shootTalonBot.setSensorPhase(false);
    m_shootTalonTop.setSensorPhase(false);

    // initialize Turn Table CanCoder
    // set units of the CANCoder to radians, with velocity being radians per second

    var config = new CANCoderConfiguration();
    // config.sensorCoefficient = 2 * Math.PI / 4096.0;
    // config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

    m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 40);
    m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_shootTalonBot.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

    m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 40);
    m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_shootTalonTop.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100);

    setDefaultCommand(new RunCommand(this::stop, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // SHUFFLEBOARD HELPERS
  public double getTopMotorRPM() {
    return ticksPer100msToRPM(m_shootTalonTop.getSelectedSensorVelocity());
  }

  public double getBotMotorRPM() {
    return ticksPer100msToRPM(m_shootTalonBot.getSelectedSensorVelocity());
  }

  public double getBotClosedLoopTarget() {
    if (m_shootTalonBot.getControlMode() == ControlMode.PercentOutput) {
      return 0.0;
    }
    return ticksPer100msToRPM(m_shootTalonBot.getClosedLoopTarget());
  }

  public double getTopClosedLoopTarget() {
    if (m_shootTalonTop.getControlMode() == ControlMode.PercentOutput) {
      return 0.0;
    }
    return ticksPer100msToRPM(m_shootTalonTop.getClosedLoopTarget());
  }

  // END SHUFFLEBOARD HELPERS

  public void stop() {
    m_shootTalonBot.setVoltage(0);
    m_shootTalonTop.setVoltage(0);
  }

  // Sets speed for RPM.
  public void setSpeedsRPM(double topRPM, double botRPM) {
    setShootMotorsVelocityMode(rpmToTicksPer100ms(topRPM),
        rpmToTicksPer100ms(botRPM));
  }

  // Sets Speeds for Distance.
  public void setSpeedsDist(double distanceFeet) {
    setShootMotorsVelocityMode(rpmToTicksPer100ms(topFeetToRPM(distanceFeet)),
        rpmToTicksPer100ms(botFeetToRPM(distanceFeet)));
  }

  // Sets doubles to Talons.
  public void setShootMotorsVelocityMode(double shootTopSpeed, double shootBotSpeed) {
    m_shootTalonTop.set(ControlMode.Velocity, shootTopSpeed);
    m_shootTalonBot.set(ControlMode.Velocity, shootBotSpeed);
  }

  // Converts RPM to Ticks/100MS.
  public double rpmToTicksPer100ms(double rpm) {
    double minutesPerSecond = 1.0 / 60.0;
    double secondsPer100ms = 1.0 / 10.0;
    double ticksPerRotation = 2048;
    double ticksPer100ms = rpm * minutesPerSecond * secondsPer100ms * ticksPerRotation;
    return ticksPer100ms;
  }

  // Converts Ticks/100MS to RPM.
  public double ticksPer100msToRPM(double ticksPer100ms) {
    double secondsPerMinute = 60.0;
    double oneHundredMSPerSecond = 10.0;
    double rotationsPerTick = 1.0 / 2048;
    double RPM = ticksPer100ms * secondsPerMinute * oneHundredMSPerSecond * rotationsPerTick;
    return RPM;
  }

  // Equation for Top Motor.
  public double topFeetToRPM(double topfeet) {
    // double RPMquadtop = -1419.522 + 396.7329 * topfeet + -3.353022 * (topfeet *
    // topfeet);
    return m_interpolatingSpeeds_top.getInterpolated(new InterpolatingDouble(topfeet)).value;

  }

  // Equation for Bot Motor.
  public double botFeetToRPM(double botfeet) {
    return m_interpolatingSpeeds_bot.getInterpolated(new InterpolatingDouble(botfeet)).value;
  }

  public boolean isSpinningFastEnoughForBarf() {
    boolean isBotFast = ticksPer100msToRPM(m_shootTalonBot.getSelectedSensorVelocity()) > 1000;
    boolean isTopFast = ticksPer100msToRPM(m_shootTalonTop.getSelectedSensorVelocity()) > 1000;
    return isBotFast && isTopFast;
  }

  public void setBarfVoltage() {
    m_shootTalonBot.setVoltage(2.5);
    m_shootTalonTop.setVoltage(2.5);
  }
}
