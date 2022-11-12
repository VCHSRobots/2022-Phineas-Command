// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CTREConstants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimberConstants.kSolenoidForward,
      ClimberConstants.kSolenoidReverse);;

  private WPI_TalonFX m_master = new WPI_TalonFX(ClimberConstants.kClimberMotor1, CTREConstants.kCanivoreName);
  private WPI_TalonFX m_follower_1 = new WPI_TalonFX(ClimberConstants.kClimberMotor2, CTREConstants.kCanivoreName);
  private WPI_TalonFX m_follower_2 = new WPI_TalonFX(ClimberConstants.kClimberMotor3, CTREConstants.kCanivoreName);

  private DigitalInput m_bottomLeftLimit = new DigitalInput(ClimberConstants.kLeftBottomLimitDio);
  private DigitalInput m_bottomRightLimit = new DigitalInput(ClimberConstants.kRightBottomLimitDio);
  public double m_encoderValue;
  private boolean m_hasBeenCalibrated = false;

  /** Creates a new Climber. */
  public Climber() {
    // sensors send
    Shuffleboard.getTab("computil").addBoolean("Left Limit", () -> !m_bottomLeftLimit.get());
    Shuffleboard.getTab("computil").addBoolean("Right Limit", () -> !m_bottomRightLimit.get());
    Shuffleboard.getTab("computil").addNumber("tePosInches", () -> getArmPositionInches());
    Shuffleboard.getTab("computil").addNumber("teCurrent", () -> m_master.getStatorCurrent());
    Shuffleboard.getTab("super").addNumber("climbenc", () -> m_master.getSelectedSensorPosition()).withPosition(3, 0);

    // motor configs
    m_master.configFactoryDefault(50);
    m_follower_1.configFactoryDefault(50);
    m_follower_2.configFactoryDefault(50);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.slot0.kP = 0.11;
    config.slot0.kI = 0.0;
    config.slot0.kD = 0.5;
    config.slot0.kF = 0.0;
    config.motionAcceleration = 24 / ClimberConstants.kInchesPerEncoderTick; // 6 in / s*s
    config.motionCruiseVelocity = 18 / ClimberConstants.kInchesPerEncoderTick; // 12 in per sec
    config.motionCurveStrength = 6;
    config.supplyCurrLimit.currentLimit = 30;
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 30;
    config.supplyCurrLimit.triggerThresholdTime = 0.5;

    m_master.configAllSettings(config);

    m_master.setNeutralMode(NeutralMode.Brake);
    m_follower_1.setNeutralMode(NeutralMode.Brake);
    m_follower_2.setNeutralMode(NeutralMode.Brake);

    m_follower_1.follow(m_master);
    m_follower_2.follow(m_master);

    m_master.setInverted(false);
    m_follower_1.setInverted(InvertType.FollowMaster);
    m_follower_2.setInverted(InvertType.OpposeMaster);

    m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_master.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 50);
    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);
    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_follower_1.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 50);
    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);
    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);
    m_follower_2.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255);

    // motor configs
    m_master.configFactoryDefault(100);
    m_follower_1.configFactoryDefault(100);
    m_follower_2.configFactoryDefault(100);

    m_master.setNeutralMode(NeutralMode.Brake);
    m_follower_1.setNeutralMode(NeutralMode.Brake);
    m_follower_2.setNeutralMode(NeutralMode.Brake);

    m_master.setSelectedSensorPosition(0);
    m_master.configForwardSoftLimitEnable(true);
    m_master.configForwardSoftLimitThreshold(300000);

    // init solenoids
    m_solenoid.set(Value.kReverse);

    setDefaultCommand(new RunCommand(() -> armsStop(), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // THis is the control function that is called from tele_periodic and
  // test_periodic.
  // NOTE: For the new climber auto functions to work without breaking the robot,
  // This routine must be called on every TelePeriodic cycle!!
  public void control(boolean shortHookBack, boolean shortHookForward, boolean armsUp, boolean armsDown,
      boolean climbNext, boolean climbFinish, double climbArmSpeedDown, double climbArmSpeedUp) {

    // solenoids
    if (shortHookBack) {
      hooksReverse();
    } else if (shortHookForward) {
      hooksForward();
    }

    if (armsUp && armsDown) {
      setClimberToZero();
    } else {
      // motors
      if (armsUp) {
        armsUp();
      } else if (armsDown) {
        armsDown();
      } else if (climbArmSpeedDown > 0.0) {
        armsDownAtSpeed(climbArmSpeedDown);
      } else if (climbArmSpeedUp > 0.0) {
        armsUpAtSpeed(climbArmSpeedUp);
      } else {
        armsStop();
      }
    }
  }

  // Teleop Periodic (Note: I don't think this is ever called?)
  public void climberMove() {
    // limit switch
    // if (bottomLimit.get()) {
    m_follower_1.setSelectedSensorPosition(0);
    m_follower_2.setSelectedSensorPosition(0);
    // }
  }

  public void hooksForward() {
    m_solenoid.set(Value.kForward);
  }

  public void hooksReverse() {
    m_solenoid.set(Value.kReverse);

  }

  // Returns true if the hooks are forward.
  public boolean getHooksInForward() {
    if (m_solenoid.get() == Value.kForward) {
      return true;
    }
    return false;
  }

  // Returns the Arm position in inches. The arms are at zero
  // when the magnetic senser is actived. As they extend up,
  // the position increases. This number is ONLY valid
  // if the arms are calibrated.
  public double getArmPositionInches() {
    return m_master.getSelectedSensorPosition() * ClimberConstants.kInchesPerEncoderTick_Auto;
  }

  public void armsUp() {
    m_master.set(ControlMode.PercentOutput, 0.95); // was 0.9
  }

  public void armsUpAtSpeed(double percent) {
    m_master.set(ControlMode.PercentOutput, percent);
  }

  public void armsDown() {
    if (getClimberZero()) {
      m_master.set(ControlMode.PercentOutput, 0);
    } else if (getArmPositionInches() < ClimberConstants.kSlowDownDistanceInches) {
      m_master.set(ControlMode.PercentOutput, -0.3);
    } else {
      m_master.set(ControlMode.PercentOutput, -0.95); // was -0.85
    }
  }

  public void armsDownAtSpeed(double percent) {
    double x = -percent;
    if (getClimberZero()) {
      m_master.set(ControlMode.PercentOutput, 0);
    } else if (getArmPositionInches() < ClimberConstants.kSlowDownDistanceInches) {
      m_master.set(ControlMode.PercentOutput, -0.3);
    } else {
      m_master.set(ControlMode.PercentOutput, x);
    }
  }

  public void armsStop() {
    m_master.set(ControlMode.PercentOutput, 0);
  }

  public void resetPosition() {
    m_master.setSelectedSensorPosition(0);
  }

  public boolean setClimberToZero() {
    if (getClimberZero()) {
      // 13 to 62, 52 to 231, GEAR RATIO: 21.19
      m_hasBeenCalibrated = true;
      m_master.set(ControlMode.PercentOutput, 0);
      m_master.setSelectedSensorPosition(0);
      return true;
    } else {
      m_hasBeenCalibrated = false;
      m_master.set(ControlMode.PercentOutput, -0.07);
      return false;
    }
  }

  // This is called by RobotPeriodic. If ever the climber arms are down,
  // the climber will be calibrated from that point on.
  public void checkZero() {
    if (getClimberZero()) {
      m_master.setSelectedSensorPosition(0);
      m_hasBeenCalibrated = true;
    }
  }

  public boolean isCalibrated() {
    return m_hasBeenCalibrated;
  }

  // Returns TRUE if either arm is all the way down.
  public boolean getClimberZero() {
    // add other limit too?
    return !m_bottomRightLimit.get() || !m_bottomLeftLimit.get();
  }
}
