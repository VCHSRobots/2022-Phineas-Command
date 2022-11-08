// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftTurningEncoderOffsetRadians);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftTurningEncoderOffsetRadians);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightTurningEncoderOffsetRadians);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightTurningEncoderOffsetRadians);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(DriveConstants.kGyroAhrsPort);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0));

  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(7);

  static ProfiledPIDController thetaController = new ProfiledPIDController(2.0, 0, 0,
      new Constraints(DriveConstants.kMaxAngularSpeedRadiansPerSecond,
          12 * DriveConstants.kMaxAngularSpeedRadiansPerSecond));

  private boolean m_fieldRelative = true;
  private boolean m_prevResetOdometry = false;

  // tracking vars to output to dashboard
  private ChassisSpeeds m_lastChassisSpeedsDesired = new ChassisSpeeds();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    Thread zeroGyroThread = new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {

      }
    });
    zeroGyroThread.setDaemon(true);
    zeroGyroThread.start();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Shuffleboard.getTab("debug").add("front left", m_frontLeft);
    Shuffleboard.getTab("debug").add("front right", m_frontRight);
    Shuffleboard.getTab("debug").add("back left", m_rearLeft);
    Shuffleboard.getTab("debug").add("back right", m_rearRight);

    Shuffleboard.getTab("debug").addNumber("getAngle", () -> m_gyro.getAngle());
    Shuffleboard.getTab("debug").addNumber("getYaw", () -> m_gyro.getYaw());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getGyroRotation2d() {
    double angle = Math.IEEEremainder(-m_gyro.getAngle(), 360);
    return Rotation2d.fromDegrees(angle);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_gyro.setAngleAdjustment(-pose.getRotation().getDegrees() - m_gyro.getYaw());
    m_odometry.resetPosition(pose, getGyroRotation2d());
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
      Translation2d centerOfRotationMeters) {
    m_lastChassisSpeedsDesired = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    driveFromChassisSpeeds(m_lastChassisSpeedsDesired, centerOfRotationMeters);
  }

  public void driveFromChassisSpeeds(ChassisSpeeds chassisSpeed, Translation2d centerOfRotationMeters) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeed, centerOfRotationMeters);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void driveWithXbox(double driveY, double driveX, double xboxRot, boolean frontLeftCOR,
      boolean frontRightCOR) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xSpeedLimiter
        .calculate(MathUtil.applyDeadband(driveY, OIConstants.xboxDeadband))
        * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_ySpeedLimiter
        .calculate(MathUtil.applyDeadband(driveX, OIConstants.xboxDeadband))
        * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(xboxRot, 0.05))
        * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    final var centerOfRotationMeters = new Translation2d();

    drive(xSpeed, ySpeed, rot, m_fieldRelative, centerOfRotationMeters);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  // m_frontLeft.resetEncoders();
  // m_rearLeft.resetEncoders();
  // m_frontRight.resetEncoders();
  // m_rearRight.resetEncoders();
  // }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setFieldRelative() {
    m_fieldRelative = true;
  }

  public void changeOdometry(boolean setFieldRelative, boolean setRobotRelative, boolean resetOdometryLaunchPad,
      boolean resetOdometryFender) {
    if (setFieldRelative) {
      m_fieldRelative = true;
      resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    } else if (setRobotRelative) {
      m_fieldRelative = false;
    }
    if (resetOdometryLaunchPad && !m_prevResetOdometry) {
      Translation2d launchPad = new Translation2d(3.86, 5.46);
      resetOdometry(new Pose2d(launchPad, new Rotation2d()));
      m_fieldRelative = true;
    }
    if (resetOdometryFender && !m_prevResetOdometry) {
      Translation2d rightFender = new Translation2d(7.82, 2.95);
      resetOdometry(new Pose2d(rightFender, Rotation2d.fromDegrees(-111)));
      m_fieldRelative = true;
    }
    m_prevResetOdometry = resetOdometryLaunchPad;

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.setSmartDashboardType("Swerve Drive");
    builder.addBooleanProperty("Field Oriented", () -> m_fieldRelative, null);
    builder.addDoubleProperty("Rotation deg", () -> getGyroRotation2d().getDegrees(), null);
    builder.addDoubleProperty("Desired Vx m-s", () -> m_lastChassisSpeedsDesired.vxMetersPerSecond, null);
    builder.addDoubleProperty("Desired Vy m-s", () -> m_lastChassisSpeedsDesired.vyMetersPerSecond, null);
    builder.addDoubleProperty("Desired Rot rad-s", () -> m_lastChassisSpeedsDesired.omegaRadiansPerSecond, null);
  }
}
