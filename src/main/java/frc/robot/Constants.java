// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 8;

    public static final int kFrontLeftTurningMotorPort = 3;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int kFrontLeftTurningEncoderPort = 12;
    public static final int kRearLeftTurningEncoderPort = 13;
    public static final int kFrontRightTurningEncoderPort = 11;
    public static final int kRearRightTurningEncoderPort = 14;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final double kFrontLeftTurningEncoderOffsetRadians = Units.degreesToRadians(-40.00);
    public static final double kRearLeftTurningEncoderOffsetRadians = Units.degreesToRadians(76.82);
    public static final double kFrontRightTurningEncoderOffsetRadians = Units.degreesToRadians(97.21);
    public static final double kRearRightTurningEncoderOffsetRadians = Units.degreesToRadians(15.91);

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(20);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    public static final SPI.Port kGyroAhrsPort = SPI.Port.kMXP;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    // public static final double ksVolts = 1;
    // public static final double kvVoltSecondsPerMeter = 0.8;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 4.3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 8 * Math.PI;

    public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
    public static final double kWheelDiameterMeters = 2 * kWheelRadiusMeters;
    public static final double kDriveEncoderDistancePerPulse = 2.0 * Math.PI * kWheelRadiusMeters * (1.0 / 6.54)
        * (1.0 / (double) CTREConstants.kTalonFXEncoderResolution);

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) CTREConstants.kTalonFXEncoderResolution;

    public static final double kPModuleTurningController = 2.5;

    public static final double kPModuleDriveController = 2.0;

    public static final double kSModuleDriveFeedforward = 0.4;
    public static final double kVModuleDriveFeedforward = 2.2;

    public static final double kSModuleTurningFeedforward = 0.34;
    public static final double kVModuleTurningFeedforward = 0.32;
  }

  public static final class ShooterConstants {
    public static final int kTopMotorCanId = 1111;
    public static final int kBottomMotorCanId = 1111;
  }

  public static final class TurretConstants {
    public final double kEncoderTicksPerDegree = (231.0 / 56.0) * (62.0 / 13.0) * 2048 * (1.0 / 360.0);
    public final double kZeroOffsetDegrees = 88.5;
    public final double kMinAngle = -135;
    public final double kMaxAngle = 275;
    public final double kMaxAngularVelocity = 500.0; // keep within 560, started at 135, was 600
    public final double kMaxAngularAcceleration = 12000.0; // keep within 5700, started at 455
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double xboxDeadband = 0.09;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class CTREConstants {
    public static final int kTalonFXEncoderResolution = 2048;
    public static final String kCanivoreName = "canivore";
  }
}
