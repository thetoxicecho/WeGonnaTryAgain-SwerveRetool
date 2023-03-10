// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

  public static final double stickDeadband = 0.1;

  public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

  public static final double driveGearRatio = 6.75;

  public static final double angleGearRatio = 12.8;

  public static final double maxSpeed = 4;

  public static final double maxAngularVelocity = 10.0;

  public static final double driveKP = 0.05;
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKF = 0.0;

  public static final double trackWidth = 0.53; // TODO: This must be tuned to specific robot
  public static final double wheelBase = 0.53; // TODO: This must be tuned to specific robot

  public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  public static final class SwerveModuleConfigurations {
    public int moduleNumber;
    public int angleEncoderId;
    public int angleMotorId;
    public int driveMotorId;
    public double angleEncoderOffset;
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public double kNorm;

    public SwerveModuleConfigurations(int moduleNumber, int angleEncoderId, int angleMotorId, int driveMotorId,
        double angleEncoderOffset, double kP, double kI, double kD, double kF, double kNorm) {
      this.moduleNumber = moduleNumber;
      this.angleEncoderId = angleEncoderId;
      this.angleMotorId = angleMotorId;
      this.driveMotorId = driveMotorId;
      this.angleEncoderOffset = angleEncoderOffset;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kF = kF;
      this.kNorm = kNorm;
    }
  }

  public static final SwerveModuleConfigurations s_frontLeft = new SwerveModuleConfigurations(
      0,
      2,
      5,
      4,
      -220,
      0.150d,
      0.092d,
      0.001d,
      0.0,
      12);

  public static final SwerveModuleConfigurations s_frontRight = new SwerveModuleConfigurations(
      1,
      0,
      1,
      0,
      -203.5,
      0.150d,
      0.092d,
      0.001d,
      0.0,
      12);

  public static final SwerveModuleConfigurations s_backLeft = new SwerveModuleConfigurations(
      2,
      1,
      3,
      6,
      -211,
      0.150d,
      0.092d,
      0.001d,
      0.0,
      12);

  public static final SwerveModuleConfigurations s_backRight = new SwerveModuleConfigurations(
      3,
      3,
      7,
      2,
      -209,
      0.15d,
      0.092d,
      0.001d,
      0.0,
      12);
}