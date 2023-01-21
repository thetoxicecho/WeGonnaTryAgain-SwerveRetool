// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SwerveModule m_frontLeft = new SwerveModule(Constants.s_frontLeft);
  SwerveModule m_frontRight = new SwerveModule(Constants.s_frontRight);
  SwerveModule m_backLeft = new SwerveModule(Constants.s_backLeft);
  SwerveModule m_backRight = new SwerveModule(Constants.s_backRight);
  
  public Swerve() {
  }

  @Override
  public void periodic() {

  }

  public void resetAllModulesToAbsolute() {

  }

  public void PIDLoop() {
    m_frontLeft.moveToTargetAngle();
    m_frontRight.moveToTargetAngle();
    m_backLeft.moveToTargetAngle();
    m_backRight.moveToTargetAngle();  
  }

  public void setModuleAngles(double targetAngle) {
    m_frontLeft.setTargetAngle(targetAngle);
    m_frontRight.setTargetAngle(targetAngle);
    m_backLeft.setTargetAngle(targetAngle);
    m_backRight.setTargetAngle(targetAngle);
  }

  public void percentZero() {
    
  }
}
