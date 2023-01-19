// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SwerveModule m_frontLeft = new SwerveModule(0, 2, 5, 4, 0);
  SwerveModule m_frontRight = new SwerveModule(1, 0, 1, 0, 0);
  SwerveModule m_backLeft = new SwerveModule(2, 1, 3, 6, 0);
  SwerveModule m_backRight = new SwerveModule(3, 3, 7, 2, 0);
  
  public Swerve() {
  }

  @Override
  public void periodic() {

  }

  public void resetAllModulesToAbsolute() {

  }

  public void PIDLoop(double targetAngle) {
    m_frontLeft.moveToTargetAngle(targetAngle);
    m_frontRight.moveToTargetAngle(targetAngle);
    m_backLeft.moveToTargetAngle(targetAngle);
    m_backRight.moveToTargetAngle(targetAngle);  
  }

  public void percentZero() {
    
  }
}
