// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */
  private final Drivetrain m_Drivetrain;
  DoubleSupplier m_leftSpeed, m_forwardSpeed, m_radiansRotation;
  public SwerveDrive(Drivetrain drivetrain, DoubleSupplier forwardSpeed, DoubleSupplier leftSpeed, DoubleSupplier radiansRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Drivetrain = drivetrain;
    m_leftSpeed = leftSpeed;
    m_forwardSpeed = forwardSpeed;
    m_radiansRotation = radiansRotation;
    addRequirements(drivetrain);
    // System.out.println("Spin!");
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Spin! 2");
    // m_Drivetrain.spinMotorsTest(0.2);
    m_Drivetrain.swerveDrive(
      (m_leftSpeed.getAsDouble() < 0.02 && m_leftSpeed.getAsDouble() > -0.02) ? 0 : m_leftSpeed.getAsDouble(),
      (m_forwardSpeed.getAsDouble() < 0.02 && m_forwardSpeed.getAsDouble() > -0.02) ? 0 : m_forwardSpeed.getAsDouble(),
      (m_radiansRotation.getAsDouble() < 0.02 && m_radiansRotation.getAsDouble() > -0.02) ? 0 : m_radiansRotation.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Drivetrain.swerveDrive(
      // check if the speeds are not in the middle
      (m_leftSpeed.getAsDouble() < 0.02 && m_leftSpeed.getAsDouble() > -0.02) ? 0 : m_leftSpeed.getAsDouble(),
      (m_forwardSpeed.getAsDouble() < 0.02 && m_forwardSpeed.getAsDouble() > -0.02) ? 0 : m_forwardSpeed.getAsDouble(),
      (m_radiansRotation.getAsDouble() < 0.02 && m_radiansRotation.getAsDouble() > -0.02) ? 0 : m_radiansRotation.getAsDouble());  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.swerveDrive(0.0, 0.0, 0.0);
    // m_Drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
