// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.DriveConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class JoystickCommand extends CommandBase {
  private final SwerveDriveSubsystem m_subsystem;

  private final double xAxis;
  private final double yAxis;
  private final double rotationAxis;

  private final boolean fieldOrientedControl;

  private final SlewRateLimiter xSlewy, ySlewy, rotationSlewy;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickCommand(SwerveDriveSubsystem subsystem, double xAxis, double yAxis, double rotationAxis, boolean fieldOrientedControl) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;

    this.fieldOrientedControl = fieldOrientedControl;

    this.xSlewy = new SlewRateLimiter(DriveConstants.teleDriveMaxAngularAccelerationPerSecond);
    this.ySlewy = new SlewRateLimiter(DriveConstants.teleDriveMaxAngularAccelerationPerSecond);
    this.rotationSlewy = new SlewRateLimiter(DriveConstants.teleDriveMaxAngularAccelerationPerSecond);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    // Getting the speeds of each axis from the joystick
    double xSpeed = this.xAxis;
    double ySpeed = this.yAxis;
    double rotationSpeed = this.rotationAxis;

    if(Math.abs(xSpeed) < 0.05)
    {
        xSpeed = 0.0;
    }

    if(Math.abs(ySpeed) < 0.05)
    {
        ySpeed = 0.0;
    }

    if(Math.abs(rotationSpeed) < 0.05)
    {
        rotationSpeed = 0.0;
    }

    xSpeed = xSlewy.calculate(xSpeed) * DriveConstants.teleDriveMaxSpeedMetersPerSecond;
    ySpeed = ySlewy.calculate(ySpeed) * DriveConstants.teleDriveMaxSpeedMetersPerSecond;
    rotationSpeed = rotationSlewy.calculate(rotationSpeed) * DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedControl)
    {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, m_subsystem.getRotation2d());
    }
    else
    {
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    }

    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
      m_subsystem.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
