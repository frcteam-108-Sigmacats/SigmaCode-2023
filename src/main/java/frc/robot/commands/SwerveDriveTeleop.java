// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class SwerveDriveTeleop extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //Getting the x and y axis of the joystick at the end of this file
  private Translation2d translation;
  //Whether or not to use the gyro scope
  private boolean fieldRelative;
  
  private SwerveSubsystem swerve;
  private CommandXboxController driver;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveDriveTeleop(SwerveSubsystem swerve, CommandXboxController driver, boolean fieldRelative) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    this.driver = driver;
    this.fieldRelative = fieldRelative;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yAxis = -driver.getLeftY();
    double xAxis = -driver.getLeftX();
    double rotAxis = -driver.getRightX();

    yAxis = (Math.abs(yAxis) < SwerveConstants.deadband ? 0 : yAxis);
    xAxis = (Math.abs(xAxis) < SwerveConstants.deadband ? 0 : xAxis);
    rotAxis = (Math.abs(rotAxis) < SwerveConstants.deadband ? 0 : rotAxis);

    translation = new Translation2d(yAxis, xAxis).times(SwerveConstants.maxDriveSpeed);
    //rotation = rotAxis * SwerveConstants.maxTurnSpeed;
    swerve.drive(translation, rotAxis, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}