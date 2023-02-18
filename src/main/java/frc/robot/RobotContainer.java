// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveDriveTeleop;
import frc.robot.commands.ClawMechTester.ClawArmTester;
import frc.robot.commands.ClawMechTester.ClawIntakeTester;
import frc.robot.subsystems.ClawMechanism;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ClawMechanism clawMech = new ClawMechanism();
  private Trigger leftTrigger, rightTrigger, leftBumper, rightBumper, kA, kB, kY, kX, upPov, downPov;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver =
      new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(swerveSubsystem, driver, fieldRelative));
    // Configure the trigger bindings
    configureBindings();
    leftTrigger.whileTrue(new ClawIntakeTester(clawMech, 0.75));//Cone intake
    leftBumper.whileTrue(new ClawIntakeTester(clawMech, -0.75));//Cone outtake
    rightTrigger.whileTrue(new ClawIntakeTester(clawMech, -0.5));//Cube intake
    rightBumper.whileTrue(new ClawIntakeTester(clawMech, 0.5));//Cube outtake
    upPov.whileTrue(new ClawArmTester(clawMech, 0.3));//Should extend arm
    downPov.whileTrue(new ClawArmTester(clawMech, -0.3));//Should retract arm
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    leftTrigger = driver.leftTrigger();
    rightTrigger = driver.rightTrigger();
    upPov = driver.povUp();
    downPov = driver.povDown();
    kA = driver.a();
    kB = driver.b();
    kY = driver.y();
    kX = driver.x();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
