// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveTeleop;
import frc.robot.commands.ClawMechTester.ClawArmTester;
import frc.robot.commands.ClawMechTester.ClawIntakeTester;
import frc.robot.commands.ClawMechTester.ClawRotateTester;
import frc.robot.commands.ClawMechTester.setClawStateTest;
import frc.robot.commands.AutoClaw;
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
  private final ClawMechanism clawMechanism = new ClawMechanism();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private Trigger kA, kB, kY, kX, upPOV, downPOV, leftBumper, leftTrigger, rightBumper, rightTrigger;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver =
      new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean wantSlewRate = false;
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(swerveSubsystem, driver, fieldRelative, wantSlewRate));
    // Configure the trigger bindings
    configureBindings();
    downPOV.whileTrue(new ClawArmTester(clawMechanism, 0.2));
    rightTrigger.whileTrue(new ClawIntakeTester(clawMechanism, -0.75));
    rightBumper.whileTrue(new ClawIntakeTester(clawMechanism, 0.75));
    leftTrigger.whileTrue(new ClawIntakeTester(clawMechanism, 0.75));
    leftBumper.whileTrue(new ClawIntakeTester(clawMechanism, -0.75));
    upPOV.whileTrue(new ClawArmTester(clawMechanism, -0.2));
    kX.whileTrue(new ClawRotateTester(clawMechanism, -0.2));
    kY.whileTrue(new ClawRotateTester(clawMechanism, 0.2));
    kA.onTrue(new setClawStateTest(clawMechanism, 40, 0));
    kB.onTrue(new setClawStateTest(clawMechanism, 0, 0));
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
    kA = driver.a();
    kB = driver.b();
    kY = driver.y();
    kX = driver.x();
    upPOV = driver.povUp();
    downPOV = driver.povDown();
    leftBumper = driver.leftBumper();
    leftTrigger = driver.leftTrigger();
    rightBumper = driver.rightBumper();
    rightTrigger = driver.rightTrigger();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
