// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetClawStates;
import frc.robot.commands.SwerveDriveTeleop;
import frc.robot.commands.ClawTesters.HoldArmTester;
import frc.robot.commands.ClawTesters.clawArmtester;
import frc.robot.commands.ClawTesters.clawIntakeHoldTester;
import frc.robot.commands.ClawTesters.clawIntakeTester;
import frc.robot.commands.ClawTesters.testingArmExtenders;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
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
  public final static Claw m_Claw = new Claw();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver =
      new CommandXboxController(0);

  public Trigger righTrigger, leftTrigger, rightBumper, leftBumper, kA, kB, kY, kX, upPov, downPov;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = false;
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(swerveSubsystem, driver, fieldRelative));
    //m_Claw.setDefaultCommand(new SetClawStates(m_Claw, 0, 0));
    // Configure the trigger bindings
    configureBindings();
    // leftTrigger.whileTrue(new RunIntake(1, 0.25));//Cone intake
    // righTrigger.whileTrue(new RunIntake(2, -0.25));//Cube intake
    leftTrigger.whileTrue(new testingArmExtenders(m_Claw, true));
    righTrigger.whileTrue(new testingArmExtenders(m_Claw, false));
    // rightBumper.whileTrue(new RunIntake(2, 0.25));//Cone intake
    // leftBumper.whileTrue(new RunIntake(1, -0.25));//Cube intake
    leftBumper.whileTrue(new clawIntakeTester(m_Claw, 0.85));
    leftBumper.whileFalse(new clawIntakeHoldTester(m_Claw));
    rightBumper.whileTrue(new clawIntakeTester(m_Claw, -0.85));
    rightBumper.whileFalse(new clawIntakeHoldTester(m_Claw));
    // kA.whileTrue(new SetClawStates(m_Claw, 1, 0.5));//Cube outtake
    // kB.whileTrue(new SetClawStates(m_Claw, 1, -0.5));//Cone outtake
    // kY.whileTrue(new SetClawStates(m_Claw, 2, 0.5));//Cube outtake
    // kX.whileTrue(new SetClawStates(m_Claw, 2, -0.5));//Cone outtake
    upPov.onTrue(new clawArmtester(m_Claw, 0.15));
    upPov.onFalse(new HoldArmTester(m_Claw));
    downPov.onTrue(new clawArmtester(m_Claw, -0.15));
    downPov.onFalse(new HoldArmTester(m_Claw));
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
    righTrigger = driver.rightTrigger();
    leftTrigger = driver.leftTrigger();
    leftBumper = driver.leftBumper();
    rightBumper = driver.rightBumper();
    kA = driver.a();
    kB = driver.b();
    kY = driver.y();
    kX = driver.x();
    upPov = driver.povUp();
    downPov = driver.povDown();
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("2Cones_Charge", new PathConstraints(1, 1));
    List<PathPlannerTrajectory> tryGroup = PathPlanner.loadPathGroup("Testing", new PathConstraints(1, 1), new PathConstraints(1, 1));
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", null);
    // An example command will be run in autonomous

    // new PPSwerveControllerCommand(tryGroup, swerveSubsystem::getPose, Constants.swerveKinematics, 
    // new PIDController(0.3, 0, 0), new PIDController(0.3, 0, 0), 
    // new PIDController(0.3, 0, 0), swerveSubsystem::setModuleStates, swerveSubsystem),
    // new InstantCommand(() -> swerveSubsystem.stopModules()));
    
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    new PIDConstants(0.0000001, 0, 0), new PIDConstants(-3.0, 0, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);

    Command fullauto = autoBuilder.fullAuto(tryGroup);
    return fullauto;
  }
}
