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
  private final CommandXboxController operator = new CommandXboxController(1);

  public Trigger dRightTrigger, dLeftTrigger, dRightBumper, dLeftBumper, dKA, dKB, dKY, dKX, dUpPov, dDownPov;
  public Trigger oRightTrigger, oLeftTrigger, oRightBumper, oLeftBumper, oKA, oKB, oKY, oKX, oUpPov, oDownPov;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = false;
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(swerveSubsystem, driver, fieldRelative));
    //m_Claw.setDefaultCommand(new SetClawStates(m_Claw, 1));
    // Configure the trigger bindings
    configureBindings();
    // leftTrigger.whileTrue(new RunIntake(1, 0.25));//Cone intake
    // righTrigger.whileTrue(new RunIntake(2, -0.25));//Cube intake
    // leftTrigger.whileTrue(new testingArmExtenders(m_Claw, true));
    // righTrigger.whileTrue(new testingArmExtenders(m_Claw, false));
    dLeftTrigger.whileTrue(new RunIntake(1, -0.5));//negative  is cone intake
    dLeftTrigger.whileFalse(new SetClawStates(m_Claw, 1));
    dRightTrigger.whileTrue(new RunIntake(2, 0.5));//positive is cube intake
    dRightTrigger.whileFalse(new SetClawStates(m_Claw, 1));
    // rightBumper.whileTrue(new RunIntake(2, 0.25));//Cone intake
    // leftBumper.whileTrue(new RunIntake(1, -0.25));//Cube intake
    dLeftBumper.whileTrue(new clawIntakeTester(m_Claw, 0.85));
    dLeftBumper.whileFalse(new clawIntakeHoldTester(m_Claw));
    dRightBumper.whileTrue(new clawIntakeTester(m_Claw, -0.85));
    dRightBumper.whileFalse(new clawIntakeHoldTester(m_Claw));
    dKA.whileTrue(new SetClawStates(m_Claw, 0));//Cube outtake
    dKB.whileTrue(new SetClawStates(m_Claw, 2));//Cone outtake
    dKY.whileTrue(new SetClawStates(m_Claw, 3));//Cube outtake
    dKX.whileTrue(new SetClawStates(m_Claw, 4 ));//Cone outtake
    dUpPov.whileTrue(new clawArmtester(m_Claw, 0.15));
    dUpPov.whileFalse(new clawArmtester(m_Claw, 0));
    dDownPov.whileTrue(new clawArmtester(m_Claw, -0.15));
    dDownPov.whileFalse(new clawArmtester(m_Claw, 0));
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
    dRightTrigger = driver.rightTrigger();
    dLeftTrigger = driver.leftTrigger();
    dLeftBumper = driver.leftBumper();
    dRightBumper = driver.rightBumper();
    dKA = driver.a();
    dKB = driver.b();
    dKY = driver.y();
    dKX = driver.x();
    dUpPov = driver.povUp();
    dDownPov = driver.povDown();
    
    oRightTrigger = operator.rightTrigger();
    oLeftTrigger = operator.leftTrigger();
    oLeftBumper = operator.leftBumper();
    oRightBumper = operator.rightBumper();
    oKA = operator.a();
    oKB = operator.b();
    oKY = operator.y();
    oKX = operator.x();
    oUpPov = operator.povUp();
    oDownPov = operator.povDown();
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
