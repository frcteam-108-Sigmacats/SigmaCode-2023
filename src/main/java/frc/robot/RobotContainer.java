// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetClawStates;
import frc.robot.commands.SwerveDriveTeleop;
import frc.robot.commands.ZeroModules;
import frc.robot.commands.ClawTesters.HoldArmTester;
import frc.robot.commands.ClawTesters.MoveArm;
import frc.robot.commands.ClawTesters.clawArmtester;
import frc.robot.commands.ClawTesters.clawIntakeHoldTester;
import frc.robot.commands.ClawTesters.clawIntakeTester;
import frc.robot.commands.ClawTesters.testingArmExtenders;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveSubsystem;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  public final static Claw m_Claw = new Claw();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public Trigger dRightTrigger, dLeftTrigger, dRightBumper, dLeftBumper, dKA, dKB, dKY, dKX, dUpPov, dDownPov, dLeftPov, dRightPov;
  public Trigger oRightTrigger, oLeftTrigger, oRightBumper, oLeftBumper, oKA, oKB, oKY, oKX, oUpPov, oDownPov, oLeftPov, oRightPov;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = false;
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(swerveSubsystem, driver, fieldRelative));
    //m_Claw.setDefaultCommand(new SetClawStates(m_Claw, 1));
    // Configure the trigger bindings
    configureBindings();

    //Driver's buttons
    dDownPov.whileTrue(new InstantCommand(()->swerveSubsystem.zeroHeading()));
    dUpPov.whileTrue(new ZeroModules(swerveSubsystem));
    dLeftTrigger.whileTrue(new RunIntake(1, -0.75));//negative is cone
    dLeftTrigger.whileFalse(new SetClawStates(m_Claw, 1));
    dRightTrigger.whileTrue(new RunIntake(2, 0.65));//positive is cube intake
    dRightTrigger.whileFalse(new SetClawStates(m_Claw, 5));//Sensor needs to be fixed in order to change the state to 2
    dLeftBumper.whileTrue(new RunIntake(3, -0.75));
    dLeftBumper.whileFalse(new SetClawStates(m_Claw, 7));
    dRightBumper.whileTrue(new RunIntake(4, 0.65));
    dRightBumper.whileFalse(new SetClawStates(m_Claw, 8));
    dKA.whileTrue(new InstantCommand(()->swerveSubsystem.zeroHeading()));
    // dKY.whileTrue(new SetClawStates(m_Claw, 2));
    // dKB.whileTrue(new SetClawStates(m_Claw, 3));
    // dKX.whileTrue(new SetClawStates(m_Claw, 4));
    // dLeftPov.whileTrue(new MoveArm(m_Claw, 0.5));
    // dRightPov.whileTrue(new MoveArm(m_Claw, -0.5));

    //Operator's buttons
    oLeftBumper.whileTrue(new clawIntakeTester(m_Claw, 0.85)); //Positive is Outtake cone
    oLeftBumper.whileFalse(new clawIntakeHoldTester(m_Claw));
    oRightBumper.whileTrue(new clawIntakeTester(m_Claw, -0.5 )); //Negative is Outtake cube
    oRightBumper.whileFalse(new clawIntakeHoldTester(m_Claw));
    oKA.whileTrue(new SetClawStates(m_Claw, 0));
    oKY.whileTrue(new SetClawStates(m_Claw, 2));
    oKB.whileTrue(new SetClawStates(m_Claw, 3));
    oKX.whileTrue(new SetClawStates(m_Claw, 4 ));
    oLeftPov.whileTrue(new MoveArm(m_Claw, 0.5));
    oRightPov.whileTrue(new MoveArm(m_Claw, -0.5));
    oDownPov.onTrue(new testingArmExtenders(m_Claw, true));
    oUpPov.onTrue(new testingArmExtenders(m_Claw, false));
    
    
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
    dLeftPov = driver.povLeft();
    dRightPov = driver.povRight();
    
    
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
    oLeftPov = operator.povLeft();
    oRightPov = operator.povLeft();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("Try3", new PathConstraints(4, 4));
    // List<PathPlannerTrajectory> tryGroup = PathPlanner.loadPathGroup("PathTesting", new PathConstraints(3, 4));
    //List<PathPlannerTrajectory> blue = PathPlanner.loadPathGroup("Blue", new PathConstraints(4, 4));
    //List<PathPlannerTrajectory> low = PathPlanner.loadPathGroup("Blue_down", new PathConstraints(4, 4));
    HashMap<String, Command> eventMap = new HashMap<>();
    //List<PathPlannerTrajectory> knock = PathPlanner.loadPathGroup("Knockout", new PathConstraints(4, 4), new PathConstraints(4, 4), new PathConstraints(4, 4), new PathConstraints(1, 1), new PathConstraints(4, 4));
    List<PathPlannerTrajectory> blue = PathPlanner.loadPathGroup("Up", new PathConstraints(4, 4));
    List<PathPlannerTrajectory> red = PathPlanner.loadPathGroup("Up", new PathConstraints(4, 4));

    eventMap.put("intakecone", new RunIntake(1, -0.5));
    eventMap.put("intakecube", new RunIntake(5, 0.5));//Fix sensor before 2nd case
    eventMap.put("drivecone", new SetClawStates(m_Claw, 1));//Cone
    eventMap.put("drivecube", new SetClawStates(m_Claw, 6));//Cube
    eventMap.put("highpos", new SetClawStates(m_Claw, 2));
    eventMap.put("midpos", new SetClawStates(m_Claw, 3));
    eventMap.put("lowpos", new SetClawStates(m_Claw, 4));
    eventMap.put("outtakecube", new clawIntakeTester(m_Claw, -0.75));
    eventMap.put("outtakecone", new clawIntakeTester(m_Claw, 0.85));
    

    // new PPSwerveControllerCommand(tryGroup, swerveSubsystem::getPose, Constants.swerveKinematics, 
    // new PIDController(0.3, 0, 0), new PIDController(0.3, 0, 0), 
    // new PIDController(0.3, 0, 0), swerveSubsystem::setModuleStates, swerveSubsystem),
    // new InstantCommand(() -> swerveSubsystem.stopModules()));
    
    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.01, 0, 0), new PIDConstants(0, 0, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command fullauto = autoBuilder.fullAuto(blue);

    // SwerveAutoBuilder autoBuilder2 = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.01, 0, 0), new PIDConstants(0, 0, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command fullauto2 = autoBuilder2.fullAuto(blue);

    // SwerveAutoBuilder autoBuilder3= new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.01, 0, 0), new PIDConstants(0, 0, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command fullauto3 = autoBuilder3.fullAuto(low);

    // SwerveAutoBuilder autoBuilder4= new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.01, 0, 0), new PIDConstants(0, 0, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    // Command fullauto4 = autoBuilder4.fullAuto(low);

    // SwerveAutoBuilder autoBuilder5= new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    // new PIDConstants(0.01, 0, 0), new PIDConstants(0, 0, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    // Command midblue = autoBuilder5.fullAuto(knock);

    // chooser.addOption("Top Blue with Charging", fullauto);
    // chooser.addOption("Top Red with Charging", fullauto2);
    // chooser.addOption("Blue low", fullauto3);
    // chooser.addOption("Red low", fullauto4);
    // chooser.setDefaultOption("Nothing", null);

    SwerveAutoBuilder autoBlue = new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    new PIDConstants(0.01, 0, 0), new PIDConstants(0, 0, 0), swerveSubsystem::setModuleStates, eventMap, false, swerveSubsystem);
    Command loadZoneBlue = autoBlue.fullAuto(blue);
    
    SwerveAutoBuilder autoRed= new SwerveAutoBuilder(swerveSubsystem::getPose, swerveSubsystem::resetOdometry, SwerveConstants.swerveKinematics,
    new PIDConstants(0.01, 0, 0), new PIDConstants(0, 0, 0), swerveSubsystem::setModuleStates, eventMap, true, swerveSubsystem);
    Command loadZoneRed = autoRed.fullAuto(blue);

    //SmartDashboard.putData("Auto Chooser", chooser);
    //return chooser.getSelected();

    //Blue with taxi
    //return loadZoneBlue;


    //Red with taxi
    return loadZoneRed;
  }
}
