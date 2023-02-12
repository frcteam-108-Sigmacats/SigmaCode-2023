// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class ClawMechanism extends SubsystemBase {
  private CANSparkMax leftClawArmMotor;
  private CANSparkMax rightClawArmMotor;

  private CANSparkMax clawIntake;
  private CANSparkMax clawRotate;

  public SparkMaxPIDController rotateArmPID;
  public SparkMaxPIDController rotateClawPID;
  private DigitalInput gamePieceDetect;

  private AbsoluteEncoder throughBoreAbs;
  private RelativeEncoder armEncoder;
  private RelativeEncoder rotateClawEnc;
  /** Creates a new ExampleSubsystem. */
  public ClawMechanism() {
    leftClawArmMotor = new CANSparkMax(11, MotorType.kBrushless);
    rightClawArmMotor = new CANSparkMax(12, MotorType.kBrushless);
    clawIntake = new CANSparkMax(13, MotorType.kBrushless);
    clawRotate = new CANSparkMax(14, MotorType.kBrushless);

    leftClawArmMotor.restoreFactoryDefaults();
    rightClawArmMotor.restoreFactoryDefaults();
    clawIntake.restoreFactoryDefaults();
    clawRotate.restoreFactoryDefaults();

    rotateArmPID = leftClawArmMotor.getPIDController();
    rotateClawPID = clawRotate.getPIDController();
 
    throughBoreAbs = leftClawArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    armEncoder = leftClawArmMotor.getEncoder();
    rotateClawEnc = clawRotate.getEncoder();

    armEncoder.setPositionConversionFactor(SwerveConstants.armGearRat * 360);
    rotateClawEnc.setPositionConversionFactor(SwerveConstants.clawRat * 360);

    rotateArmPID.setFeedbackDevice(armEncoder);
    rotateArmPID.setP(0.7);
    rotateArmPID.setI(0);
    rotateArmPID.setD(0);

    rotateClawPID.setFeedbackDevice(rotateClawEnc);
    rotateClawPID.setP(0.0005);
    rotateClawPID.setI(0.00001);
    rotateClawPID.setD(0);

    leftClawArmMotor.setIdleMode(IdleMode.kBrake);
    rightClawArmMotor.setIdleMode(IdleMode.kBrake);
    clawRotate.setIdleMode(IdleMode.kBrake);

    leftClawArmMotor.setSmartCurrentLimit(40);
    rightClawArmMotor.setSmartCurrentLimit(40);
    clawIntake.setSmartCurrentLimit(40);
    clawRotate.setSmartCurrentLimit(40);

    throughBoreAbs.setPositionConversionFactor(360);
    leftClawArmMotor.setInverted(false);
    rightClawArmMotor.setInverted(true);

    leftClawArmMotor.burnFlash();
    rightClawArmMotor.burnFlash();
    clawIntake.burnFlash();
    clawRotate.burnFlash();

    gamePieceDetect = new DigitalInput(2);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public boolean isGamePieceIn(){
    boolean isGamePiece = gamePieceDetect.get();
    return isGamePiece;
  }
  public boolean isRobotInComm(){
    //Get robot pose and if it is within the community zone return true
    return false;
  }
  public void testClawArms(double speed){
    leftClawArmMotor.set(speed);
    rightClawArmMotor.set(speed);

  }
  public void testClawIntake(double speed){
    clawIntake.set(speed);
  }
  public void testClawRotate(double speed){
    clawRotate.set(speed);
  }
  public void setArmPosition(double position){
    rotateArmPID.setReference(position, ControlType.kPosition);
  }
  public void setClawPosition(){
    rotateClawPID.setReference(rotateClawEnc.getPosition(), ControlType.kPosition);
  }
  public void getArmPositon(){
    System.out.println("Arm Position: " + armEncoder.getPosition());
  }
  public void getClawPos(){
    System.out.println("Claw Position: " + rotateClawEnc.getPosition());
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getArmPositon();
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
