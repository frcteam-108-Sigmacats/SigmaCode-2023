// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawMechanism extends SubsystemBase {
  private CANSparkMax leftClawArmMotor;
  private CANSparkMax rightClawArmMotor;

  public SparkMaxPIDController rotateArmPID;
  private DigitalInput gamePieceDetect;

  private AbsoluteEncoder throughBoreAbs;
  /** Creates a new ExampleSubsystem. */
  public ClawMechanism() {
    leftClawArmMotor = new CANSparkMax(11, MotorType.kBrushless);
    rightClawArmMotor = new CANSparkMax(12, MotorType.kBrushless);

    leftClawArmMotor.restoreFactoryDefaults();
    rightClawArmMotor.restoreFactoryDefaults();

    rotateArmPID = leftClawArmMotor.getPIDController();
 
    throughBoreAbs = leftClawArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

    rotateArmPID.setFeedbackDevice(throughBoreAbs);
    rotateArmPID.setP(0.1);
    rotateArmPID.setI(0);
    rotateArmPID.setD(0);

    leftClawArmMotor.setIdleMode(IdleMode.kBrake);
    rightClawArmMotor.setIdleMode(IdleMode.kBrake);

    leftClawArmMotor.setSmartCurrentLimit(40);
    rightClawArmMotor.setSmartCurrentLimit(40);

    throughBoreAbs.setPositionConversionFactor(360);
    rightClawArmMotor.follow(leftClawArmMotor);

    leftClawArmMotor.burnFlash();
    rightClawArmMotor.burnFlash();

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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
