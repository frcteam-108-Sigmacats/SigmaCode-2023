// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveSetUp;

public class SwerveSubsystem extends SubsystemBase {
  //Construction of each module

  //Front Left
  private SwerveModule frontLeft = new SwerveModule(SwerveSetUp.fLDriveMotor, SwerveSetUp.isFLDriveMotorReversed, 
  SwerveSetUp.frontLDriveKP, SwerveSetUp.frontLDriveKI, SwerveSetUp.frontLDriveKD, SwerveSetUp.fLTurnMotor, SwerveSetUp.isFLTurnMotorReversed, 
  SwerveSetUp.frontLTurnKP, SwerveSetUp.frontLTurnKI, SwerveSetUp.frontLTurnKD, SwerveSetUp.fLChassisAngleOffset);
  //Front Right
  private SwerveModule frontRight = new SwerveModule(SwerveSetUp.fRDriveMotor, SwerveSetUp.isFRDriveMotorReversed, 
  SwerveSetUp.frontRDriveKP, SwerveSetUp.frontRDriveKI, SwerveSetUp.frontRDriveKD, SwerveSetUp.fRTurnMotor, SwerveSetUp.isFRTurnMotorReversed, 
  SwerveSetUp.frontRTurnKP, SwerveSetUp.frontRTurnKI, SwerveSetUp.frontRTurnKD, SwerveSetUp.fRChassisAngleOffset);
  //Back Left
  private SwerveModule backLeft = new SwerveModule(SwerveSetUp.bLDriveMotor, SwerveSetUp.isBLDriveMotorReversed, 
  SwerveSetUp.backLDriveKP, SwerveSetUp.backLDriveKI, SwerveSetUp.backLDriveKD, SwerveSetUp.bLTurnMotor, SwerveSetUp.isBLTurnMotorReversed, 
  SwerveSetUp.backLTurnKP, SwerveSetUp.backLTurnKI, SwerveSetUp.backLTurnKD, SwerveSetUp.bLChassisAngleOffset);
  //Back Right
  private SwerveModule backRight = new SwerveModule(SwerveSetUp.bRDriveMotor, SwerveSetUp.isBRDriveMotorReversed, 
  SwerveSetUp.backRDriveKP, SwerveSetUp.backRDriveKI, SwerveSetUp.backRDriveKD, SwerveSetUp.bRTurnMotor, SwerveSetUp.isBRTurnMotorReversed, 
  SwerveSetUp.backRTurnKP, SwerveSetUp.backRTurnKI, SwerveSetUp.backRTurnKD, SwerveSetUp.bRChassisAngleOffset);

  private SwerveDriveOdometry odometry;
  private WPI_Pigeon2 gyro = new WPI_Pigeon2(1);
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    zeroHeading();
    odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, new Rotation2d(), getModulesPosition());
  }
  //Gets the robots heading based on gyro
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360));
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  //Resets the gyros heading
  public void zeroHeading(){
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      getHeading(), getModulesPosition());
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("FrontL Angle: ", frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("FrontR Angle: ", frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("BackL Angle: ", backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("BackR Angle: ", backRight.getState().angle.getDegrees());
  }
  //Sets the swerve module states based on the parameters values mainly used for joystick drive
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );
    setModuleStates(swerveModuleStates);
  }
  //Sets the modules states and assures that the speed of the modules never goes beyond the maxDriveSpeed
  public void setModuleStates(SwerveModuleState[] desiredState){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, SwerveConstants.maxDriveSpeed);
    frontLeft.setDesiredState(desiredState[0]);
    frontRight.setDesiredState(desiredState[1]);
    backLeft.setDesiredState(desiredState[2]);
    backRight.setDesiredState(desiredState[3]);
  }
  //Gets the modules positions 
  public SwerveModulePosition[] getModulesPosition(){
    SwerveModulePosition[] position = new SwerveModulePosition[4];
    position[0] = frontLeft.getPosition();
    position[1] = frontRight.getPosition();
    position[2] = backLeft.getPosition();
    position[3] = backRight.getPosition();
    return position;
  }
  //Resets odometry ot the specified pose
  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getHeading(), getModulesPosition(), pose);
  }
  public void resetEncoders(){
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
    backLeft.resetEncoder();
    backRight.resetEncoder();
  }
  //Used for field centric drive. If gyro is inversed it will go the opposite way of the way it was positioned
  public Rotation2d getYaw() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return (SwerveSetUp.invertedGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
  }
}
