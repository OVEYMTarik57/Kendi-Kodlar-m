
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */
  private final VictorSP frontLeftMotor = new VictorSP(Constants.DriveConstants.frontLeftMotorPin);
  private final VictorSP frontRightMotor = new VictorSP(Constants.DriveConstants.frontRightMotorPin);
  private final VictorSP rearLeftMotor = new VictorSP(Constants.DriveConstants.rearLeftMotorPin);
  private final VictorSP rearRightMotor = new VictorSP(Constants.DriveConstants.rearRightMotorPin);
  private final Encoder rightWheelEncoder = new Encoder(Constants.DriveConstants.rightWheelEncoder_A,DriveConstants.rightWheelEncoder_B,false,EncodingType.k4X);
  private final Encoder leftWheelEncoder = new Encoder(Constants.DriveConstants.leftWheelEncoder_A,DriveConstants.leftWheelEncoder_B,false,EncodingType.k4X);


  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, rearLeftMotor);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, rearRightMotor);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final DifferentialDriveOdometry m_odometry;

  public DriveSubsystem() {
    
    rightWheelEncoder.setDistancePerPulse((7.62*2*Math.PI)/2048.0);
    leftWheelEncoder.setDistancePerPulse((7.62*2*Math.PI)/2048.0);
    gyro.calibrate();
    
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    
      // Update the odometry in the periodic block
      m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftWheelEncoder.getDistance(),
                      rightWheelEncoder.getDistance());
    }
  
  

  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot, true); //fwd ileri geri, rot döndürme...
  }
//fwd=ileri geri, rot=sağ sol...
public double getRightWheelCm(){
  return rightWheelEncoder.getDistance();
}
public double getLeftWheelCm(){
  return leftWheelEncoder.getDistance();
}
  public double  readYawAngle()
  {
    return gyro.getAngle();
  }
   public double getHeading(){
     return Math.IEEEremainder(-1*gyro.getAngle(), 360);
   }
   

   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftWheelEncoder.getRate(), rightWheelEncoder.getRate());
  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(-rightVolts);
    m_drive.feed();
  }
}
