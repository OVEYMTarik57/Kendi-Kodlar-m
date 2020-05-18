/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.Left8Cell;
import frc.robot.commands.autonomous.Middle3Cell;
import frc.robot.commands.autonomous.Right3Cell;
import frc.robot.commands.autonomous.Right6Cell;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {
    public Trajectory[] CenterRight6Cell = new Trajectory[2]; //yazdığımız otonomları burada tanımlıyoruz.
    public Trajectory[] Left3Cell = new Trajectory[1];
    public Trajectory[] Middle3Cell = new Trajectory[1];
    public Trajectory[] Right6Cell = new Trajectory[2];
    public Trajectory[] Left8Cell = new Trajectory[4];
    public Trajectory[] Right3Cell = new Trajectory[1];
    private DriveSubsystem m_drive;


    

    public SneakyTrajectory(DriveSubsystem drive){
       m_drive =drive;
        
       var autoVoltageConstraint =
      
       new DifferentialDriveVoltageConstraint(
         new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
         DriveConstants.kDriveKinematics,
         10);
 
         TrajectoryConfig configForward =
         new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                              DriveConstants.kMaxAccelerationMetersPerSecondSquared)
             // Add kinematics to ensure max speed is actually obeyed
             .setKinematics(DriveConstants.kDriveKinematics)
             // Apply the voltage constraint
             .addConstraint(autoVoltageConstraint);
 
            
             TrajectoryConfig configBackward =
           new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                              DriveConstants.kMaxAccelerationMetersPerSecondSquared)
             // Add kinematics to ensure max speed is actually obeyed
             .setKinematics(DriveConstants.kDriveKinematics)
             // Apply the voltage constraint
             .addConstraint(autoVoltageConstraint);
 
             configBackward.setReversed(true);

             CenterRight6Cell[0] = TrajectoryGenerator.generateTrajectory( //ilk aşama olduğu için 0 yazdık.
                 List.of(
                     new Pose2d(12.80, 5.79, new Rotation2d(0)), //rotation açı fakat radyan biriminden.
                     new Pose2d(9.75, 7.54, new Rotation2d(0)),
                     new Pose2d(7.92, 7.54, new Rotation2d(0))),
                     configBackward);

             CenterRight6Cell[1] = TrajectoryGenerator.generateTrajectory(  // ikinci aşama olduğu için 1 yazık.
                    List.of(
                    new Pose2d(7.92, 7.54, new Rotation2d(0)), //rotation açı fakat radyan biriminden.
                    new Pose2d(10.97, 5.79, new Rotation2d(0)),
                    new Pose2d(12.80, 5.79, new Rotation2d(0))),
                     configForward);    
    

                     Middle3Cell[0] = TrajectoryGenerator.generateTrajectory(  
                     List.of(
                     new Pose2d(3.09868824, 2.42023392, new Rotation2d(3.141592654)), //rotation açı fakat radyan biriminden.
                     new Pose2d(1.40448792, 2.42023392, new Rotation2d(3.141592654))),
                    configForward);    



                    Left3Cell[0] = TrajectoryGenerator.generateTrajectory(  
                     List.of(
                     new Pose2d(3.11380632, 4.41694824, new Rotation2d(-2.539305709)), //rotation açı fakat radyan biriminden.
                     new Pose2d(1.55576016, 3.34295496, new Rotation2d(-2.539305709))),
                    configForward);    
            

                    
                   
                   
                    Right6Cell[0] = TrajectoryGenerator.generateTrajectory(  
                     List.of(
                     new Pose2d(12.80992104, 5.80860408, new Rotation2d(3.141592654)), //rotation açı fakat radyan biriminden.
                     new Pose2d(10.767822, 7.53301008, new Rotation2d(3.141592654)),
                     new Pose2d(8.07531024, 7.53301008, new Rotation2d(3.141592654))),
                    configBackward);  


                    Right6Cell[1] = TrajectoryGenerator.generateTrajectory(  
                     List.of(
                     new Pose2d(8.07531024, 7.53301008, new Rotation2d(0)), //rotation açı fakat radyan biriminden.
                     new Pose2d(10.767822, 7.53301008, new Rotation2d(0)),
                     new Pose2d(12.80992104, 5.80860408, new Rotation2d(0))),
                    configForward);  






                    Left8Cell[0] = TrajectoryGenerator.generateTrajectory(  
                     List.of(
                     new Pose2d(12.7796544, 0.80171544, new Rotation2d(3.141592654)), //rotation açı fakat radyan biriminden.
                     new Pose2d(9.9963732, 0.80171544, new Rotation2d(3.141592654))),
                    configBackward);  


                    Left8Cell[1] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(9.9963732, 0.80171544, new Rotation2d(0)), //rotation açı fakat radyan biriminden.
                      new Pose2d(12.91580856, 4.12955232, new Rotation2d(0.499347426))),
                     configForward);  


                     Left8Cell[3] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(12.91580856, 4.12955232, new Rotation2d(0.499347426)), //rotation açı fakat radyan biriminden.
                      new Pose2d(11.356848, 7.290816, new Rotation2d(2.413167273)),
                      new Pose2d(8.104632, 7.531608, new Rotation2d(-3.114424858))),
                     configBackward);  

                     
                     Left8Cell[3] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(8.104632, 7.531608, new Rotation2d(-3.114424858)), //rotation açı fakat radyan biriminden.
                      new Pose2d(10.814304, 7.427976, new Rotation2d(-0.205395582)),
                      new Pose2d(12.780264, 5.779008, new Rotation2d(0))),
                     configForward);  





                     Right3Cell[0] = TrajectoryGenerator.generateTrajectory(  
                      List.of(
                      new Pose2d(12.80992104, 7.77505176, new Rotation2d(-0.550386089)), //rotation açı fakat radyan biriminden.
                      new Pose2d(14.24693112, 7.06410576, new Rotation2d(-0.550386089))),
                     configForward);  



                   

     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
                    }

     public RamseteCommand getRamsete(Trajectory trajectory){
        
        return new RamseteCommand(
        trajectory,
      m_drive::getPose,
      new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      m_drive::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_drive::tankDriveVolts,
      m_drive
  );
        

     }
    
    }