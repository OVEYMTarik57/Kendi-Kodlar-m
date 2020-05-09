/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {
    public Trajectory centerRight6cell[];
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
 
             configReversed.setReversed(true);
    }

    
}
