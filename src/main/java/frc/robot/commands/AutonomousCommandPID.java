/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousCommandPID extends PIDCommand {
  /**
   * Creates a new AutonomousCommandPID.
   */
  public AutonomousCommandPID(DriveSubsystem m_drive, double targetDistance) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.distanceP, DriveConstants.distanceI, DriveConstants.distanceD),
        // This should return the measurement
        () -> (m_drive.getRightWheelCm()+ m_drive.getLeftWheelCm())/2,
        // This should return the setpoint (can also be a constant)
        targetDistance,
        // This uses the output
        output -> {
          // Use the output here
          m_drive.arcadeDrive(output, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(m_drive);
    getController().setTolerance(DriveConstants.distanceAccuracy);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
