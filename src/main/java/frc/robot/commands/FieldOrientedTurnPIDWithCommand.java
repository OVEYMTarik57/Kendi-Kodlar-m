

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

//FieldOrientedTurnPID ile aynı şey sadece kütüphane hali...

public class FieldOrientedTurnPIDWithCommand extends PIDCommand {
  /**
   * Creates a new FieldOrientedTurnPIDWithCommand.
   */
  public FieldOrientedTurnPIDWithCommand(DriveSubsystem m_drive, double targetAngle) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD),
        // This should return the measurement
        () -> m_drive.getHeading(),
        // This should return the setpoint (can also be a constant)
        targetAngle,
        // This uses the output
        output -> {
          // Use the output here
          m_drive.arcadeDrive(0, output);
        });
 
        
        addRequirements(m_drive);
        getController().setTolerance(DriveConstants.turnAccuracy);
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
