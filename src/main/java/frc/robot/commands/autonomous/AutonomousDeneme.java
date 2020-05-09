/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intake;
import frc.robot.commands.*;
import frc.robot.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousDeneme extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousDeneme.
   */
  public AutonomousDeneme(LiftSubsystem m_lift, ArmSubsystem m_arm, DriveSubsystem m_drive, Shooter m_shooter,
      HopperSubsystem m_hopper, intake m_intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(new RunShooter(m_shooter, 1).withTimeout(0.75),
        new RunHopper(m_hopper, 0.8).raceWith(new RunShooter(m_shooter, 0.8).withTimeout(2.75)),
        new AutonomousDrive(m_drive, -0.8, -300).raceWith(new RunHopper(m_hopper, 0.8)
            .raceWith(new RunIntake(m_intake, 0.8).raceWith(new RunShooter(m_shooter, -0.3)))),
        new AutonomousDrive(m_drive, 0.8, 300), new RunShooter(m_shooter, 0.75),
        new RunHopper(m_hopper, 0.8).raceWith(new RunShooter(m_shooter, 0.8).withTimeout(2.5)));
  }
}
