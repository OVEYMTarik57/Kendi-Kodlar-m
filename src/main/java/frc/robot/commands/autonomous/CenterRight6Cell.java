/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SneakyTrajectory;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CenterRight6Cell extends SequentialCommandGroup {
  /**
   * Creates a new CenerRight6Cell.
   */
  public CenterRight6Cell(SneakyTrajectory s_trajectory, Shooter shooter, intake intake, 
  HopperSubsystem hopper, DriveSubsystem drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand()); 
    super(
      new RunShooter(shooter, 0.75).withTimeout(0.75),
      new RunHopper(hopper, 0.5).raceWith(new RunShooter(shooter,0.75)).withTimeout(2),
      s_trajectory.getRamsete(s_trajectory.CenterRight6Cell[0]).
      raceWith(new RunHopper(hopper, 0.5)).raceWith(new RunIntake(intake, 0.75))
      ,s_trajectory.getRamsete(s_trajectory.CenterRight6Cell[1]).andThen(() -> drive.arcadeDrive(0,0)),
      new RunShooter(shooter, 0.75).withTimeout(0.75),
      new RunHopper(hopper, 0.5).raceWith(new RunShooter(shooter, 0.75)).withTimeout(2));
    

    
  }
}
