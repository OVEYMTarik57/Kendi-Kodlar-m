/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RunArm extends CommandBase {
  /**
   * Creates a new RunArm.
   */
  private final ArmSubsystem m_arm;
  private final double m_speed;

  public RunArm(ArmSubsystem arm, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = arm;
    this.m_speed = speed;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.runArm(m_speed);
    System.out.println("Kol Açısı:"+m_arm.getArmAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_speed > 0) {
      return m_arm.getArmAngle() >=270;
    } else {
      return m_arm.getArmAngle() <=5;
      
    }
  }
}
