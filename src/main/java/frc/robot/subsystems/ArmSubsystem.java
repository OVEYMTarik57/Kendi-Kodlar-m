/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LiftConstants;

public class ArmSubsystem extends SubsystemBase {
  /**
   * Creates a new ReostaSubsystem.
   */
    private final VictorSP centerArmMotor = new VictorSP(ArmConstants.centerArmMotorPort);
    private final AnalogPotentiometer pot = new AnalogPotentiometer(ArmConstants.armPotPort, 270, 5);
    private final Encoder armEncoder = new Encoder(ArmConstants.ArmEncoder_A,ArmConstants.ArmEncoder_B,false,EncodingType.k4X);
    
    public ArmSubsystem() {
      armEncoder.setDistancePerPulse(3.0/2048.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runArm(double speed){
    centerArmMotor.set(speed);
  }
  public void stopArm(){
    centerArmMotor.set(0);
  }
    
    public double getAngle(){
    return pot.get();
  }
  public double getArmAngle(){
    return armEncoder.getDistance()+5;
  }
}
