/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class IntakeConstants{
public static final int rightIntakeMotorPort =0;
public static final int leftIntakeMotorPort  =1;
public static final int intakeSolenoidForwardPort =0;
public static final int intakeSolenoidBackwardPort =1;

}
    public final class JoystickConstants{
     public static final int driverControllerPort = 0;

    }

    public final class ShooterConstants{

        public static final int rightShooterMotorPort = 2;
        public static final int leftShooterMotorPort =  3;


        
    }

    public static class DriveConstants{
        public static final int frontLeftMotorPin = 4; 
        public static final int frontRightMotorPin =5;
        public static final int rearLeftMotorPin =6;
        public static final int rearRightMotorPin =7;
        public static final int rightWheelEncoder_A =0;
        public static final int rightWheelEncoder_B =1;
        public static final int leftWheelEncoder_A=2;
        public static final int leftWheelEncoder_B=3;

        public static final double turnP = 1.0;
        public static final double turnI = 0.0;
        public static final double turnD = 0.0;

        public static final double distanceP = 1.0;
        public static final double distanceI = 0.0;
        public static final double distanceD = 0.0; //Örneğin 1 verdik. hata yı 1 ile çarpıyoruz. hata azaldığı için otomatik olarak yavaşlıyor.

        public static final double turnAccuracy = 2.0;
        public static final double distanceAccuracy = 3.0;
    
       //trajectory
        public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
    
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    

        public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    

         // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    
    }   

    public final class ClimbConstants{
        public  static final int solenoidForwardPin =2;
        public  static final int solenoidBackwardPin =3;
        public static final int compressorPin=0;
        //compresör her zaman 0 dır.

    }
    public final class LiftConstants{
        public static final int rightLiftMotorPort = 8;
        public static final int leftLiftMotorPort  = 9; 
        public static final int topLimitSwitchPort = 5;
        public static final int bottomLimitSwitchPort =6;
        public static final int liftEncoder_A =7;
        public static final int liftEncoder_B =8;

    }
    public final class ArmConstants{
        public static final int centerArmMotorPort = 10;
        public static final int armPotPort = 0;
        public static final int ArmEncoder_A = 4;
        public static final int ArmEncoder_B = 5;

        
    }

    public final class HopperConstants{
        public static final int upHopperMotorPort =0 ;
        public static final int downHopperMotorPort =1 ;

    }

}