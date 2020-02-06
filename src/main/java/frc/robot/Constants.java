/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    public static final int k_Controler_port = 0;
   
    //Trajectory constants//
    public static final double ksVolts = 0.109 ;
    public static final double kvVoltSecondsPerMeter = 6.78;
    public static final double kaVoltSecondsSquaredPerMeter = 0.719;
    public static final double kPDriveVel = 0.3; //on loop type position
    //public static final double kPDriveVel = 1.87; //on loop type velocity
    public static final double kP=46.8, kI=0, kD=18.5;
    
    public static final double kTrackwidthMeters = 0.6523019180914628;
    public static DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 0.3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kmaxCentripetalAccelerationMetersPerSecondSq = 0.03;
    public static final double  kDifferentialDriveKinematicsConstraint = 0.3;

    public static final boolean kGyroReversed = true;

    public static final double kEncoderDistanceConversionFactor =((double)1.0/53.761974);
    public static final double kEncoderVelocityConversionFactor =((double)1.0/(60 * 53.761974));
    //Trajectory constants//


    public static CANSparkMax DRIVE_LEFT_1 = new CANSparkMax(25, MotorType.kBrushless);
    public static CANSparkMax DRIVE_LEFT_2 = new CANSparkMax(24, MotorType.kBrushless);
    public static CANSparkMax DRIVE_LEFT_3 = new CANSparkMax(23, MotorType.kBrushless);
    public static CANSparkMax DRIVE_RIGHT_1 = new CANSparkMax(20, MotorType.kBrushless);
    public static CANSparkMax DRIVE_RIGHT_2 = new CANSparkMax(21, MotorType.kBrushless);
    public static CANSparkMax DRIVE_RIGHT_3 = new CANSparkMax(22, MotorType.kBrushless);
}
