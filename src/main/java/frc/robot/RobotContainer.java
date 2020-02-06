/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.trajectoryconstraints.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public final XboxController m_pilot = new XboxController(Constants.k_Controler_port);
  public final Compressor compressor = new Compressor(0);
  


  //subSystems
  private final Drivetrain_Subsystem m_Drive = new Drivetrain_Subsystem();
  private final CANSparkVelPID_Subsystem m_velPIDL = new CANSparkVelPID_Subsystem( new CANSparkMax(25, MotorType.kBrushless));
  private final CANSparkVelPID_Subsystem m_velPIDR = new CANSparkVelPID_Subsystem(new CANSparkMax(20, MotorType.kBrushless));
  //private CANPIDController m_pidControllerRIGHT = new CANPIDController(Drivetrain_Subsystem.DRIVE_RIGHT_1); 
  //private CANPIDController m_pidControllerLEFT = new CANPIDController(Drivetrain_Subsystem.DRIVE_LEFT_1);
  //commands
  private final DefaultDrive m_DefaultDrive = new DefaultDrive(m_Drive, m_pilot);
  private final VelPID vPid = new VelPID(m_velPIDL, m_velPIDR);
  //Trajectory contraints
  private final DifferentialDriveKinematicsConstraint DifferentialDriveKinematicsConstraint 
     = new DifferentialDriveKinematicsConstraint(Constants.kDriveKinematics,Constants.kDifferentialDriveKinematicsConstraint);
  private final CentripetalAccelerationConstraint CentripetalAccelerationConstraint
   = new CentripetalAccelerationConstraint(Constants.kmaxCentripetalAccelerationMetersPerSecondSq);

  /**
   * gitThe container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    compressor.setClosedLoopControl(true);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger DriveEnabler_Button = new JoystickButton(m_pilot, XboxController.Button.kStart.value)
    .whenPressed(m_DefaultDrive);

    Trigger DriveDisabler_Button = new JoystickButton(m_pilot, XboxController.Button.kB.value)
   .and(new JoystickButton(m_pilot, XboxController.Button.kY.value))
  .cancelWhenActive(m_DefaultDrive);

  Trigger DriveNMeters = new JoystickButton(m_pilot, XboxController.Button.kB.value)
  .and(new JoystickButton(m_pilot, XboxController.Button.kA.value))
  .toggleWhenActive(vPid);

    //Trigger DriveNMetre = new JoystickButton(m_pilot, buttonNumber)
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   m_Drive.invertRightMotors();
     var autoVoltageConstraint =
     new DifferentialDriveVoltageConstraint(
         new SimpleMotorFeedforward(Constants.ksVolts,
                                    Constants.kvVoltSecondsPerMeter,
                                    Constants.kaVoltSecondsSquaredPerMeter),
         Constants.kDriveKinematics,
         10);

 // Create config for trajectory
 TrajectoryConfig config =
     new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                          Constants.kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(Constants.kDriveKinematics)
         // Apply the voltage constraint
         .addConstraint(autoVoltageConstraint)
         .addConstraint(DifferentialDriveKinematicsConstraint)
         .addConstraint(CentripetalAccelerationConstraint);

config.setKinematics(Constants.kDriveKinematics);

 // An example trajectory to follow.  All units in meters.
 Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
     // Start at the origin facing the +X direction
     new Pose2d(0, 0, new Rotation2d(0)),
     // Pass through these two interior waypoints, making an 's' curve path
     List.of(
      // new Translation2d(1,0)
     ),
     // End 3 meters straight ahead of where we started, facing forward
     new Pose2d(3, 3, new Rotation2d(0)),
     //Repeated occilation between left and right motors in a diagonal direction occurs between these points     
     config
     );

  
 RamseteCommand ramseteCommand = new RamseteCommand(
     exampleTrajectory,
     m_Drive::getPose,
     new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
     new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
     Constants.kDriveKinematics,
     m_Drive::getWheelSpeeds,
     new PIDController(Constants.kP, Constants.kI, Constants.kD),// LEFT PID
     new PIDController(Constants.kP, Constants.kI, Constants.kD),// rIGHT PID
     // RamseteCommand passes volts to the callback
     m_Drive::tankDriveVolts,
     m_Drive
 );
  return ramseteCommand.andThen(() -> m_Drive.tankDriveVolts(0, 0));
  }

 public void resetAuto(){
  m_Drive.invertRightMotors();
  m_Drive.resetOdometry();
 }
 public void resetTele(){
  m_Drive.resetOdometry();
  m_Drive.undoRightMotors();
 }
  

//public void setSafety(){
//    m_Drive.setSafety();
//}
//public void setNoSafety(){
//  m_Drive.setNoSafety();
//}

}
//https://www.youtube.com/watch?v=wqJ4tY0u6IQ//