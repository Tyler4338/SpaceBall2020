package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DefaultDrive extends CommandBase {
    private final Drivetrain_Subsystem m_drive;
    private final XboxController m_pilot;
    
  
    //CheckThis// NOT IN USE
    public DefaultDrive(Drivetrain_Subsystem subsystem,  XboxController pilot) {
      m_drive = subsystem;
      m_pilot = pilot;
   
      addRequirements(m_drive);

      
    }

    @Override
    public void execute() {
        

        m_drive.arcadeDrive(m_pilot);
    

        
     
    }
  }