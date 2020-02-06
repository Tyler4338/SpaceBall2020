package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Drivetrain_Subsystem extends SubsystemBase {
    //Encoders//
    private CANEncoder m_EncoderRight;
    private CANEncoder m_EncoderLeft;
    private final DifferentialDriveOdometry m_odometry;
    //Motor Controlers// 
    public static CANSparkMax DRIVE_LEFT_1 = new CANSparkMax(25, MotorType.kBrushless);//changed
    private CANSparkMax DRIVE_LEFT_2 = new CANSparkMax(24, MotorType.kBrushless);
    private CANSparkMax DRIVE_LEFT_3 = new CANSparkMax(23, MotorType.kBrushless);
    public static CANSparkMax DRIVE_RIGHT_1 = new CANSparkMax(20, MotorType.kBrushless); //changed
    private CANSparkMax DRIVE_RIGHT_2 = new CANSparkMax(21, MotorType.kBrushless);
    private CANSparkMax DRIVE_RIGHT_3 = new CANSparkMax(22, MotorType.kBrushless);
    //

    private final Gyro m_gyro;
    
    //Drivetrain
    private DifferentialDrive drive ; 

    public Drivetrain_Subsystem(){

    

        DRIVE_LEFT_1.restoreFactoryDefaults();
        DRIVE_LEFT_2.restoreFactoryDefaults();
        DRIVE_LEFT_3.restoreFactoryDefaults();
    
        DRIVE_LEFT_2.follow(DRIVE_LEFT_1);
        DRIVE_LEFT_3.follow(DRIVE_LEFT_1);

        DRIVE_RIGHT_1.restoreFactoryDefaults();
        DRIVE_RIGHT_2.restoreFactoryDefaults();
        DRIVE_RIGHT_3.restoreFactoryDefaults();
    
        DRIVE_RIGHT_2.follow(DRIVE_RIGHT_1);
        DRIVE_RIGHT_3.follow(DRIVE_RIGHT_1);

        DRIVE_LEFT_1.setInverted(false);
        DRIVE_LEFT_2.setInverted(false);
        DRIVE_LEFT_3.setInverted(false);
        DRIVE_RIGHT_1.setInverted(false);   
        DRIVE_RIGHT_2.setInverted(false);
        DRIVE_RIGHT_3.setInverted(false);

        drive = new DifferentialDrive(DRIVE_LEFT_1, DRIVE_RIGHT_1);
        m_gyro = new ADXRS450_Gyro();
        m_gyro.reset();
        m_EncoderLeft = DRIVE_LEFT_1.getEncoder();
        m_EncoderRight = DRIVE_RIGHT_1.getEncoder();
        //
        m_EncoderLeft.setPositionConversionFactor(Constants.kEncoderDistanceConversionFactor);
        m_EncoderRight.setPositionConversionFactor(Constants.kEncoderDistanceConversionFactor);
        //
        m_EncoderLeft.setVelocityConversionFactor(Constants.kEncoderVelocityConversionFactor);
        m_EncoderRight.setVelocityConversionFactor(Constants.kEncoderVelocityConversionFactor);
        //

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }
    public void periodic(){
        //if(m_gyro.getAngle() > 360){
        //    m_gyro.reset();
        //}
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_EncoderLeft.getPosition(), 
        m_EncoderRight.getPosition());
        
        SmartDashboard.putNumber("endoerval", getHeading());
        SmartDashboard.putNumber("d", m_gyro.getAngle());
      //  SmartDashboard.putNumber("odometry", m_odometry.getPoseMeters().getTranslation());
    
    
    }
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_EncoderLeft.getVelocityConversionFactor(),m_EncoderRight.getVelocityConversionFactor()); // assuming Encoder.getrate() is the same as .getVelocityConversionFactor()
      }
    public void resetOdometry() {
        resetEncoders();

        //m_gyro.reset(); //Check
        m_odometry.resetPosition(new Pose2d(0,0, new Rotation2d(0)), Rotation2d.fromDegrees(getHeading()));
    }
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        DRIVE_LEFT_1.setVoltage(leftVolts);
        //DRIVE_LEFT_2.setVoltage(leftVolts);
        //DRIVE_LEFT_3.setVoltage(leftVolts);
        DRIVE_RIGHT_1.setVoltage(rightVolts);
        //DRIVE_RIGHT_2.setVoltage(rightVolts);
        //DRIVE_RIGHT_3.setVoltage(rightVolts);// This came inversed
        drive.feed();
      }
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);//gyro is inversed
     // return -m_gyro.getAngle();
    }
    public double getTurnRate() {
        return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
      }
    /**
    * Drive method
    */
    public void arcadeDrive (XboxController pilot){
          drive.arcadeDrive(-pilot.getY(Hand.kLeft),-pilot.getX(Hand.kRight),true);
          drive.feed();
    }

    public void resetEncoders() {
        m_EncoderRight.setPosition(0);
        m_EncoderLeft.setPosition(0);
        m_gyro.reset();
      }
    
    public void invertRightMotors(){  //this is a temporary fix 
        DRIVE_LEFT_1.setInverted(false);
        DRIVE_LEFT_2.setInverted(false);
        DRIVE_LEFT_3.setInverted(false);
        DRIVE_RIGHT_1.setInverted(true);   
        DRIVE_RIGHT_2.setInverted(true);
        DRIVE_RIGHT_3.setInverted(true);// invalid pid from characterization tool
    }
    public void undoRightMotors(){  //this is a temporary fix 
        DRIVE_LEFT_1.setInverted(false);
        DRIVE_LEFT_2.setInverted(false);
        DRIVE_LEFT_3.setInverted(false);
        DRIVE_RIGHT_1.setInverted(false);   
        DRIVE_RIGHT_2.setInverted(false);
        DRIVE_RIGHT_3.setInverted(false);// invalid pid from characterization tool
    }

    public void setSafety(){
        drive.setSafetyEnabled(true);
    }
    public void setNoSafety(){
        drive.setSafetyEnabled(false);
    }
}