// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//Kauli Labs Dependencies
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

//WPILIB Dependencies
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;

//Class containing all functions and variables pertaining to the SwerveDrive
public class SwerveDrive extends SubsystemBase 
{ 
  double maxVelocity; //The maximum linear speed of the robot in meters per second
  double maxAngularSpeed; //The maximum angular speed of the robot in radians per second
  double headingAdjustment = 0; //An adjustment to be applied to the gyro sensor if needed
  SwerveDriveKinematics kinematics; //A kinematics object used by the odometry object to determine wheel locations
  String moduleType; //The type of Swerve Module being utilized
  boolean debugMode = false; //Whether or not to enable debug features (DISABLE FOR COMPETITIONS)

  //Instantiate four Swerve Modules according to the class SwerveModule constructor
  private final SwerveModule frontLeft  = new SwerveModule(SwerveConstants.DRIVEFRONTLEFT, SwerveConstants.ROTATIONFRONTLEFT, SwerveConstants.ENCODERFRONTLEFT, 45); 
  private final SwerveModule frontRight =  new SwerveModule(SwerveConstants.DRIVEFRONTRIGHT, SwerveConstants.ROTATIONFRONTRIGHT, SwerveConstants.ENCODERFRONTRIGHT, -45); 
  private final SwerveModule backLeft   =  new SwerveModule(SwerveConstants.DRIVEBACKLEFT, SwerveConstants.ROTATIONBACKLEFT, SwerveConstants.ENCODERBACKLEFT, -45); 
  private final SwerveModule backRight  = new SwerveModule(SwerveConstants.DRIVEBACKRIGHT, SwerveConstants.ROTATIONBACKRIGHT, SwerveConstants.ENCODERBACKRIGHT, 45); 
  
  //The gyro used to determine the robot heading is a Kauli Labs NavX plugged into the MXP port on the roborio
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  //Odometry determines the robots position on the field
  private final SwerveDriveOdometry odometry; 

  //This switch is used as an external input to tell the SwerveDrive to normalize the Swerve Modules
  private DigitalInput normalizeSwitch = new DigitalInput(0);  
  //This switch is used as an external input to tell the SwerveDrive to reset the odometry
  private DigitalInput resetOdometry = new DigitalInput(1);

  /**
   * The constructor for the swerve drive
   * @param maxVelocity The desired max velocity of the robot in meters per second
   * @param maxAngularSpeed The desired max angular speed of the robot in radians per second 
   * @param moduleType Pass in "geared" for wcp gear swerve modules or "belted" for belted modules
   * @param kinematics A kinematics object containing the locations of each swerve module relative to robot center 
   */
  public SwerveDrive(double maxVelocity, double maxAngularSpeed, String moduleType, SwerveDriveKinematics kinematics) 
    {
      this.maxAngularSpeed = maxAngularSpeed; 
      this.maxVelocity = maxVelocity; 
      this.moduleType = moduleType; 
      this.kinematics = kinematics; 
      
      resetGyro(); 

      //Using the moduleType passed in to the constructor, set the appropriate settings for the modules used.
      frontLeft.setModuleSettings(moduleType);
      frontRight.setModuleSettings(moduleType);
      backLeft.setModuleSettings(moduleType);
      backRight.setModuleSettings(moduleType);  

      /*
      * Initialize the odometry (if this is done outside of the constructor it will pass garbage values 
      * for the distances of the Swerve Modules). 
      */
      odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions());
          
  }

  /**
   * The constructor for the swerve drive for pathplanner use
   * @param maxVelocity The desired max velocity of the robot in meters per second
   * @param maxAngularSpeed The desired max angular speed of the robot in radians per second 
   * @param moduleType Pass in "geared" for wcp gear swerve modules or "belted" for belted modules
   * @param kinematics A kinematics object containing the locations of each swerve module relative to robot center 
   * @param config PID constants and other settings for autonomous driving
   */
  public SwerveDrive(double maxVelocity, double maxAngularSpeed, String moduleType, SwerveDriveKinematics kinematics, HolonomicPathFollowerConfig config) 
    {
      this.maxAngularSpeed = maxAngularSpeed; 
      this.maxVelocity = maxVelocity; 
      this.moduleType = moduleType; 
      this.kinematics = kinematics; 
      
      resetGyro(); 

      //Using the moduleType passed in to the constructor, set the appropriate settings for the modules used.
      frontLeft.setModuleSettings(moduleType);
      frontRight.setModuleSettings(moduleType);
      backLeft.setModuleSettings(moduleType);
      backRight.setModuleSettings(moduleType);  

      /*
      * Initialize the odometry (if this is done outside of the constructor it will pass garbage values 
      * for the distances of the Swerve Modules). 
      */
      odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions());
      
      //A pathplanner AutoBuilder object for autonomous driving
      AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getChassisSpeeds, this::driveRobotOriented, config, this);
  }

  //Reset the gyro using NavX reset function and apply a user-specified adjustment to the angle
  public void resetGyro()
  {
    if (gyro != null)
    {
      gyro.reset();
      gyro.setAngleAdjustment(headingAdjustment);
    }
  }

  @Override 
  public void periodic() 
    {

      //Periodically update the swerve odometry
      updateOdometry(); 

      //Normalize the modules when the normalize switch is pressed
      if(!normalizeSwitch.get())
        {
          normalizeModules();
        }
      //Reset the odometry readings when reset odometry switch is pressed
      if(!resetOdometry.get())
        {
          zeroPose();
        }

      if(debugMode)
        {
          SmartDashboard.putNumber("Raw Gyro Angle", gyro.getAngle());
        
          SmartDashboard.putNumber("SwerveModuleAngle/frontLeft", frontLeft.getAngle()); 
          SmartDashboard.putNumber("SwerveModuleAngle/frontRight", frontRight.getAngle()); 
          SmartDashboard.putNumber("SwerveModuleAngle/backLeft", backLeft.getAngle()); 
          SmartDashboard.putNumber("SwerveModuleAngle/backRight", backRight.getAngle()); 
  
          SmartDashboard.putNumber("SwerveModuleDistanceFL", frontLeft.getSwerveModulePosition().distanceMeters);
          SmartDashboard.putNumber("SwerveModuleDistanceFR", frontRight.getSwerveModulePosition().distanceMeters);
          SmartDashboard.putNumber("SwerveModuleDistanceBL", backLeft.getSwerveModulePosition().distanceMeters);
          SmartDashboard.putNumber("SwerveModuleDistanceBR", backRight.getSwerveModulePosition().distanceMeters);

          SmartDashboard.putNumber("SwerveModuleVelocityFL", frontLeft.getSwerveModuleState().speedMetersPerSecond);
          SmartDashboard.putNumber("SwerveModuleVelocityFR", frontRight.getSwerveModuleState().speedMetersPerSecond);
          SmartDashboard.putNumber("SwerveModuleVelocityBL", backLeft.getSwerveModuleState().speedMetersPerSecond);
          SmartDashboard.putNumber("SwerveModuleVelocityBR", backRight.getSwerveModuleState().speedMetersPerSecond);
  
          SmartDashboard.putNumber("SwerveModuleAngleFL", frontLeft.getSwerveModulePosition().angle.getDegrees());
          SmartDashboard.putNumber("SwerveModuleAngleFR", frontRight.getSwerveModulePosition().angle.getDegrees());
          SmartDashboard.putNumber("SwerveModuleAngleBL", backLeft.getSwerveModulePosition().angle.getDegrees());
          SmartDashboard.putNumber("SwerveModuleAngleBR", backRight.getSwerveModulePosition().angle.getDegrees());
  
          SmartDashboard.putNumber("SwerveDrive/Pose/X", odometry.getPoseMeters().getX());
          SmartDashboard.putNumber("SwerveDrive/Pose/Y", odometry.getPoseMeters().getY());
          SmartDashboard.putNumber("SwerveDrive/Pose/Z", odometry.getPoseMeters().getRotation().getDegrees());
        }
    }

  //Stop all swerve modules by setting their speeds to 0
  public void stop()
    {
      drive(0.0, 0.0, 0.0, false);
    } 

  public void driveRobotOriented(ChassisSpeeds robotRelativeSpeeds)
  {

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    setModuleStates(targetStates);
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented)
    {

      SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
        (fieldOriented && gyro != null)
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * maxVelocity, 
                                              ySpeed * maxVelocity, 
                                              rotationSpeed * maxAngularSpeed, 
                                              Rotation2d.fromDegrees(getGyroAngle())) 
        : new ChassisSpeeds(xSpeed, 
                            ySpeed, 
                            rotationSpeed)); 
      //This function should limit our speed to the value we set (maxVelocity)
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity); 
      setModuleStates(swerveModuleStates);
    }

  public void updateOdometry()
    { 
       odometry.update( Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions());
    }


  public void setStartLocation(Pose2d pose) 
    {
      gyro.setAngleAdjustment(pose.getRotation().getDegrees() - getGyroAngle());
      odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions(), getPose());
    }  

  public Pose2d getPose()
    { 
      Pose2d pose = odometry.getPoseMeters();
      return new Pose2d( pose.getX(), pose.getY(), pose.getRotation()); 
    } 

  public void setModuleStates(SwerveModuleState[] states)
    { 
      frontLeft.setDesiredState(states[0], parkingBrake);
      frontRight.setDesiredState(states[1], parkingBrake); 
      backLeft.setDesiredState(states[2], parkingBrake);
      backRight.setDesiredState(states[3], parkingBrake);
    }

  public SwerveModulePosition[] getSwerveModulePositions() 
    {
      return new SwerveModulePosition[] 
        {
          frontLeft.getSwerveModulePosition(),
          frontRight.getSwerveModulePosition(),
          backLeft.getSwerveModulePosition(),
          backRight.getSwerveModulePosition()
        };
    }

  public SwerveModuleState[] getSwerveModuleStates()
  {
    return new SwerveModuleState[] 
        {
          frontLeft.getSwerveModuleState(),
          frontRight.getSwerveModuleState(),
          backLeft.getSwerveModuleState(),
          backRight.getSwerveModuleState()
        };
  }

  public void zeroPose()
    {
      System.out.println("resetting pose");
      odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions(), new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(getGyroAngle())));
    }

  public void resetPose(Pose2d pose)
  {
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }
  /*
   * Get the angle of the gyro. This angle is negated to reflect the fact that 
   * the code is expecting counterclockwise to be positive (critical for odometry).
   */
  private double getGyroAngle()
    {
      if(gyro == null)
        {
          return 0.0;
        }
      return -gyro.getAngle();
    } 
 

  public void brakeMode()
    { 
      frontLeft.brakeMode();
      backLeft.brakeMode(); 
      frontRight.brakeMode(); 
      backRight.brakeMode();
    }

  private boolean parkingBrake = false; 

  public void parkingBrake(boolean enabled)
    { 
      parkingBrake = enabled; 
    }
 
  private void normalizeModules()
    {
      frontLeft.normalizeModule();
      backLeft.normalizeModule(); 
      frontRight.normalizeModule(); 
      backRight.normalizeModule();
    }

 /**
  * Enable debug settings like extra SmartDashboard data. Do not use this in competitions
  * because it consumes too much RAM!
  */
  public void enableDebugMode()
   {
     System.out.println("DEBUG MODE ENABLED");
     debugMode = true;
   }

  /**
   * Set an offset to the robots heading in degrees. This is useful if the robots physical forward
   * direction is not the direction of forward travel desired. This value will update when the gyro is 
   * reset.
   * @param adjustmentDeg an adjustment, in degrees, of the robots heading
   */
  public void setHeadingAdjustment(double adjustmentDeg)
    {
      headingAdjustment = adjustmentDeg; 
      System.out.printf("Set an angle adjustment of %.2f degrees", adjustmentDeg);
    }
}
