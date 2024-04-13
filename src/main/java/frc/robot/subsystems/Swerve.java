// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoBuilderConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.CanIds;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;

public class Swerve extends SubsystemBase {
  public Swerve(){
    configureHolonomic();
    SmartDashboard.putData("Field", Field);
  }
  public void configureHolonomic(){
        AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry, 
        this::getChassisSpeeds, 
        this::setChassisSpeed,
        AutoBuilderConstants.hPathConf, 
        () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this);
  }
  
  // Create MAXSwerveModules
  public final AHRS Gyro = new AHRS(Port.kMXP);
  private Field2d Field = new Field2d();
    
  private final SwerveKit FrontLeft = new SwerveKit(
      CanIds.kFrontLeftDrivingCanId,
      CanIds.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveKit FrontRight = new SwerveKit(
      CanIds.kFrontRightDrivingCanId,
      CanIds.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveKit RearLeft = new SwerveKit(
      CanIds.kRearLeftDrivingCanId,
      CanIds.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveKit RearRight = new SwerveKit(
      CanIds.kRearRightDrivingCanId,
      CanIds.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor

  // Slew rate filter variables for controlling lateral acceleration
  private double CurrentRotation = 0.0;
  private double CurrentTranslationDir = 0.0;
  private double CurrentTranslationMag = 0.0;

  private SlewRateLimiter MagLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter RotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double PreviousTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry Odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(Gyro.getAngle()),
      new SwerveModulePosition[] {
          FrontLeft.getPosition(),
          FrontRight.getPosition(),
          RearLeft.getPosition(),
          RearRight.getPosition()
    });
  
  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics
                , Gyro.getRotation2d()
                , new SwerveModulePosition[]{
                    FrontLeft.getPosition(),
                    FrontRight.getPosition(),
                    RearLeft.getPosition(),
                    RearRight.getPosition()
                }, 
                new Pose2d()
    );
    
    NetworkTable SwerveTable = NetworkTableInstance.getDefault().getTable("SwerveTable");
    StructArrayPublisher<Pose2d> posePublisher =  SwerveTable.getStructArrayTopic("Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> odometryPublisher =  SwerveTable.getStructTopic("Odometry", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> statePublisher =  SwerveTable.getStructArrayTopic("Sweve Module State", SwerveModuleState.struct).publish();
 

    public void updatePoseEstimator(){
        poseEstimator.update(
            Gyro.getRotation2d(),
            new SwerveModulePosition[]{
                FrontLeft.getPosition(),
                FrontRight.getPosition(),
                RearLeft.getPosition(),
                RearRight.getPosition()
            });

        Field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

  /** Creates a new DriveSubsystem. */
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    
    Odometry.update(
        Gyro.getRotation2d(),
        new SwerveModulePosition[] {
            FrontLeft.getPosition(),
            FrontRight.getPosition(),
            RearLeft.getPosition(),
            RearRight.getPosition()
    });
      posePublisher.set(
        new Pose2d[]{
            Odometry.getPoseMeters()
        }
      );
      SmartDashboard.putNumber("Swerve Angle", Gyro.getAngle());
      updatePoseEstimator();
      odometryPublisher.set(Odometry.getPoseMeters());
      statePublisher.set(getSweveStates());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }

  public SwerveModuleState[] getSweveStates(){
    return new SwerveModuleState[]{
        FrontLeft.getState(),
        FrontRight.getState(),
        RearLeft.getState(),
        RearRight.getState()
    };
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    Odometry.resetPosition(
        Rotation2d.fromDegrees(Gyro.getAngle()),
        new SwerveModulePosition[] {
            FrontLeft.getPosition(),
            FrontRight.getPosition(),
            RearLeft.getPosition(),
            RearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
 //  !!!!! DRIVING COMMAND !!!!!    
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double InputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double InputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double DirectionSlewRate;
      if (CurrentTranslationMag != 0.0) {
        DirectionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / CurrentTranslationMag);
      } else {
        DirectionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double CurrentTime = WPIUtilJNI.now() * 1e-6;
      double ElapsedTime = CurrentTime - PreviousTime;
      double AngleDif = SwerveUtils.AngleDifference(InputTranslationDir, CurrentTranslationDir);
      if (AngleDif < 0.45*Math.PI) {
        CurrentTranslationDir = SwerveUtils.StepTowardsCircular(CurrentTranslationDir, InputTranslationDir, DirectionSlewRate * ElapsedTime);
        CurrentTranslationMag = MagLimiter.calculate(InputTranslationMag);
      }
      else if (AngleDif > 0.85*Math.PI) {
        if (CurrentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          CurrentTranslationMag = MagLimiter.calculate(0.0);
        }
        else {
          CurrentTranslationDir = SwerveUtils.WrapAngle(CurrentTranslationDir + Math.PI);
          CurrentTranslationMag = MagLimiter.calculate(InputTranslationMag);
        }
      }
      else {
        CurrentTranslationDir = SwerveUtils.StepTowardsCircular(CurrentTranslationDir, InputTranslationDir, DirectionSlewRate * ElapsedTime);
        CurrentTranslationMag = MagLimiter.calculate(0.0);
      }
      PreviousTime = CurrentTime;
      
      xSpeedCommanded = CurrentTranslationMag * Math.cos(CurrentTranslationDir);
      ySpeedCommanded = CurrentTranslationMag * Math.sin(CurrentTranslationDir);
      CurrentRotation = RotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      CurrentRotation = rot;
    }
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered    = CurrentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    FrontLeft.setDesiredState(swerveModuleStates[0]);
    FrontRight.setDesiredState(swerveModuleStates[1]);
    RearLeft.setDesiredState(swerveModuleStates[2]);
    RearRight.setDesiredState(swerveModuleStates[3]);
  }
 //  !!!!! END !!!!!




  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    FrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    FrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    RearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    RearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  /**
   * Tends to reset fieldrealtive
   */
  public void resetRelative() {

    FrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    FrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    RearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    RearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));

    zeroHeading();
  }



  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    FrontLeft.setDesiredState(desiredStates[0]);
    FrontRight.setDesiredState(desiredStates[1]);
    RearLeft.setDesiredState(desiredStates[2]);
    RearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    FrontLeft.resetEncoders();
    RearLeft.resetEncoders();
    FrontRight.resetEncoders();
    RearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    Gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(Gyro.getAngle()).getDegrees();
  }
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return Gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(FrontLeft.getState(), FrontRight.getState(), RearLeft.getState(), RearRight.getState());
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.8);
    FrontLeft.setDesiredState(swerveModuleStates[0]);
    FrontRight.setDesiredState(swerveModuleStates[1]);
    RearLeft.setDesiredState(swerveModuleStates[2]);
    RearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command goToPose2d(Pose2d pose){
    System.out.println(pose);
    return AutoBuilder.pathfindToPose
      (pose,
      new PathConstraints(4.0, 4.0, 2*Math.PI, 4*Math.PI)
      );
      
                  
  }
}