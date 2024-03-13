// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain.SimSwerveModule;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPRamseteController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathRamsete;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;


@SuppressWarnings("unused")
public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */


    private final static WPI_TalonSRX left1 = new WPI_TalonSRX(4) ;
    private final WPI_TalonSRX left2 = new WPI_TalonSRX(6);
    private final static WPI_TalonSRX right1 = new WPI_TalonSRX(5);
    private final WPI_TalonSRX right2 = new WPI_TalonSRX(8);
    private final DifferentialDrive Robotdrivetrain = new DifferentialDrive(left1, right1);
    private static final double rotationsToFeet = (1.0 / 4096 * 6 * Math.PI / 12)/3.281;
    //private final PIDController drivetrainPIDController = new PIDController(0.5, 0.5, 0.5);
    public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.55);
    double distanceCovered = 0;
    double leftspeed = 0;
    double rightspeed = 0;
    double startTime = 0;
    ChassisSpeeds chassisSpeedsSupplier;
    static Pose2d currentRobotPose;
    double linearVelocity;
    double headingValue;
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final Field2d m_field = new Field2d();
    private static Encoder leftEncoder = new Encoder(1, 2, false, EncodingType.k4X);
    private static Encoder rightEncoder = new Encoder(3, 4, true, EncodingType.k4X);
    static double LeftDistance = -(left1.getSelectedSensorPosition() * rotationsToFeet);
    static double RightDistance = right1.getSelectedSensorPosition() * rotationsToFeet;
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), LeftDistance, RightDistance);
    boolean isFirstPath = true;
    //public PathPlannerTrajectory traj = new PathPlannerTrajectory(null, chassisSpeeds, null);


public void drivetrain() {
  left2.follow(left1);
  right2.follow(right1);
  right2.setInverted(InvertType.FollowMaster);
  left2.setInverted(InvertType.FollowMaster);
  EncoderInitReset();
  //gyro.calibrate();
  resetGyro();
  AutoBuilder.configureRamsete(
    this::getPose, // Robot pose supplier
    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    this::getCurrentSpeeds, // Current ChassisSpeeds supplier
    this::driveRobotRelative, // Method that will drive the robot given ChassisSpeeds
    new ReplanningConfig(), // Default path replanning config. See the API for the options here
    () -> {
        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    },
  this // Reference to this subsystem to set requirements
  );
  SmartDashboard.putData("Field", m_field);
  PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Rotation2d gryoAngle = gyro.getRotation2d();
    LeftDistance = -(left1.getSelectedSensorPosition() * rotationsToFeet);
    RightDistance = right1.getSelectedSensorPosition() * rotationsToFeet;
    odometry.update(gryoAngle, LeftDistance, RightDistance);
    distanceCovered = (LeftDistance + RightDistance) / 2;
    m_field.setRobotPose(getPose());
    SmartDashboard.putNumber("Distance Covered", getEncoderDistance());
    SmartDashboard.putNumber("Left Distance in meters", LeftDistance);
    SmartDashboard.putNumber("RightDrivetrainDistanceTravelled", RightDistance);
    SmartDashboard.putNumber("Heading of the robot", getHeading());
    SmartDashboard.putNumber("Heading of the robot", getPoseHeading());
    SmartDashboard.putNumber("Unbounded Heading", getUnboundedHeading());
    SmartDashboard.putData("Field", m_field);
  }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public void Tankdrive(double leftspeed, double rightspeed) {
      Robotdrivetrain.tankDrive(leftspeed, rightspeed);
    }

    public double getHeading(){
      headingValue = gyro.getAngle();
      return Math.IEEEremainder(headingValue, 360);
    }

    public double getUnboundedHeading(){
    return gyro.getAngle();
    }

    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

    public double getPoseHeading(){
      return odometry.getPoseMeters().getRotation().getDegrees();
    }

    public void resetPose(Pose2d pose) {
      EncoderInitReset();
      odometry.resetPosition(gyro.getRotation2d(), LeftDistance, RightDistance, pose);
    }

    public void resetGyro(){
      gyro.reset();
    }

    // getting the current speed of the robot
    public ChassisSpeeds getCurrentSpeeds() {
        var wheelSpeeds = new DifferentialDriveWheelSpeeds();
        chassisSpeedsSupplier = kinematics.toChassisSpeeds(wheelSpeeds);
        //double linearVelocity = chassisSpeedsSupplier.vxMetersPerSecond;
        //double angularVelocity = chassisSpeedsSupplier.omegaRadiansPerSecond;
        return chassisSpeedsSupplier;
        //return kinematics.toChassisSpeeds(null);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
      DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
      DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
      return wheelSpeeds;
    }

    // outputting speeds to the robot
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds){

      DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
      DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
      double leftVelocity = wheelSpeeds.leftMetersPerSecond;
      double rightVelocity = wheelSpeeds.rightMetersPerSecond;
      Tankdrive(leftVelocity, rightVelocity);
    }

    public static double getEncoderDistance(){
    return ((LeftDistance+RightDistance))/2;
  }

  public void EncoderInitReset(){
    left1.setSelectedSensorPosition(0, 0, 10);
    right1.setSelectedSensorPosition(0, 0, 10);
    rightEncoder.reset();
    leftEncoder.reset();
  }
public Command resetPathPlannerPose(boolean isFirstPathAuto) {
  isFirstPath = isFirstPathAuto;
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetPose(getPose());;
              isFirstPath = false;
          }
        }));
}
}
