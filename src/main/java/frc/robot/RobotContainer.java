// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.driveRobotCommand;
//import frc.robot.commands.moveArm;
import frc.robot.commands.setSpeedArm;
import frc.robot.commands.intake;
import frc.robot.commands.intakeOuttakeDefault;
import frc.robot.commands.inverseIntake;
import frc.robot.commands.setArmPosition;
import frc.robot.commands.outtake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.Limelight;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathRamsete;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final static DrivetrainSubsystem m_Drivetrain = new DrivetrainSubsystem();
  private final static Arm arm = new Arm();
  private final static IntakeOuttakeSubsystem intakeOuttake = new IntakeOuttakeSubsystem();
  private final static Limelight limelight = new Limelight();

  CommandXboxController driver = new CommandXboxController(0);

  Trigger leftBumper = driver.leftBumper();
  Trigger rightBumper = driver.rightBumper();
  Trigger xButton = driver.x();
  Trigger yButton = driver.y();
  Trigger bButton = driver.b();
  Trigger aButton = driver.a();
  Trigger leftTrigger = driver.leftTrigger();
  Trigger rightTrigger = driver.rightTrigger();
  Trigger leftStick = driver.leftStick();
  Trigger rightStick = driver.rightStick();
  boolean isFirstPath = true;
  PathConstraints pathConstraints = new PathConstraints(3, 3, 0, 0);

  private final SendableChooser<Command> autoChooser;

  public List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(limelight.limelightRobotPose,limelight.targetPose2dWRTRobot);
  public GoalEndState endState = new GoalEndState(0, Rotation2d.fromDegrees(0));
  public PathPlannerPath ToAmp = new PathPlannerPath(bezierPoints, pathConstraints, endState, true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("Move arm from intake to outtake position", new setArmPosition(arm, 1258));
    NamedCommands.registerCommand("Move arm from outtake to intake position", new setArmPosition(arm, 0));
    NamedCommands.registerCommand("Intake motors on", new intake(intakeOuttake));
    NamedCommands.registerCommand("Out take motors on", new outtake(intakeOuttake).withTimeout(2));

    AutoBuilder.configureRamsete(
        m_Drivetrain::getPose, // Robot pose supplier
        m_Drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        m_Drivetrain::getCurrentSpeeds, // Current ChassisSpeeds supplier
        m_Drivetrain::driveRobotRelative, // Method that will drive the robot given ChassisSpeeds
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
        m_Drivetrain // Reference to this subsystem to set requirements
    );


    // Configure the trigger bindings
    configureBindings();
    m_Drivetrain.EncoderInitReset();
    

    // basic tankdrive drivetrain
    m_Drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                m_Drivetrain.Tankdrive(
                -driver.getLeftY(), driver.getRightY()),
            m_Drivetrain));


    // when left trigger axis is pressed move the arm anti clockwise
    arm.setDefaultCommand(
      new RunCommand(
            () ->
                arm.setArmSpeed(
                driver.getLeftTriggerAxis()*5),
            arm));


    // intake and outtake motors start at rest
    intakeOuttake.setDefaultCommand(new intakeOuttakeDefault(intakeOuttake));


  autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // intake
    leftBumper.whileTrue(new intake(intakeOuttake));   
    
    // out take
    rightBumper.whileTrue(new outtake(intakeOuttake));

    // right trigger moves the arm clockwise
    rightTrigger.whileTrue(
      new RunCommand(
            () ->
                arm.setArmSpeed(
                -driver.getRightTriggerAxis()*0.5),
            arm));
    
    // inverse intake and outtake motors to remove the note incase it gets stuck.
    aButton.whileTrue(new inverseIntake(intakeOuttake));

    xButton.whileTrue(new driveRobotCommand(m_Drivetrain, 0.45, -0.5));

    bButton.onTrue(new RunCommand(
            () ->
            AutoBuilder.followPath(ToAmp)));
          }

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command AutoInitialization(){

    SequentialCommandGroup autoInitialization = new SequentialCommandGroup(
      new InstantCommand(() -> {
       m_Drivetrain.EncoderInitReset();
       m_Drivetrain.resetGyro(); 
      }, m_Drivetrain));

    return autoInitialization;
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //PathPlannerPath path = PathPlannerPath.fromPathFile("note1");
    //return AutoBuilder.followPath(path);
    //return new PathPlannerAuto("3noteAuto");
    return new SequentialCommandGroup(
      AutoInitialization(),autoChooser.getSelected());
    //return autoChooser.getSelected();
  }
}