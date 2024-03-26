// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.driveRobotCommand;
import frc.robot.commands.feederOuttake;
import frc.robot.commands.setSpeedArm;
import frc.robot.commands.intake;
import frc.robot.commands.intakeOuttakeDefault;
import frc.robot.commands.inverseIntake;
import frc.robot.commands.lowerClimber;
import frc.robot.commands.outtake;
import frc.robot.commands.setArmPosition;
import frc.robot.commands.climb;
import frc.robot.commands.climberDefault;
import frc.robot.commands.raiseClimber;
import frc.robot.commands.RetractClimber;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ClimberSubsystem;

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

  // instantiating all the subsystems to use the commands linked to them.
  private final static DrivetrainSubsystem m_Drivetrain = new DrivetrainSubsystem();
  private final static Arm arm = new Arm();
  private final static IntakeOuttakeSubsystem intakeOuttake = new IntakeOuttakeSubsystem();
  private final static Limelight limelight = new Limelight();
  private final static ClimberSubsystem climber = new ClimberSubsystem();

  // CommandXboxController because the code is command based
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController climberController = new CommandXboxController(1);

  // making all Xbox buttons to Triggers because command based likes Triggers and this is an easy way I found to do it. driver is the main controller and ClimberController is for climber because we ran out of triggers with just one controller.
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

  Trigger xButtonClimberController = climberController.x();
  Trigger yButtonClimberController = climberController.y();
  Trigger bButtonClimberController = climberController.b();
  Trigger aButtonClimberController = climberController.a();

  // path constraints to be used when robot is driving auto (using the limelight to align correctly with the amp). pretty self explanatory
  PathConstraints pathConstraints = new PathConstraints(3, 3, 0, 0);

  // an autoChooser helps you select your autos from SmartDashBoard itself. always the safest option because you dont want to deploy your code before every match to change the auto.
  private final SendableChooser<Command> autoChooser;

  // PathPlanner likes bezier points so we give it what it wants. Basically the points your robot should cover while following the path. I got these points from the limelight when it reads the april tag
  public List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(limelight.limelightRobotPose,limelight.targetPose2dWRTRobot);

  // end state of the robot when it is done. again pretty self explanatory
  public GoalEndState endState = new GoalEndState(0, Rotation2d.fromDegrees(0));

  // building the path for the robot to align with the amp and to be in the scoring position.
  public PathPlannerPath ToAmp = new PathPlannerPath(bezierPoints, pathConstraints, endState, true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // registering commands for PathPlanner. this helps load the commands in PathPlanner and build a complete auto routine using the software.
    NamedCommands.registerCommand("Outtake Position", new setArmPosition(arm, OperatorConstants.armOuttakePos));
    NamedCommands.registerCommand("Intake Position", new setArmPosition(arm, OperatorConstants.armIntakePos));
    NamedCommands.registerCommand("Intake motors on", new intake(intakeOuttake));
    NamedCommands.registerCommand("Intake motors on with out take", new intake(intakeOuttake).withTimeout(2));
    NamedCommands.registerCommand("Out take motors on", new outtake(intakeOuttake).withTimeout(2));
    NamedCommands.registerCommand("Lowering the climber to 0 degrees", new lowerClimber(climber).withTimeout(2));

    // configuring auto builder. very imp to make pathplanner talk to the robot through the code. you go wrong here you mess up the the entire auto of the robot. this code can be copy pasted along with the other necessary methods from the DrivetrainSubsystem.java
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


    // Configure the trigger bindings. basically your buttons and the command you want them to execute.
    configureBindings();

    // every subsystem needs to have a default command. otherwise you will have an error while deploying.

    // basic tankdrive commands. you can see that I have done it differently here by using RunCommand than what I did for buttons and that is because the triggers or the sticks on the xBox controllers are not buttons that can just be true or false, they can have varying values and we want to use those varying values.
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

    // default command for the climber. all motors have 0 speed
    climber.setDefaultCommand(new climberDefault(climber));

    // building autochooser on smartdashboard so that we can choose our auto from smartdashboard
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

    // the way you do it is you choose a trigger, choose if you want whileTrue or onTrue or anything else and then bind the command to it. notice that every command requires a subsystem.

    // intake. this is the easiest way to configure a button to a command.
    leftBumper.whileTrue(new intake(intakeOuttake));   
    
    // outtake
    rightBumper.whileTrue(new outtake(intakeOuttake));

    // right trigger moves the arm clockwise.
    rightTrigger.whileTrue(
      new RunCommand(
            () ->
                arm.setArmSpeed(
                -driver.getRightTriggerAxis()*0.5),
            arm));
    
    // inverse intake and outtake motors to remove the note incase it gets stuck.
    aButton.whileTrue(new inverseIntake(intakeOuttake));

    
    // feeder 
    bButton.whileTrue(new feederOuttake(intakeOuttake));

    // robot to amp auto using the limelight
    xButton.onTrue(new RunCommand(
            () ->
            AutoBuilder.followPath(ToAmp)));

    // retract climber (make it 90 degrees)
    xButtonClimberController.whileTrue(new RetractClimber(climber));

    // lower climber (make it "0 degrees")
    bButtonClimberController.whileTrue(new lowerClimber(climber));

    // raise climber
    xButtonClimberController.whileTrue(new raiseClimber(climber));

    // climb
    yButtonClimberController.whileTrue(new climb(climber));
  }

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command AutoInitialization(){

    // resetting everything before auto starts. aka auto initialization

    SequentialCommandGroup autoInitialization = new SequentialCommandGroup(
      new InstantCommand(() -> {
       m_Drivetrain.EncoderInitReset();
       m_Drivetrain.resetGyro(); 
      }, m_Drivetrain));

    return autoInitialization;
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(
      AutoInitialization(),autoChooser.getSelected());
  }

}

/*
Brief description of how command based programming works.

Firstly, you have your subsystems, which is dividing your robot into smaller systems. As you can see, I divided it into drivetrain, arm, intake/outtake and climber.

The way I did it is I wrote everything I wanted my subsystem to do in the subsystem code. Initialization, periodic, and all other methods. I would recommend doing the same so that when you want to include one thing the subsystem does in multiple commands it becomes easier and avoids repetition.

Next write your commands. Commands are officially writing what you want to do with the subsystem. Just call all the methods you need from what you wrote in the subsystems. Every command has a constructor and make sure you have addRequirements in your constructor. This ensures that one part of the subsystem is not running 2 functions at the same time. A subsystem can have multiple parts doing different things but you don't want one motor wanting to move in 2 different directions at the same time, right?

The template of a command is pretty self explanatory. You have your initialization, execution and end.  Initialization sets up what you need for that specific command, execute is running the command and end is what you want to do once the command is done.

Once you have all your commands and subsystems, you come to RobotContainer, where you control everything by configuring them with triggers. And you are good to go!

Avoid writing any extra code in the Robot.java file.

Use the Constants.java file to store your constant values. This is a very good practice, since, if needed, you only need to change your value once in the constants file.

Syntax and more information about everything can be found online. The official libraries, official documentations, chief delphi, youtube tutorials, and alumni are your best friends.
 */