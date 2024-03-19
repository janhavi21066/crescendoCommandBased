// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.driveRobotCommand;
//import frc.robot.commands.moveArm;
import frc.robot.commands.setSpeedArm;
import frc.robot.commands.intake;
import frc.robot.commands.intakeOuttakeDefault;
import frc.robot.commands.inverseIntake;
import frc.robot.commands.outtake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeOuttakeSubsystem;


@SuppressWarnings("unused")
public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DrivetrainSubsystem subsystem) {
    //return Commands.sequence(subsystem.exampleMethodCommand(), new Drivetrain());
    return null;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
