package frc.robot.commands;

import frc.robot.subsystems.IntakeOuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class feederOuttake extends Command {
  private final IntakeOuttakeSubsystem m_outtake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public feederOuttake(IntakeOuttakeSubsystem subsystem) {
    this.m_outtake = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_outtake.feederOuttake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_outtake.endOuttake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
