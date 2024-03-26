package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class raiseClimber extends Command {
  private final ClimberSubsystem m_climber;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public raiseClimber(ClimberSubsystem climber_subsystem) {
    this.m_climber = climber_subsystem;
    
    addRequirements(climber_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.raiseClimber();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.doNothingMotor2();
    m_climber.doNothingMotor3();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

