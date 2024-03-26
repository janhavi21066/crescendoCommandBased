package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class lowerClimber extends Command {

    private final ClimberSubsystem m_climber;
    public lowerClimber(ClimberSubsystem climber) {
        this.m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }
    // Called every time the scheduler runs while the command is scheduled.

    @Override
    public void execute() {
        m_climber.lowerClimber();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climber.climberMotor1.set(VictorSPXControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }    
}
