package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeOuttakeSubsystem;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField", "unused"})

public class intakeOuttakeDefault extends Command {

    private final IntakeOuttakeSubsystem m_default;
    public intakeOuttakeDefault (IntakeOuttakeSubsystem sub) {
        this.m_default = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        System.out.println("Default Intake");
    }
    // Called every time the scheduler runs while the command is scheduled.

    @Override
    public void execute() {
        m_default.endIntake();
        m_default.endOuttake();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_default.endIntake();
        m_default.endOuttake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }



    
}
