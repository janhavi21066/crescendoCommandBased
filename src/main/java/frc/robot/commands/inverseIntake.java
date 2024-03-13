package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeOuttakeSubsystem;

public class inverseIntake extends Command {

    private final  IntakeOuttakeSubsystem m_intake;
    public inverseIntake(IntakeOuttakeSubsystem sub) {
        this.m_intake = sub;
    }

    @Override
    public void initialize() {
        m_intake.initialization();
        System.out.println("Initializing Intake");
    }
    // Called every time the scheduler runs while the command is scheduled.

    @Override
    public void execute() {
        m_intake.inverse();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.endIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }



    
}
