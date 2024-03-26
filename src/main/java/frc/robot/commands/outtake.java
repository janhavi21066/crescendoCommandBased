package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeOuttakeSubsystem;

public class outtake extends Command {

    private final  IntakeOuttakeSubsystem m_outtake;
    public outtake(IntakeOuttakeSubsystem sub) {
        this.m_outtake = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing Outtake");
    }
    // Called every time the scheduler runs while the command is scheduled.

    @Override
    public void execute() {
        m_outtake.startAmpOuttake();
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
