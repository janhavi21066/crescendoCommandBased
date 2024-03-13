package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class setSpeedArm extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Arm armSubsystem;;
    private double speed;

    public setSpeedArm (Arm subsystem, double speed){
        this.armSubsystem = subsystem;
        this.speed = speed;
        addRequirements(armSubsystem);
    }
    
    
    @Override
    public void initialize() {
      armSubsystem.initialization();
      System.out.println("Giving speed to the arm");
    }
    // Called every time the scheduler runs while the command is scheduled.
    
    
    @Override
    public void execute() {
    armSubsystem.setArmSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
