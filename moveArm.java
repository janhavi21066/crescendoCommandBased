/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class moveArm extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Arm armSubsystem;;
    private double position;

    public moveArm (Arm subsystem, double position){
        this.armSubsystem = subsystem;
        this.position = position;
        addRequirements(armSubsystem);
    }
    
    
    @Override
    public void initialize() {
        armSubsystem.armEncoderreset();
        System.out.println("Initializing Move Arm");
    }
    // Called every time the scheduler runs while the command is scheduled.
    
    
    @Override
    public void execute() {
    armSubsystem.setArmPosition(position);
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
}*/
