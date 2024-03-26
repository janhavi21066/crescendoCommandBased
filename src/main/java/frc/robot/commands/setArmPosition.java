package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class setArmPosition extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Arm arm;
    private double position;

    public setArmPosition (Arm subsystem, double position) {
        this.arm = subsystem;
        this.position = position;
        addRequirements(arm);
    }
    
    
    @Override
    public void initialize() {
      arm.initialization();
      arm.motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
      arm.motor1.setSelectedSensorPosition(this.position);
      arm.motor1.configPeakOutputForward(0.5);
      arm.motor1.configPeakOutputReverse(0.5);
      System.out.println("Initializing Move Arm");
    }
    // Called every time the scheduler runs while the command is scheduled.
    
    
    @Override
    public void execute() {
      arm.setSpeedPID(position);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
