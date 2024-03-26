package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ClimberSubsystem extends SubsystemBase {

    public VictorSPX climberMotor1 = new VictorSPX(9);
    public VictorSPX climberMotor2 = new VictorSPX(10);
    public VictorSPX climberMotor3 = new VictorSPX(11);

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {


    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void retractClimber(){
    climberMotor1.set(VictorSPXControlMode.PercentOutput, OperatorConstants.climberMotor1);
  }

  public void lowerClimber(){
    climberMotor1.set(VictorSPXControlMode.PercentOutput, -OperatorConstants.climberMotor1);
  }

  public void raiseClimber(){
    climberMotor2.set(VictorSPXControlMode.PercentOutput, OperatorConstants.climberMotor2);
    climberMotor3.set(VictorSPXControlMode.PercentOutput, OperatorConstants.climberMotor3);
  }

  public void climb(){
    climberMotor3.set(VictorSPXControlMode.PercentOutput, -OperatorConstants.climberMotor3);
  }

  public void doNothingMotor1(){
    climberMotor1.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void doNothingMotor2(){
    climberMotor2.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void doNothingMotor3(){
    climberMotor3.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void doNothing(){
    climberMotor1.set(VictorSPXControlMode.PercentOutput, 0);
    climberMotor2.set(VictorSPXControlMode.PercentOutput, 0);
    climberMotor3.set(VictorSPXControlMode.PercentOutput, 0);
  }
}

