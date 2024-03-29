package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class IntakeOuttakeSubsystem extends SubsystemBase {

    private final WPI_TalonSRX intake1 = new WPI_TalonSRX(7);
    private final WPI_TalonSRX intake2 = new WPI_TalonSRX(9);
    private final WPI_TalonSRX outtake = new WPI_TalonSRX(3);



  /** Creates a new ExampleSubsystem. */
  public void IntakeOuttakeSub() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void startIntake()
  {
    intake1.set(OperatorConstants.intakeSpeed);
  }

  public void endIntake()
  {
    intake1.set(0);
  }


  public void startAmpOuttake()
  {
    outtake.set(OperatorConstants.outtakeSpeed);
  }

  public void feederOuttake(){
    outtake.set(OperatorConstants.feederOuttake);
  }

  public void endOuttake()
  {
    outtake.set(0);
  }

  public void inverse()
  {
    intake1.set(-OperatorConstants.intakeSpeed);
    outtake.set(-OperatorConstants.outtakeSpeed);
  }

  public void initialization()
  {
    intake2.follow(intake1);
    intake2.setInverted(InvertType.FollowMaster);
  }
}