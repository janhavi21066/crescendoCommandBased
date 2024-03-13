package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


@SuppressWarnings("unused")

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX motor1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX motor2 = new WPI_TalonSRX(2);
  private static Encoder  armMotorEncoder = new Encoder(5, 6, true, EncodingType.k4X);
  private static final double rotationsToDegrees = 360 / 512 * 26 / 42 * 18 / 60 * 18 / 84;
  private static final double intakeAngle = 0; //degrees
  private static final double outtakeAngle = 120; //degrees



  public Arm() {
      motor2.follow(motor1);
      motor2.setInverted(InvertType.FollowMaster);    
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Arm Angle", armPositionDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void armEncoderreset() {
      armMotorEncoder.reset();
  }

  public void setArmSpeed(double speed){
    motor1.set(speed);
  }

  /*public void setArmPosition(double pos){
    if (pos<armPositionDegrees())
    {
        motor1.set(0.3);
    }
    else if (pos>armPositionDegrees())
    {
        motor1.set(-0.3);
    }
    else if(pos==armPositionDegrees())
    {
        motor1.set(0);
    }
  }*/

  public void initialization()
  {
    motor2.follow(motor1);
    motor2.setInverted(InvertType.FollowMaster);    
  }
}
