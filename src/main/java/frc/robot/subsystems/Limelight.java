package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;


@SuppressWarnings("unused")

public class Limelight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  public static double limelightMountAngle = 45.0;
  public static double limelightLensHeightInches = 18;
  public static double goalHeightInches = 53;
  public static double aprilTagId;
  public double distanceFromAprilTag = 0;
  public static boolean limelightTargetFound = false;
  public static double[] botpose;
  public final double max_speed = 0.5;
  public double tx = 0;
  public double ty = 0;
  public double tv = 0;
  public double ta = 0;
  public static double targetOffsetAngle_Vertical = 0;
  public static double angleToGoalDegrees = 0;
  public static double angletoGoalRadians = 0;
  public static double distanceFromLimelighttoGoalinInches = 0;
  private static final String table_name = "limelight";
  private NetworkTable table;
  private LimelightHelpers robotLimelight = new LimelightHelpers();
  public Pose2d limelightRobotPose = LimelightHelpers.getBotPose2d("limelight");
  public Pose3d aprilTagPoseWRTRobot = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
  public Pose3d aprilTagPoseWRTCamera = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
  public Pose2d targetPose2dWRTRobot = aprilTagPoseWRTRobot.toPose2d();
  public Pose2d targetPose2dWRTCamera = aprilTagPoseWRTCamera.toPose2d();
    
    public Limelight() {
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table = NetworkTableInstance.getDefault().getTable(table_name);
    double tx = table.getEntry("tx").getDouble(2950);
    double ty = table.getEntry("ty").getDouble(2950);
    double tv = table.getEntry("tv").getDouble(2950);
    double ta = table.getEntry("ta").getDouble(2950);
    double apriltagId = table.getEntry("tid").getDouble(2950);
    if (tv < 1) {
        limelightTargetFound = false;
    } else if (tv == 0) {
        limelightTargetFound = true;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    }
    targetOffsetAngle_Vertical = ty;
    angleToGoalDegrees = limelightMountAngle + targetOffsetAngle_Vertical;
    angletoGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    distanceFromLimelighttoGoalinInches = (goalHeightInches - limelightLensHeightInches)/ Math.tan(angletoGoalRadians);
    limelightRobotPose = LimelightHelpers.getBotPose2d("limelight");
    aprilTagPoseWRTRobot = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    aprilTagPoseWRTCamera = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    targetPose2dWRTRobot = aprilTagPoseWRTRobot.toPose2d();
    targetPose2dWRTCamera = aprilTagPoseWRTCamera.toPose2d();
    SmartDashboard.putNumberArray("botpose", NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]));
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("April Tag Id", apriltagId);
    SmartDashboard.putNumber("Distance from goal in feet", (distanceFromLimelighttoGoalinInches / 12));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}