package frc.robot;

import javax.xml.crypto.dsig.Transform;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class ShooterConstants{
      public static final int SHOOTER_MOTOR_ID = 27;
      public static final double FLYWHEEL_P = 0.00015;
      public static final double FLYWHEEL_I = 0.00000;
      public static final double FLYWHEEL_D = 0.00000;
    }
    public static class PhotonVisionConstants {
    //Transform3d from the center of the robot to the camera mount position (ie, robot ➔ camera) in the Robot Coordinate System
    //The Cameras are mounted on the back of the value so all transform signs are flipped (not rotations). + ➔ -
    
    public static final Transform3d ROBOT_TO_LEFT_CAMERA =
        new Transform3d(Units.inchesToMeters(12.460005), Units.inchesToMeters(-10.3415), 0, new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(45)));

    public static final Transform3d ROBOT_TO_FORWARD_CAMERA =
        new Transform3d(Units.inchesToMeters(12.766455), Units.inchesToMeters(11.158455), 0, new Rotation3d(0, Units.degreesToRadians(10), 0));

    public static final double AMBIGUITY_RATIO_CUTOFF = 0.2;
    public static final double POINT_AT_P = 0.002;
    public static final double POINT_AT_I = 0;
    public static final double POINT_AT_D = 0;
    public static final double POINT_AT_P_GENERAL = 0.013;
  }

  public static class ClimbConstants {
    public static final int ClimbMotorId = 33;
    public static final double climbForwardSpeed = .75; //1 .5
    public static final double ClimbBackwardSpeed = -.6;
    public static final int ClimbStop = 0;
    public static final int ClimbServoStop = 0;
    public static final double climbLockServoPosition = 0.25;
    public static final double climbUnlockServoPosition = 0;
    public static final double ClimbInLimit = -3.6;
    public static final double ClimbOutLimit = -1;
    public static final double ClimbP = 0.05;
    public static final double ClimbI = 0;
    public static final double ClimbD = 0;
    public static final double ClimbReadySetpoint = -4.34; //-4.18   4.24
    public static final double ClimbLockedInSetpoint = 1; //0 is too low... 2 almost works, but chain touches elevator
    public static final int climbServo = 1;
  }

  public static class PathPlannerConstants {
    // PID Autobuilder
    public static final PIDConstants TRANSLATIONAL_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ROTATIONAL_PID = new PIDConstants(5, 0, 0);
    
    // PathPlanner Pathfinding Constants
    public static final double MAX_VELOCITY_MPS = 4.3; //3
    public static final double MAX_ACCELERATION_MPS = 5; //4
    public static final double MAX_ANGULAR_VELOCITY_RAD = Units.degreesToRadians(90); //Units.degreesToRadians(90)
    public static final double MAX_ANGULAR_ACCELERATION_RAD = Units.degreesToRadians(400); //Units.degreesToRadians(400)
    public static final double NOMINAL_VOLTAGE_VOLTS = 11.5;

    public static final double PATHFIND_END_SPEED_MPS = 0.1;
    public static final double PATHFIND_END_SPEED_MPS_STATION = 1;//.5
    

  }
  public static class PoseConstants {
    public static final Pose2d APRIL_TAG_POSE = new Pose2d(-0.59, -0.93, new Rotation2d(143.83));
    public static final double[] BLUE_ALLIANCE_HUB_LOCATION = {4.62534, 4.034536};
    public static final double[] RED_ALLIANCE_HUB_LOCATION = {11.91514, 4.034536};
  }
  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 27;
     public static final Transform3d ROBOT_RELATIVE_TURRET_POSITION = new Transform3d(
      0, 
      Units.inchesToMeters(9.5),
      0,
      new Rotation3d(0, 0, 0));

    public static final double ROBOT_RELATIVE_TURRET_STARTING_ANGLE = Math.atan(ROBOT_RELATIVE_TURRET_POSITION.getX()/ROBOT_RELATIVE_TURRET_POSITION.getY());

    public static final double ROBOT_RELATIVE_TURRET_MAGNITUDE = Math.sqrt(ROBOT_RELATIVE_TURRET_POSITION.getX() * ROBOT_RELATIVE_TURRET_POSITION.getX() 
                                                                          + ROBOT_RELATIVE_TURRET_POSITION.getY() * ROBOT_RELATIVE_TURRET_POSITION.getY());

    public static final double TURRET_GEAR_RATIO = 15;
  }
}
