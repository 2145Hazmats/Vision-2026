// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;

public class Visionsubsystem extends SubsystemBase {
  private final Field2d visionField = new Field2d();
  private CommandSwerveDrivetrain m_drivetrain = null;

  private final double RadiusOfToleranceSquare = 0.5;

  PhotonCamera cameraLeft = new PhotonCamera("CameraLeft");
  private PhotonPipelineResult leftResult = null;
  private PhotonTrackedTarget leftTrackedTarget = null;
  private PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(
    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    PhotonVisionConstants.ROBOT_TO_LEFT_CAMERA
  );
  private EstimatedRobotPose leftEstimatedRobotPose = null;

  PhotonCamera cameraForward = new PhotonCamera("CameraForward");
  private PhotonPipelineResult forwardResult = null;
  private PhotonTrackedTarget forwardTrackedTarget = null;
  private PhotonPoseEstimator forwardPoseEstimator = new PhotonPoseEstimator(
    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    PhotonVisionConstants.ROBOT_TO_FORWARD_CAMERA
  );
  private EstimatedRobotPose forwardEstimatedRobotPose = null;

  private Matrix<N3, N1> curStdDevs;
  private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 6); //VecBuilder.fill(4, 4, 8); (2 , 2 , 8)
  private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, .75); //0.5,0.5,1
  /** Creates a new Visionsubsystem. */
  public Visionsubsystem(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) { curStdDevs = kSingleTagStdDevs; }
    else {
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      for (var tgt : targets) { // Precalculation - see how many tags we found, and calculate an average-distance metric
        var tagPose = leftPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) { // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else { // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) estStdDevs = kMultiTagStdDevs;
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
              estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          curStdDevs = estStdDevs;
      }
    }
  }

  public void addVisionPose2d(Pose2d pose2d, double timestampSeconds) {
    SmartDashboard.putNumber("aVP2d pose2d X", pose2d.getX());
    SmartDashboard.putNumber("aVP2d pose2d Y", pose2d.getY());
    SmartDashboard.putNumber("aVP2d pose2d Rot", pose2d.getRotation().getDegrees());
    SmartDashboard.putNumber("aVP2d timestampSeconds", timestampSeconds);
    // Sets trust value for vision measurements
    // charizardsSkateboard.setVisionMeasurementStdDevs(curStdDevs);
    // charizardsSkateboard.addVisionMeasurement(pose2d, timestampSeconds);
    m_drivetrain.setVisionMeasurementStdDevs(curStdDevs);

    double xUpperLimitOfTrustBox = m_drivetrain.getState().Pose.getX() + RadiusOfToleranceSquare;
    double xLowerLimitOfTrustBox = m_drivetrain.getState().Pose.getX() - RadiusOfToleranceSquare;

    double yUpperLimitOfTrustBox = m_drivetrain.getState().Pose.getY() + RadiusOfToleranceSquare;
    double yLowerLimitOfTrustBox = m_drivetrain.getState().Pose.getY() - RadiusOfToleranceSquare;
    

    //  SmartDashboard.putNumber("charizardsSkateboard X", charizardsSkateboard.getState().Pose.getX());
    //  SmartDashboard.putNumber("charizardsSkateboard Y", charizardsSkateboard.getState().Pose.getY());
    //  SmartDashboard.putNumber("charizardsSkateboard Rot", charizardsSkateboard.getState().Pose.getRotation().getDegrees());

    if (MathUtil.isNear(0, m_drivetrain.getState().Speeds.vxMetersPerSecond, 0.1) && MathUtil.isNear(0, m_drivetrain.getState().Speeds.vyMetersPerSecond, 0.1)) {
      //are we moving,, if so then add trust box  
      m_drivetrain.addVisionMeasurement(pose2d, timestampSeconds);
    } else if(pose2d.getX() >= xLowerLimitOfTrustBox && pose2d.getX() <= xUpperLimitOfTrustBox && pose2d.getY() >= yLowerLimitOfTrustBox && pose2d.getY() <= yUpperLimitOfTrustBox) {
      m_drivetrain.addVisionMeasurement(pose2d, timestampSeconds);
    }
  }

  public void CameraTeamColorSwitcher(boolean isBlue) {
    if (isBlue == true) {
      cameraLeft.setPipelineIndex(0);
      cameraForward.setPipelineIndex(0);
    }
    else {
      cameraLeft.setPipelineIndex(1);
      cameraForward.setPipelineIndex(1);
    }
  }

  @Override
  public void periodic() {
    // Get camera results
    leftResult = cameraLeft.getLatestResult();
    forwardResult = cameraForward.getLatestResult();
    // backLeftResult = backLeftCamera.getLatestResult();
    // backRightResult = backRightCamera.getLatestResult(); //NEEDS CHANGING BEFORE WE RETIRE FOR MICHALS SAFTEY NEXT YEAR

    // Central Camera
    // Try to update "latestRobotPose" with a new "EstimatedRobotPose" using a "PhotonPoseEstimator"
    // If "latestRobotPose" is updated, call addVisionPose2d() and pass the updated "latestRobotPose" as an argument
    try {
      // Only accepts camera results if they see more than 1 april tag, or if it sees 1 april tag and the poseAmbiguity is low
      // COMMENT OUT THE LINE BELOW THIS AND IT'S CLOSING BRACKETS IF THIS DOESN'T WORK
      if ((leftResult.getTargets().size() == 1 && leftResult.getBestTarget().poseAmbiguity < PhotonVisionConstants.AMBIGUITY_RATIO_CUTOFF) 
      || leftResult.getTargets().size() > 1) {
        leftEstimatedRobotPose = leftPoseEstimator.update(leftResult).get();
        updateEstimationStdDevs(leftPoseEstimator.update(leftResult), cameraLeft.getAllUnreadResults().get(0).getTargets());
        addVisionPose2d(leftEstimatedRobotPose.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
        visionField.setRobotPose(leftEstimatedRobotPose.estimatedPose.toPose2d());
      }
      SmartDashboard.putBoolean("leftLatestRobotPose Update", true);
    } catch (Exception e) {
      leftEstimatedRobotPose = null;
      SmartDashboard.putBoolean("leftLatestRobotPose Update", false);
    }

    // Same thing but for the left camera
    try {
      // Only accepts camera results if they see more than 1 april tag, or if it sees 1 april tag and the poseAmbiguity is low
      // COMMENT OUT THE LINE BELOW THIS AND IT'S CLOSING BRACKETS IF THIS DOESN'T WORK
      if ((forwardResult.getTargets().size() == 1 && forwardResult.getBestTarget().poseAmbiguity < PhotonVisionConstants.AMBIGUITY_RATIO_CUTOFF) 
      || forwardResult.getTargets().size() > 1) {
        forwardEstimatedRobotPose = forwardPoseEstimator.update(forwardResult).get();
        updateEstimationStdDevs(forwardPoseEstimator.update(forwardResult), cameraForward.getAllUnreadResults().get(0).getTargets());
        addVisionPose2d(forwardEstimatedRobotPose.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
        visionField.setRobotPose(forwardEstimatedRobotPose.estimatedPose.toPose2d());
      }
      SmartDashboard.putBoolean("forwardLatestRobotPose Update", true);
    } catch (Exception e) {
      forwardEstimatedRobotPose = null;
      SmartDashboard.putBoolean("forwardLatestRobotPose Update", false);
    }
    // This method will be called once per scheduler run
    SmartDashboard.putData("VisionField", visionField);
  }
}
