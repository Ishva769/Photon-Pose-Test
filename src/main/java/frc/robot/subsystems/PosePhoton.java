// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class PosePhoton extends SubsystemBase {
  /** Creates a new PosePhoton. */
 // ---------- Cameras (names must match PhotonVision dashboard) ----------
    public PhotonCamera frontCam = new PhotonCamera("frontCam");
    public PhotonCamera backCam  = new PhotonCamera("backCam");
    public Transform3d robotToFrontCam = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(12),    // 12 in forward
            Units.inchesToMeters(0),     // centered
            Units.inchesToMeters(8)),    // 8 in up
        new Rotation3d(
            0,                           // roll
            Units.degreesToRadians(-20), // pitch: negative = tilted up
            0));                         // yaw: facing forward

    public Transform3d robotToBackCam = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-12),   // 12 in behind
            Units.inchesToMeters(0),
            Units.inchesToMeters(8)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            Units.degreesToRadians(180))); // facing backward


 // ---------- Field + estimators ----------
    public AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public PhotonPoseEstimator frontEstimator =
        new PhotonPoseEstimator(fieldLayout, robotToFrontCam);

    public PhotonPoseEstimator backEstimator =
        new PhotonPoseEstimator(fieldLayout, robotToBackCam);

  public PosePhoton() {

  }
public void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator, String label) {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> estimate =
                estimator.estimateLowestAmbiguityPose(result);

            if (estimate.isPresent()) {
                Pose2d pose = estimate.get().estimatedPose.toPose2d();

                SmartDashboard.putNumber(label + " X (m)", pose.getX());
                SmartDashboard.putNumber(label + " Y (m)", pose.getY());
                SmartDashboard.putNumber(label + " Heading (deg)", pose.getRotation().getDegrees());
                SmartDashboard.putNumber(label + " Tag ID", result.getBestTarget().getFiducialId());
            }
        }
      }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    processCamera(frontCam, frontEstimator, "Front");
    processCamera(backCam, backEstimator, "Back");

  }
}
