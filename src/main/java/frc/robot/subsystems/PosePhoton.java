// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

public class PosePhoton extends SubsystemBase {
  /** Creates a new PosePhoton. */
 // ---------- Cameras (names must match PhotonVision dashboard) ----------
    public PhotonCamera frontCam = new PhotonCamera("cam1");
    public PhotonCamera backCam  = new PhotonCamera("cam2");
    public PhotonCamera leftCam  = new PhotonCamera("cam3");
    public PhotonCamera rightCam = new PhotonCamera("cam4");

    public Pose2d pose = new Pose2d();

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

    public Transform3d robotToRightCam = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(0),     // centered
            Units.inchesToMeters(-12),   // 12 in left
            Units.inchesToMeters(8)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            Units.degreesToRadians(-90))); // facing left

    public Transform3d robotToLeftCam = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(0),     // centered
            Units.inchesToMeters(12),    // 12 in right
            Units.inchesToMeters(8)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            Units.degreesToRadians(90))); // facing right

 // ---------- Field + estimators ----------
    public AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public PhotonPoseEstimator frontEstimator =
        new PhotonPoseEstimator(fieldLayout, robotToFrontCam);

    public PhotonPoseEstimator backEstimator =
        new PhotonPoseEstimator(fieldLayout, robotToBackCam);

    public PhotonPoseEstimator leftEstimator =
        new PhotonPoseEstimator(fieldLayout, robotToLeftCam);

    public PhotonPoseEstimator rightEstimator =
        new PhotonPoseEstimator(fieldLayout, robotToRightCam);

    public PosePhoton() {
        SmartDashboard.putData("Field", Telemetry.m_field);
    }

    public void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator, String label) {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) continue;

            // Prefer multi-tag, fall back to single tag
            Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = estimator.estimateLowestAmbiguityPose(result);
            }
            if (estimate.isEmpty()) continue;

            EstimatedRobotPose est = estimate.get();
            Pose2d visionPose = est.estimatedPose.toPose2d();

            // Reject poses outside the field
            if (visionPose.getX() < 0 || visionPose.getX() > 17.6
            || visionPose.getY() < 0 || visionPose.getY() > 8.1) continue;

            // Trust multi-tag more than single-tag
            int tags = est.targetsUsed.size();
            var stdDevs = tags > 1
                ? VecBuilder.fill(0.5, 0.5, 1.0)
                : VecBuilder.fill(1.5, 1.5, 4.0);

            RobotContainer.drivetrain.addVisionMeasurement(
                visionPose,
                Utils.fpgaToCurrentTime(est.timestampSeconds),
                stdDevs);

            pose = visionPose;   // assigns the FIELD (no "Pose2d" in front)

            SmartDashboard.putNumber(label + " X (m)", visionPose.getX());
            SmartDashboard.putNumber(label + " Y (m)", visionPose.getY());
            SmartDashboard.putNumber(label + " Heading (deg)", visionPose.getRotation().getDegrees());
            SmartDashboard.putNumber(label + " Tag ID", result.getBestTarget().getFiducialId());
        }
    }

    @Override
    public void periodic() {
        processCamera(frontCam, frontEstimator, "Front");
        processCamera(backCam,  backEstimator,  "Back");
        processCamera(leftCam,  leftEstimator,  "Left");
        processCamera(rightCam, rightEstimator, "Right");

        // Separate field object so it doesn't fight your drivetrain's own robot pose
        Telemetry.m_field.getObject("Vision").setPose(pose);
    }
}