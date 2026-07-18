// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PhotonVision extends SubsystemBase {
  private final PhotonCamera RedMagic;
  private final PhotonPoseEstimator not_LiDAR;
  private AprilTagFieldLayout fieldLayout;
  private Optional<EstimatedRobotPose> visionPose = Optional.empty();
  public PhotonVision() {

    RedMagic = new PhotonCamera("DroidCam_Video");
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    

    Transform3d RobotToPhoneCam = new Transform3d(
      new Translation3d(0.394,0.019,0.107), 
      new Rotation3d(0, Radians.convertFrom(-40, Degrees),0)
    );

    not_LiDAR = new PhotonPoseEstimator(fieldLayout, RobotToPhoneCam);
    
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = RedMagic.getLatestResult();

    // ¿Hay target?
    SmartDashboard.putBoolean("PV/Has Target", result.hasTargets());

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();

      // Get information from target.
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();

      SmartDashboard.putNumber("PV/Yaw", target.getYaw());
      SmartDashboard.putNumber("PV/Pitch", target.getPitch());
      SmartDashboard.putNumber("PV/Area", target.getArea());
      SmartDashboard.putNumber("PV/Target ID", target.getFiducialId());

    } else {
      // Valores por defecto cuando no hay target
      SmartDashboard.putNumber("PV/Yaw", 0.0);
      SmartDashboard.putNumber("PV/Pitch", 0.0);
      SmartDashboard.putNumber("PV/Area", 0.0);
      SmartDashboard.putNumber("PV/Target ID", -1);
    }

    // Latencia total del pipeline (ms) - unavailable in this PhotonVision version, show 0.0 as fallback
    SmartDashboard.putNumber(
      "PV/Latency (ms)",
      0.0
    );

    visionPose = not_LiDAR.estimateCoprocMultiTagPose(result);
    if (visionPose.isEmpty()) {
      visionPose = not_LiDAR.estimateLowestAmbiguityPose(result);
    }
  }

  public Optional<EstimatedRobotPose> getVisionEstimatedLocation(){
    return visionPose;
  }
}
