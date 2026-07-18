// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CorrectPoseWithVision extends Command {
  private final Drivetrain chassis;
  private final PhotonVision vision;
  public CorrectPoseWithVision(PhotonVision vision, Drivetrain chasis) {
    this.chassis = chasis;
    this.vision = vision;

    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 
    Optional<EstimatedRobotPose> visionEstimated = vision.getVisionEstimatedLocation();
    if (visionEstimated.isPresent()){
      EstimatedRobotPose visionReportedPose = visionEstimated.get();
      Pose2d visionPose2d = visionReportedPose.estimatedPose.toPose2d();
      double visionTimestamp = visionReportedPose.timestampSeconds;
      chassis.addVisionMeasurement(visionPose2d, visionTimestamp);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
