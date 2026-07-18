// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Mecanum;
import frc.robot.Constants.PIDFFs;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAim_PassRight extends Command {
  private final Drivetrain mecanum;
  private final DoubleSupplier moveChassis;
  private final DoubleSupplier strafeChassis;

  private final Translation2d blueHUB = new Translation2d(4.620, 4.032);
  private final Translation2d redHUB = new Translation2d(4.032, 4.032);

  private final PIDController ChassisPID = new PIDController(
    PIDFFs.kP_chassis,
    PIDFFs.kI_chassis,
    PIDFFs.kD_chassis
  );

  public AutoAim_PassRight(
    Drivetrain chassis,
    DoubleSupplier MoveInX,
    DoubleSupplier MoveInY
  ){
    this.mecanum = chassis;
    this.moveChassis = MoveInX;
    this.strafeChassis = MoveInY;

    //Ajustes PID Rotacion (code snippet took from DriveWithConstants)
    ChassisPID.enableContinuousInput(-(Mecanum.kContinuousInput), Mecanum.kContinuousInput);
    ChassisPID.setIntegratorRange(-(Mecanum.kIntegratorRange), Mecanum.kIntegratorRange);
    ChassisPID.setTolerance(Mecanum.kErrorToleranceChasis);
    ChassisPID.isContinuousInputEnabled();

    addRequirements(mecanum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var alliance = DriverStation.getAlliance();
    boolean isBlue = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;

    Translation2d myHUB = isBlue ? blueHUB : redHUB;
    Pose2d currentPose = mecanum.getPose2d();

    Rotation2d AngleTarget = myHUB.minus(currentPose.getTranslation()).getAngle();
    double RotateToHub = ChassisPID.calculate(currentPose.getRotation().getDegrees(), AngleTarget.getDegrees());

    double vxMPS = moveChassis.getAsDouble() * -(Mecanum.kMaxSpeedWheel);
    double vyMPS = strafeChassis.getAsDouble() * -(Mecanum.kMaxSpeedWheel);


    ChassisSpeeds mecanumSpeeds;

    //Toggle para modo Robot Centric
    if (mecanum.isFieldOriented() || !(DriverStation.isAutonomous())){  
      mecanumSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMPS/4, vyMPS/4, RotateToHub*4.5, mecanum.getHeading());
    } else {
      mecanumSpeeds = new ChassisSpeeds(vxMPS/4, vyMPS/4, RotateToHub*4.5);
    }

    mecanum.driveWithSpeeds(mecanumSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mecanum.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
