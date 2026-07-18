// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.PIDFFs;
import frc.robot.Constants.Mecanum;

import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveWithConstants extends Command {
  private final Drivetrain chassis;
  private final DoubleSupplier moveChassis;
  private final DoubleSupplier strafeChassis;
  private final DoubleSupplier rotateChassis;
  private final IntSupplier getPOV;
  private final BooleanSupplier isSpeedReduced;

  private double xRC_SlowMode;
  private double LastAngle;

  //Slews
  private final SlewRateLimiter SlewMOVE = new SlewRateLimiter(8);
  private final SlewRateLimiter SlewSTRAFE = new SlewRateLimiter(4);
  private final SlewRateLimiter SlewROTATE = new SlewRateLimiter(8);

  private final PIDController ChassisPID = new PIDController(
    PIDFFs.kP_chassis,
    PIDFFs.kI_chassis,
    PIDFFs.kD_chassis
  );

  private double AngleTarget;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public DriveWithConstants(
    Drivetrain chasisMecanum, 
    DoubleSupplier xSpeed, 
    DoubleSupplier ySpeed, 
    DoubleSupplier zRotation,
    IntSupplier AnglePOV,
    BooleanSupplier ToggleSlowmode){

    this.chassis = chasisMecanum;
    this.moveChassis = xSpeed;
    this.strafeChassis = ySpeed;
    this.rotateChassis = zRotation;
    this.getPOV = AnglePOV;
    this.isSpeedReduced = ToggleSlowmode;

    //Ajustes PID Rotacion (code snippet took from Bea's code)
    ChassisPID.enableContinuousInput(-(Mecanum.kContinuousInput), Mecanum.kContinuousInput);
    ChassisPID.setIntegratorRange(-(Mecanum.kIntegratorRange), Mecanum.kIntegratorRange);
    ChassisPID.setTolerance(Mecanum.kErrorToleranceChasis);
    ChassisPID.isContinuousInputEnabled();

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AngleTarget = chassis.getHeadingDegrees();
    xRC_SlowMode = 1.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xRC_SlowMode = isSpeedReduced.getAsBoolean() ? 0.4 : 1.0;

    //Datos RAW para calculos
    double dMove = -(moveChassis.getAsDouble());
    double dStrafe = (strafeChassis.getAsDouble());
    double dRot = (rotateChassis.getAsDouble());

    double limMove = Math.abs(dMove) > Mecanum.kDeadband ? dMove : 0.0;
    double limStrafe = Math.abs(dStrafe) > Mecanum.kDeadband ? (dStrafe * xRC_SlowMode) : 0.0;
    double limRot = Math.abs(dRot) > Mecanum.kDeadband ? dRot : 0.0;

    //Asignamos los angulos de la cruceta
    double LockInPOV = getPOV.getAsInt();
    double MecanumRotacionPID = 0.0;

    //Los datos buenos para el control
    double MecanumMove = SlewMOVE.calculate(limMove);
    double MecanumStrafe = SlewSTRAFE.calculate(limStrafe);
    double MecanumRotacionRAW = SlewROTATE.calculate(limRot);

    //Rotacion normal actualizando setpoint
    if (Math.abs(MecanumRotacionRAW) > 0.02 && LockInPOV == -1){
      AngleTarget = chassis.getHeadingDegrees();
      MecanumRotacionPID = MecanumRotacionRAW * xRC_SlowMode;
    }
    // Rotacion AutoAIM con cruceta
    else if (LockInPOV != -1){
      AngleTarget = -LockInPOV; 
      //Obtenemos angulo coterminal, mate 2 TecMi!
      if (AngleTarget < -180){
        AngleTarget = AngleTarget + 360;
      }
      MecanumRotacionPID = -(ChassisPID.calculate(chassis.getHeadingDegrees(), AngleTarget));
    }
    else
    { //Counter rotacion con PID
      MecanumRotacionPID = -(ChassisPID.calculate(chassis.getHeadingDegrees(), AngleTarget));
    }

    double vxMPS = MecanumMove * Mecanum.kMaxSpeedWheel;
    double vyMPS = MecanumStrafe * -(Mecanum.kMaxSpeedWheel);
    double omRPS = MecanumRotacionPID * -4.5;

    ChassisSpeeds mecanumSpeeds;

    //Toggle para modo Robot Centric
    if (chassis.isFieldOriented()){  
      mecanumSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMPS, vyMPS, omRPS, chassis.getHeading());
    } else {
      mecanumSpeeds = new ChassisSpeeds(vxMPS,vyMPS,omRPS);
    }

    chassis.driveWithSpeeds(mecanumSpeeds);

    //Storage cruceta para gyro falso de dashboard
    if (LockInPOV != -1){
      LastAngle = LockInPOV;
    }

    //Angulo actual y meta
    SmartDashboard.putNumber("AngulosChasis/AngleTarget", AngleTarget);
    SmartDashboard.putNumber("AutoAim/Value", LastAngle);
    SmartDashboard.putString("AutoAim/.type", "Gyro");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
