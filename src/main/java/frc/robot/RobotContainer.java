// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.PuertoControl;
import frc.robot.commands.DriveWithConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final Drivetrain MecanumChassis = new Drivetrain();
  private final Intake intake = new Intake();
  private final SendableChooser<Command> autoChooserPathPlanner;
  private final CommandXboxController ControlCero =
      new CommandXboxController(PuertoControl.kDriverControllerPort);

  public RobotContainer() {
    MecanumChassis.setDefaultCommand(new DriveWithConstants(MecanumChassis,
      () -> ControlCero.getLeftY(),
      () -> ControlCero.getLeftX(),
      () -> ControlCero.getRightX(),
      () -> ControlCero.getHID().getPOV(),
      () -> ControlCero.rightBumper().getAsBoolean()
    ));

    NamedCommands.registerCommand(
      "Agarrar Pelotitas",
      intake.runEnd(
        intake::intakeWithValue,
        intake::stopRollers
      )
    );

    NamedCommands.registerCommand(
      "Escupir Pelotitas",
      intake.runEnd(
        intake::reverseWithValue,
        intake::stopRollers
      )
    );

    NamedCommands.registerCommand(
      "Detener Intake",
      intake.runOnce(
        intake::stopRollers
      )
    );

    autoChooserPathPlanner = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autónomos PathPlanner", autoChooserPathPlanner);

    configureBindings();
  }

  private void configureBindings() {

    ControlCero.leftBumper().onTrue(
      Commands.runOnce(
        () -> MecanumChassis.setFieldOriented(
          !MecanumChassis.isFieldOriented()
        ),
        MecanumChassis
      )
    );

    ControlCero.start().onTrue(
      Commands.runOnce(
        () -> MecanumChassis.resetNavx(),
        MecanumChassis
      )
    );

    ControlCero.x().whileTrue(
      intake.runEnd(
        intake::intakeWithValue,
        intake::stopRollers
      )
    );

    ControlCero.b().whileTrue(
      intake.runEnd(
        intake::reverseWithValue,
        intake::stopRollers)
    );

    ControlCero.back().onTrue(
      Commands.runOnce(
        () -> MecanumChassis.ResetALL(),
        MecanumChassis
      )
    );
  }

  public Command getAutonomousCommand() {
    return autoChooserPathPlanner.getSelected();
  }
}
