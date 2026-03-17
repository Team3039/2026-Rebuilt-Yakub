// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Indexer.IndexerState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setKickerBackPassive extends Command {
  /** Creates a new SetTurretManualOverride. */
  public setKickerBackPassive() {
    addRequirements(RobotContainer.flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.indexer.setState(IndexerState.BackPASSIVE);
    RobotContainer.hopper.setState(HopperState.BackPASSIVE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.hopper.setState(HopperState.IDLE);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}