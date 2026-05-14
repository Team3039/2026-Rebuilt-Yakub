// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel.FlywheelState;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Indexer.IndexerState;
// import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretState;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StartLeftPassing extends Command {
  /** Creates a new SetTurretManualOverride. */
  public StartLeftPassing() {
    addRequirements(RobotContainer.flywheel);
    addRequirements(RobotContainer.turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.flywheel.setState(FlywheelState.PASSING);
    RobotContainer.indexer.setState(IndexerState.PASSING);
    RobotContainer.hopper.setState(HopperState.PASSIVE);
    RobotContainer.intake.setState(IntakeState.PASSIVE_UP);
    RobotContainer.turret.setState(TurretState.RIGHTPASSING);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        RobotContainer.intake.setState(IntakeState.PASSIVE_UP);


    }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.flywheel.setState(FlywheelState.IDLE);
    RobotContainer.turret.setState(TurretState.HUBTRACKING);
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.hopper.setState(HopperState.IDLE);
    RobotContainer.intake.setState(IntakeState.IDLE);




  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}