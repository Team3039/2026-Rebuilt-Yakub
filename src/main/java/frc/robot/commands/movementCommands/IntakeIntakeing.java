// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movementCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActuateIntakeToSetpoint;
import frc.robot.commands.setIntakerollersIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeIntakeing extends SequentialCommandGroup {
  /** Creates a new ScoreCoralL2. */
  public IntakeIntakeing() {
    
    addCommands(
        new ActuateIntakeToSetpoint(4.9, .2),
        new setIntakerollersIntake()
          
    );
  }
}