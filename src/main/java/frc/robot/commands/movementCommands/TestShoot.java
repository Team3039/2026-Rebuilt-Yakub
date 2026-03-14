// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.movementCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActuateHoodToSetpoint;
import frc.robot.commands.setFlyWheels;
import frc.robot.commands.setHopperPassive;
import frc.robot.commands.setIntakerollersIntake;
import frc.robot.commands.setKickerPassive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestShoot extends SequentialCommandGroup {

  public TestShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new setFlyWheels(),    
    new setKickerPassive(),
    new setHopperPassive(),
    new setIntakerollersIntake() 
    );
  }
}