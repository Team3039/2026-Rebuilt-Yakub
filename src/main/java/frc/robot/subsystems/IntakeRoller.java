// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase {


  // Create the possible states of the claw
  public enum RollerState {
    IDLE,
    Intake,
    OutTake,
  }

  // Create a variable to store the current state of the claw
  RollerState clawState = RollerState.IDLE;

 

  // Create a talonfx for the claw
  TalonFX claw = new TalonFX(Constants.Ports.INTAKEROLLER);

  

  // Claw Constructor
  public IntakeRoller() {

    // Create a talonfx configurator.
    TalonFXConfiguration clawConfig = new TalonFXConfiguration();

    // Inverted and Neutral Modes
    clawConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    clawConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configurator to the claw motor
    claw.getConfigurator().apply(clawConfig);

  }
    // Create a CANrange configurator

  /**
   * Get the current state of the claw
   * 
   * @return the current state of the claw as a RollerState
   */
  public RollerState getState() {
    return clawState;
  }
  
  /**
   * Set the state of the claw
   * 
   * @param state the state to set the claw to
   */
  public void setState(RollerState state) {
    clawState = state;
  }

  /**
   * Set the speed of the claw
   * <p>
   * Positive values will intake algae, negative values will intake coral
   * 
   * @param speed the speed to set the claw to (-1 to 1)
   */
  public void setWheelSpeed(double speed) {
    claw.set(speed);
  }



  /** 
   * Check to see whether the intake has either gamepiece
   * 
   * @return true if the intake has either gamepiece, false otherwise
   */
  


  

  /**
   * Check to see if the claw is aligned with the branch.
   * It does this by checking the distance detected by the branchCANRange.
   * If it detects an object closer than 0.5 meters, it is likely the branch, and thus we are aligned.
   * 
   * @return true if the claw is aligned with the branch, false otherwise
   */
 





  @Override
  public void periodic() {
  


    // Claw State Machine
    switch (clawState) {

      // In the idle state, the claw does not intake, and it isnt deactivated
      case IDLE:
        setWheelSpeed(0);
       
        break;





    
      case Intake:
       
          setWheelSpeed(-0.7);
       

    
        case OutTake:
        // setWheelSpeed(-0.5);
       

      // In the passive state, the claw will not intake, and will deactivate the intake. 
      //  This will be used when the claw has a gamepiece
    }
  


    

  }
}