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
import frc.robot.TunerConstants;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  // Create the possible states of the indexer
  public enum IndexerState {
    IDLE,
    PASSIVE,
    Intakeing,
    Shooting,
    BackPASSIVE,
  }

  // Create a variable to store the current state of the indexer
  IndexerState indexerState = IndexerState.IDLE;

  // Keep track of whether or not the intake has a coral
  public boolean hasFuel = false;

  // Create a talonfx for the indexer
  TalonFX indexer = new TalonFX(Constants.Ports.INDEXER);
  TalonFX kicker = new TalonFX(Constants.Ports.KICKER);

  // This CANrange is used to detect coral in the intake
  CANrange INDEXERCANRANGE = new CANrange(Constants.Ports.INDEXERCANRANGE);

  // Indexer Constructor
  public Indexer() {

    // Create a talonfx configurator.
    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

    // Inverted and Neutral Modes
    indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configurator to the indexer motor
    indexer.getConfigurator().apply(indexerConfig);

  }
  // Create a CANrange configurator

  /**
   * Get the current state of the indexer
   * 
   * @return the current state of the indexer as a IndexerState
   */
  public IndexerState getState() {
    return indexerState;
  }

  /**
   * Set the state of the indexer
   * 
   * @param state the state to set the indexer to
   */
  public void setState(IndexerState state) {
    indexerState = state;
  }

  /**
   * Set the speed of the indexer
   * <p>
   * Positive values will intake algae, negative values will intake coral
   * 
   * @param speed the speed to set the indexer to (-1 to 1)
   */

  public void setkickerSpeed(double speed) {
    kicker.set(speed);
  }

  /**
   * Check to see whether the intake has either gamepiece
   * 
   * @return true if the intake has either gamepiece, false otherwise
   */
  public boolean hasGamepiece() {
    return hasFuel;
  }

  public boolean isFuelIn() {
    return INDEXERCANRANGE.getDistance().getValueAsDouble() < 0.15;
  }

  /**
   * Check to see if the indexer is aligned with the branch.
   * It does this by checking the distance detected by the branchCANRange.
   * If it detects an object closer than 0.5 meters, it is likely the branch, and
   * thus we are aligned.
   * 
   * @return true if the indexer is aligned with the branch, false otherwise
   */

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("CanRange Distance Detected",
    // branchCANRange.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Indexer Current", kicker.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("Indexer Status", String.valueOf(kicker.getSupplyCurrent().getValueAsDouble()));
    SmartDashboard.putBoolean("Has Fuel", isFuelIn());

    // Indexer State Machine
    switch (indexerState) {

      // In the idle state, the indexer does not intake, and it isnt deactivated
      case IDLE:

        setkickerSpeed(0);

        break;

      case Intakeing:
        if (isFuelIn()) {
          setkickerSpeed(0);
        } else if (!hasGamepiece()) {
          setkickerSpeed(0.3);
        }
        break;

      case Shooting:
        setkickerSpeed(0.3);
        break;

      // In the passive state, the indexer will not intake, and will deactivate the
      // intake.
      // This will be used when the indexer has a gamepiece
      case PASSIVE:

        if (Flywheel.isAtVelocitySetpoint() == true) {
          setkickerSpeed(1);
        } else {
          setkickerSpeed(.0);
        }

        break;

      case BackPASSIVE:

        setkickerSpeed(-1);

        break;
    }

  }
}