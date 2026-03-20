//nothing to see here

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {

  // Create the possible states of the Intake
  public enum IntakeState {
    IDLE,
    MANUAL,
    POSITION,
    TRACKING,
    PASSIVE_UP

  }



  // Create a variable to store the current state of the intake
  public IntakeState intakeState = IntakeState.IDLE;

  // Create a talonfx for the intake
  public TalonFX Intake = new TalonFX(Constants.Ports.INTAKE);

  // Create a PID Controller for the intake
  private PIDController controller = new PIDController(
      Constants.Intake.Intake_KP,
      Constants.Intake.Intake_KI,
      Constants.Intake.Intake_KD);

  {

  }


  // Create a variable to store the setpoint of the Intake in kraken encoder
  // ticks
  public static double setpointIntake  = 0;

  // Intake Constructor
  public Intake() {

    // Create a talonfx configurator.
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Soft Limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 4.9;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = .20;


    // Inverted and Neutral Modes
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configurator to the Intake motor
    Intake.getConfigurator().apply(config);
  }

  /**
   * Get the state of the intake
   * 
   * @return The state of the intake as an IntakeState
   */
  public IntakeState getState() {
    return intakeState;
  }

  /**
   * Set the state of the intake
   * 
   * @param state The state to set the intake to
   */
  public void setState(IntakeState state) {
    intakeState = state;
  }
  

  

  public double getIntakePosition() {

  double position = Intake.getPosition().getValueAsDouble();

    return position;
  }
  // Constants.intakeGearRatio


  public void setIntakePosition() {
  double pidOutput = controller.calculate(getIntakePosition(), setpointIntake);

  double output = pidOutput;

  if (Math.abs(pidOutput) > 0.001) {
    output += Math.copySign(Constants.Intake.Intake_KS, pidOutput);
  }

  output = MathUtil.clamp(output, -.8, 0.2);

  Intake.set(output);
}


  /**
   * Set the output of the Intake with feedforward
   * 
   * @param percent The percentage to set the intake to
   */
  public void setIntakePercent(double percent) {
  if (Math.abs(percent) > 0.001) {
    percent += Math.copySign(Constants.Intake.Intake_KS, percent);
  }
  Intake.set(percent);
}

  public void stop() {
    Intake.set(0);
  }

  /**
   * Get the current position of the intake
   * 
   * @return the current angle of the intake in kraken ticks
   */
  

  /**
   * Get the current setpoint of the Intake
   * 
   * @return the current setpoint of the Intake
   */
  public static double getSetpoint() {
    return setpointIntake;
  }

  /**
   * Set the setpoint of the Intake
   * 
   * @param setpoint the setpoint to set the Intake to, in kraken encoder ticks
   */
  public static void setSetpoint(double setpoint) {
    setpointIntake = setpoint;
  }

  
  /**
   * Check if the Intake is at the setpoint within a given tolerance
   * 
   * @param tolerance the tolerance to check if the Intake is at the setpoint
   * @return true if the wrist is at the setpoint within the tolerance, false
   *         otherwise
   */
  public boolean isAtSetpoint(double tolerance) {
    return Math.abs((setpointIntake - getIntakePosition())) <= tolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Encoder", getIntakePosition());
    
    // SmartDashboard.putNumber("Target Rot to hub", getTargetRotToHub());

    SmartDashboard.putNumber("Intake Output", Intake.get());
    SmartDashboard.putNumber("Intake error", Math.abs((setpointIntake - getIntakePosition() )));
    
    SmartDashboard.putNumber("Intake Setpoint", (getSetpoint() ));

    // SmartDashboard.putNumber("Intake Output Current",
    // Intake.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("Intake State", String.valueOf(getState()));
  // SmartDashboard.putBoolean("isAtSetpoint?", controller.atSetpoint());
  

    // Intake State Machine
    switch (intakeState) {

      // In the Idle state, the Intake rests at the bottom of the robot
      case IDLE:
        stop();
        break;

      // In the Manual state, the Intake is controlled directly by the operator
      case MANUAL:
        setIntakePercent(RobotContainer.driverPad.getLeftY() * 0.1);
        break;


        case PASSIVE_UP:
        setIntakePercent(-.1);
        break;

      // In the Position state, the Intake is controlled by the setpoint
      case POSITION:
        setIntakePosition();
        break;

      
    }
  }
}