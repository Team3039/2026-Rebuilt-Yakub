//nothing to see here

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.naming.spi.DirStateFactory.Result;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// imported InterpolatingTreeMap not required; removed to fix unused import
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
 
public class Hood extends SubsystemBase {

  // Create the possible states of the Hood
  public enum HoodState {
    IDLE,
    MANUAL,
    POSITION,
    TRACKING

  }

  // Create a variable to store the current state of the hood
  public HoodState hoodState = HoodState.IDLE;

  // Create a talonfx for the hood
  public TalonFX hoodMotor = new TalonFX(Constants.Ports.HOOD);

  // Create a PID Controller for the hood
  private PIDController controller = new PIDController(
      Constants.Hood.Hood_KP,
      Constants.Hood.Hood_KI,
      Constants.Hood.Hood_KD);

  {

  }

  // Create a variable to store the setpoint of the hood in kraken encoder
  // ticks
  public static double setpointHood = 0;

  public double getDistanceFromHub() {
    return RobotContainer.drivetrain.getDistanceToHub();
  }

  // Interpolating map for desired hood positions by distance
  private final InterpolatingDoubleTreeMap dissierdHoodPosition = new InterpolatingDoubleTreeMap();

  // Hood Constructor
  public Hood() {

    // Create a talonfx configurator.
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Soft Limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1.7;

    // Inverted and Neutral Modes
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configurator to the Hood motor
    hoodMotor.getConfigurator().apply(config);

    {

     
      
      dissierdHoodPosition.put(1.60, -0.0);
      dissierdHoodPosition.put(1.70, -0.25);
      dissierdHoodPosition.put(1.80, -0.35);
      dissierdHoodPosition.put(1.9, -0.45);
      dissierdHoodPosition.put(2.0, -0.65);
      dissierdHoodPosition.put(2.1, -0.75);
      dissierdHoodPosition.put(2.2, -0.85);
      dissierdHoodPosition.put(2.3, -0.95);
      dissierdHoodPosition.put(2.4, -1.05);      
      dissierdHoodPosition.put(2.5, -1.15);
      dissierdHoodPosition.put(2.5, -1.15);
      dissierdHoodPosition.put(2.6, -1.25);
      dissierdHoodPosition.put(2.7, -1.35);
      dissierdHoodPosition.put(2.8, -1.45);
      dissierdHoodPosition.put(2.9, -1.55);
      dissierdHoodPosition.put(3.0, -1.65);
      dissierdHoodPosition.put(3.1, -1.55);
      

    }

  }

  /**
   * Get the state of the hood
   * 
   * @return The state of the hood as an HoodState
   */
  public HoodState getState() {
    return hoodState;
  }

  /**
   * Set the state of the hood
   * 
   * @param state The state to set the hood to
   */
  public void setState(HoodState state) {
    hoodState = state;
  }

  public double getHoodPosition() {

    double position = hoodMotor.getPosition().getValueAsDouble() - 0.2255859375;

    return position;
  }

  public void setHoodPosition() {
    double pidOutput = controller.calculate(getHoodPosition(), setpointHood);

    double output = pidOutput;

    // Add feedforward (static friction / gravity compensation)
    if (Math.abs(pidOutput) > 0.001) {
      output += Math.copySign(Constants.Hood.Hood_KS, pidOutput);
    }

    // Telemetry before clamping
    SmartDashboard.putNumber("Hood pidOutput", pidOutput);
    SmartDashboard.putNumber("Hood outputBeforeClamp", output);

    // Clamp final output to safe range
    double finalOutput = MathUtil.clamp(output, -0.32, 0.32);

    // Telemetry after clamping
    SmartDashboard.putNumber("Hood finalOutput", finalOutput);

    hoodMotor.set(finalOutput);
  }

  /**
   * Set the output of the hood with feedForward
   * 
   * @param percent The percentage to set the hood to
   */
  public void setHoodPercent(double percent) {
    hoodMotor.set(percent + Constants.Hood.Hood_KS);
  }

  public void stop() {
    hoodMotor.set(0);
  }

  /**
   * Get the current setpoint of the Hood
   * 
   * @return the current setpoint of the Hood
   */
  public static double getSetpoint() {
    return setpointHood;
  }

  /**
   * Set the setpoint of the Hood
   * 
   * @param setpoint the setpoint to set the Hood to, in kraken encoder ticks
   */
  public static void setSetpoint(double setpoint) {
    setpointHood = setpoint;
  }

  /**
   * Check if the Hood is at the setpoint within a given tolerance
   * 
   * @param tolerance the tolerance to check if the Hood is at the setpoint
   * @return true if the hood is at the setpoint within the tolerance, false
   *         otherwise
   */
  public boolean isAtSetpoint(double tolerance) {
    return Math.abs((setpointHood - getHoodPosition())) <= tolerance;
  }

  @Override
  public void periodic() {

  double Distance = getDistanceFromHub();

  Double result = dissierdHoodPosition.get(Distance);

  SmartDashboard.putNumber("Hood Encoder", getHoodPosition());

    // SmartDashboard.putNumber("Target Rot to hub", getTargetRotToHub());

  SmartDashboard.putNumber("Hood Output", hoodMotor.get());
  SmartDashboard.putNumber("Hood error", Math.abs((setpointHood - getHoodPosition())));
    SmartDashboard.putNumber("Hood Output Current",hoodMotor.getSupplyCurrent().getValueAsDouble());
  SmartDashboard.putString("Hood State", String.valueOf(getState()));
  SmartDashboard.putBoolean("isAtSetpoint?", controller.atSetpoint());
  SmartDashboard.putNumber("dissierdHoodPosition", result);
  SmartDashboard.putNumber("hoodSetpoint", setpointHood);

    // Hood State Machine
    switch (hoodState) {

      // In the Idle state, the Hood rests at the bottom of the robot
      case IDLE:
        stop();
        break;

      // In the Manual state, the Hood is controlled directly by the operator
      case MANUAL:
        setHoodPercent(RobotContainer.guitar.getLeftY());
        break;

      // In the Position state, the Hood is controlled by the setpoint
      case POSITION:
        setHoodPosition();
        break;

      // TRACKING: update the setpoint from the interpolation table if available
      case TRACKING:
        setSetpoint(result);
        setHoodPosition();
        break;

    }
  }
}
