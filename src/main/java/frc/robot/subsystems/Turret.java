//nothing to see here

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {

	// Create the possible states of the Turret
	public enum TurretState {
		IDLE,
		MANUAL,
		POSITION,
		HUBTRACKING,
		RIGHTPASSING,
		LEFTPASSING,
		PASSING

	}

	// Create a variable to store the current state of the turret
	public TurretState turretState = TurretState.IDLE;

	// Create a talonfx for the turret
	public static TalonFX Turret = new TalonFX(Constants.Ports.TURRET);
	
		// Create a PID Controller for the turret
		private PIDController controller = new PIDController(
				Constants.Turret.Turret_KP,
				Constants.Turret.Turret_KI,
				Constants.Turret.Turret_KD);
	
		{
	
		}
	
		// Create a variable to store the setpoint of the Turret in kraken encoder
		// ticks
		public static double setpointTurret = 0;
	
		public static double FaceRed = 0;
		public static double FaceBlue = 180;
	
		// Turret Constructor
		public Turret() {
	
			// Create a talonfx configurator.
			TalonFXConfiguration config = new TalonFXConfiguration();
	
			config.CurrentLimits.SupplyCurrentLimit = 20;
			config.CurrentLimits.SupplyCurrentLimitEnable = true;
	
			config.CurrentLimits.StatorCurrentLimit = 120;
			config.CurrentLimits.StatorCurrentLimitEnable = true;
	
			// Soft Limits
			config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
			config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 3.0;
			config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -3.0;
	
			// Inverted and Neutral Modes
			// config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
	
			// Apply the configurator to the Turret motor
			Turret.getConfigurator().apply(config);
		}
	
		/**
		 * Get the state of the turret
		 * 
		 * @return The state of the turret as an TurretState
		 */
		public TurretState getState() {
			return turretState;
		}
	
		/**
		 * Set the state of the turret
		 * 
		 * @param state The state to set the turret to
		 */
		public void setState(TurretState state) {
			turretState = state;
		}
	
		public double getTargetRotToHub() {
	
			double target = RobotContainer.drivetrain.getRotationToHub();
	
			return target * -1;
	
		}
	
		public double getTargetRotToRightPass() {
			
			double target = RobotContainer.drivetrain.getRotationToRightPassing();
	
			return target * -1;
	
		}
	
		public double getTargetRotToLeftPass() {
			
			double target = RobotContainer.drivetrain.getRotationToLeftPassing();
	
			return target * -1;
	
		}
	
		public static double getTurretPosition() {
		
				double position = Turret.getPosition().getValueAsDouble() + 0.27001953125; // I looooooove magic numbers, what
																						// does this number mean? I dont
																						// know, but it makes everything
																						// work, so im not gonna change it.
	
			return position * Constants.turretGearRatio;
		}
		// Constants.turretGearRatio
	
		public void setTurretPosition() {
			double pidOutput = controller.calculate(getTurretPosition(), setpointTurret);
	
			double output = pidOutput;
	
			if (Math.abs(pidOutput) > 0.001) {
				output += Math.copySign(Constants.Turret.Turret_KS, pidOutput);
			}
	
			output = MathUtil.clamp(output, -0.1, 0.1);
	
			Turret.set(output);
		}
	
		/**
		 * Set the output of the Turret with feedforward
		 * 
		 * @param percent The percentage to set the turret to
		 */
		public void setTurretPercent(double percent) {
			Turret.set(percent + Constants.Turret.Turret_KS);
		}
	
		public void stop() {
			Turret.set(0);
		}
	
		/**
		 * Get the current position of the turret
		 * 
		 * @return the current angle of the turret in kraken ticks
		 */
	
		/**
		 * Get the current setpoint of the Turret
		 * 
		 * @return the current setpoint of the Turret
		 */
		public static double getSetpoint() {
			return setpointTurret;
		}
	
		/**
		 * Set the setpoint of the Turret
		 * 
		 * @param setpoint the setpoint to set the Turret to, in kraken encoder ticks
		 */
		public static void setSetpoint(double setpoint) {
			setpointTurret = setpoint;
		}
	
		/**
		 * Check if the Turret is at the setpoint within a given tolerance
		 * 
		 * @param tolerance the tolerance to check if the Turret is at the setpoint
		 * @return true if the wrist is at the setpoint within the tolerance, false
		 *         otherwise
		 */
		public static boolean isAtSetpoint(double tolerance) {
			return Math.abs((setpointTurret - getTurretPosition())) <= tolerance;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Turret Encoder", getTurretPosition());

		// SmartDashboard.putNumber("Target Rot to hub", getTargetRotToHub());

		SmartDashboard.putNumber("Turret Output", Turret.get());
		SmartDashboard.putNumber("Turret error", Math.abs((setpointTurret - getTurretPosition())));

		SmartDashboard.putNumber("Turret Setpoint", (getSetpoint()));

		// SmartDashboard.putNumber("Turret Output Current",
		// Turret.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putString("Turret State", String.valueOf(getState()));
		SmartDashboard.putNumber("getTargetRotToHub", getTargetRotToHub());

		// Turret State Machine
		switch (turretState) {

			// In the Idle state, the Turret rests at the bottom of the robot
			case IDLE:
				stop();

				break;

			// In the Manual state, the Turret is controlled directly by the operator
			case MANUAL:
				setTurretPercent(RobotContainer.driverPad.getLeftY() * 0.3);
				break;

			// In the Position state, the Turret is controlled by the setpoint
			case POSITION:
				setTurretPosition();
				break;

			case HUBTRACKING:

				setSetpoint(getTargetRotToHub() + Swerve.getPose().getRotation().getDegrees());
				setTurretPosition();

				break;

			case RIGHTPASSING:

				setSetpoint(getTargetRotToRightPass() + Swerve.getPose().getRotation().getDegrees());
				setTurretPosition();

				break;

			case LEFTPASSING:

				setSetpoint(getTargetRotToLeftPass() + Swerve.getPose().getRotation().getDegrees());
				setTurretPosition();

				break;
		}
	}
}
