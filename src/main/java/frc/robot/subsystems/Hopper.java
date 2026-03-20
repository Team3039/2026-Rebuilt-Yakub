// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem responsible for the Hopper / Indexer
 */
public class Hopper extends SubsystemBase {

    // Possible states of the hopper / indexer
    public enum HopperState {
        IDLE,      
        PASSIVE,  
        Intakeing,     
        Shooting,    
        BackPASSIVE 
    }

    // Variable that stores the current hopper state
    HopperState hopperState = HopperState.IDLE;

    // Whether a game piece (fuel / coral) is currently in the hopper
    public boolean hasFuel = false;

    // Hopper TalonFX motor
    TalonFX hopper = new TalonFX(Constants.Ports.HOPPER);

    // CANrange sensor for detecting coral in the hopper (currently disabled)
    // CANrange INDEXERCANRANGE = new CANrange(Constants.Ports.INDEXERCANRANGE);

    /**
     * Hopper subsystem constructor
     */
    public Hopper() {
        // Create motor configuration object
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Set motor direction and neutral behavior
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configuration to the motor
        hopper.getConfigurator().apply(config);
    }

    /**
     * Returns the current hopper state
     *
     * @return the current HopperState
     */
    public HopperState getState() {
        return hopperState;
    }

    /**
     * Changes the hopper state
     *
     * @param state the new state
     */
    public void setState(HopperState state) {
        hopperState = state;
    }

    /**
     * Sets the hopper motor speed
     * <p>
     * Positive values → intake algae<br>
     * Negative values → intake coral
     *
     * @param speed value between -1.0 and 1.0
     */
    public void setHopperSpeed(double speed) {
        hopper.set(speed);
    }

    /**
     * Checks whether a game piece exists in the hopper
     *
     * @return true if a game piece exists, otherwise false
     */
    public boolean hasGamepiece() {
        return hasFuel;
    }

    // Example future sensor logic (currently disabled)
    // public boolean isFuelIn() {
    //     return INDEXERCANRANGE.getDistance().getValueAsDouble() < 0.15;
    // }

    @Override
    public void periodic() {
        // Update SmartDashboard
        SmartDashboard.putNumber("Hopper Current (Amps)", hopper.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putString("Hopper State", hopperState.toString());
        // SmartDashboard.putBoolean("Fuel Present", isFuelIn());

        // Hopper state machine
        switch (hopperState) {
            case IDLE:
                setHopperSpeed(0);
                break;

            case Intakeing:
               
                if (hopper.getSupplyCurrent().getValueAsDouble() > 39) {
                    setHopperSpeed(0.5);  
                } else {
                    setHopperSpeed(-0.5);  
                }
                break;

            case Shooting:

              setHopperSpeed(-0.5);  

                break;

            case PASSIVE:

                if (hopper.getSupplyCurrent().getValueAsDouble() > 40) {
                    setHopperSpeed(1);  
                } else {
                    setHopperSpeed(-.5);  
                }
                break;


                case BackPASSIVE:

                    setHopperSpeed(-0.5);  
                
                break;
        }
    }
}