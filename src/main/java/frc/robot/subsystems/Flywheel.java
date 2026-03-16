package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Flywheel extends SubsystemBase {

    public static double targetVelocity = 10; // RPS

    public enum FlywheelState {
        IDLE,
        SHOOTING,
        MANUAL,
        TRACKING
    }

    private final static TalonFX shooterLeft =  new TalonFX(Constants.Ports.FLYWHEEL_LEFT);
    
        private final TalonFX shooterRight = new TalonFX(Constants.Ports.FLYWHEEL_RIGHT);
    
        private FlywheelState flywheelState = FlywheelState.IDLE;
    
        private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    
        public Flywheel() {
    
            TalonFXConfiguration config = new TalonFXConfiguration();
    
            // Current Limits
            config.CurrentLimits.StatorCurrentLimit = 70;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
    
            // pid shi
            config.Slot0.kP = Constants.Flywheel.Flywheel_KP;
            config.Slot0.kI = Constants.Flywheel.Flywheel_KI;
            config.Slot0.kD = Constants.Flywheel.Flywheel_KD;
            config.Slot0.kV = Constants.Flywheel.Flywheel_FF;
    
            shooterLeft.getConfigurator().apply(config);
            shooterRight.getConfigurator().apply(config);
    
            shooterLeft.setNeutralMode(NeutralModeValue.Coast);
            shooterRight.setNeutralMode(NeutralModeValue.Coast);
        }
            
    
        public void setState(FlywheelState state) {
            flywheelState = state;
        }
    
        public FlywheelState getState() {
            return flywheelState;
        }
    
    
        
    
        public void setShooterVelocity(double rps) {
            shooterLeft.setControl(
                    velocityRequest.withVelocity(rps)
            );
    
            shooterRight.setControl(
                    velocityRequest.withVelocity(-rps)
            );
        }
    
        public void stop() {
            shooterLeft.setControl(velocityRequest.withVelocity(0));
            shooterRight.setControl(velocityRequest.withVelocity(0));
        }
    
        public static boolean isAtVelocitySetpoint() {
            return shooterLeft.getRotorVelocity().getValueAsDouble() >= targetVelocity;
    }

    public double getDistanceFromHub() {
        return Swerve.getDistanceToHub();
    }



    // private final InterpolatingDoubleTreeMap dissierdShooterSpeed = new InterpolatingDoubleTreeMap();
    // {
    //     dissierdShooterSpeed.put(3., 4.7); 
    //     dissierdShooterSpeed.put(1.5, 4.5);
    //     dissierdShooterSpeed.put(1.4, 4.0);
    //     dissierdShooterSpeed.put(4.1, 4.7); 
    // }

     private final InterpolatingDoubleTreeMap dissierdShooterSpeedv2 = new InterpolatingDoubleTreeMap();
    {
        dissierdShooterSpeedv2.put(1.80, 4.44); 
        dissierdShooterSpeedv2.put(1.70, 4.43); 
        dissierdShooterSpeedv2.put(1.60, 4.31); 
        
        dissierdShooterSpeedv2.put(2.9, 4.4); 
        dissierdShooterSpeedv2.put(3.1, 5.0); 
        dissierdShooterSpeedv2.put(4.0, 5.2); 
        dissierdShooterSpeedv2.put(5.6, 5.7); 


         
    }






    @Override
    public void periodic() {

                         
                double Distance = getDistanceFromHub();

                double result = dissierdShooterSpeedv2.get(Distance);

        double currentVelocity = shooterLeft.getRotorVelocity().getValueAsDouble();

        SmartDashboard.putNumber("Shooter RPS", currentVelocity);
        SmartDashboard.putNumber("dissierdShooterSpeed RPS", dissierdShooterSpeedv2.get(Distance));
        SmartDashboard.putBoolean("Shooter At Setpoint", isAtVelocitySetpoint());
        SmartDashboard.putString("Flywheel State", flywheelState.name());
		



        switch (flywheelState) {


            case IDLE:

                setShooterVelocity(.5);

                break;

            case SHOOTING:
                setShooterVelocity(5);
                break;


                case TRACKING:

              
                // double Distance = getDistanceFromHub();

                // double result = dissierdShooterSpeed.get(Distance); 

                setShooterVelocity(result);
                break;



			case MANUAL:
                setShooterVelocity(RobotContainer.driverPad.getLeftY() );
                break;
        }
    }
}