
package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Elevator.ElevatorPhysicalConstants.*;

public class ElevatorIOSparkMax implements ElevatorIO {
    // Motor and Encoders
    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private DutyCycleEncoder absoluteEncoder;

    private double setpoint = 0;

    public ElevatorIOSparkMax() {
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        pidController = elevatorMotor.getPIDController();
        pidController.setOutputRange(-1, 1);

        encoder = elevatorMotor.getEncoder();
        encoder.setPositionConversionFactor(ELEVATOR_REV_TO_POS_FACTOR);
        encoder.setVelocityConversionFactor(ELEVATOR_REV_TO_POS_FACTOR / 60);
        encoder.setPosition(0.6);

        absoluteEncoder = new DutyCycleEncoder(0);
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);

        encoder.setPosition(absoluteEncoder.getDistance() * 28.45 + 0.6);
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ElevatorIOInputs inputs) {
        
    }

    /** Run open loop at the specified voltage. */
    default void setVoltage(double motorVolts) {
    }

    /** Returns the distance measurement */
    default double getDistance() {
        return 0.0;
    }
    /** Sets PID Constants to manage speed */
    default void setPIDConstants(double p, double i, double d, double ff) {
    }

    /** Go to Setpoint */
    default void goToSetpoint(double setpoint) {
    }
    /** Toggles brakes on or off to stop the elevator */
    default void setBrake(boolean brake) {
    }
    /** Sets the default setpoint boolean to false */
    default boolean atSetpoint() {
        return false;
    }
    /** Sets the PID arguments used in setPIDConstants */
    default void setP(double p) {

    }
    
    default void setI(double i) {

    }

    default void setD(double d) {

    }

    default void setFF(double ff) {
        
    }
    /** Returns the PID arguments used in setPIDConstants */
    default double getP() { return 0.0; }

    default double getI() { return 0.0; }

    default double getD() { return 0.0; }

    default double getFF() { return 0.0; }

}