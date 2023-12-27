
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
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMeters = encoder.getVelocity();
        inputs.appliedVolts = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
        inputs.currentAmps = new double[] {elevatorMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {elevatorMotor.getMotorTemperature()};
    }

    /** Run open loop at the specified voltage. */
    default void setVoltage(double motorVolts) {
        elevatorMotor.setVoltage(motorVolts);
    }

    /** Returns the distance measurement */
    default double getDistance() {
        return encoder.getPosition();
    }
    /** Sets PID Constants to manage speed */
    default void setPIDConstants(double p, double i, double d, double ff) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
    }

    /** Go to Setpoint */
    default void goToSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }
    /** Toggles brakes on or off to stop the elevator */
    default void setBrake(boolean brake) {
        elevatorMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }
    /** 
     * Determines whether the elevator is at the setpoint through
     * whether the encoder's reported position is within the tolerance
     * of the PID
     */
    default boolean atSetpoint() {
        return Math.abs(encoder.getPosition() - setpoint) < ELEVATOR_PID_TOLERANCE;
    }
    /** Sets the PID arguments used in setPIDConstants */
    default void setP(double p) {
        pidController.setP(p);
    }
    
    default void setI(double i) {
        pidController.setI(i);
    }

    default void setD(double d) {
        pidController.setD(d);
    }

    default void setFF(double ff) {
        pidController.setFF(ff);
    }
    /** Returns the PID arguments used in setPIDConstants */
    default double getP() { 
        return pidController.getP();
    }

    default double getI() {
        return pidController.getI();
    }

    default double getD() {
        return pidController.getD();
    }

    default double getFF() {
        return pidController.getFF();
    }

}
