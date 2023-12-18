package frc.robot.subsystems.drive.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * Assigning as an interface so that we can implement it
 * in other classes and use it as a template
*/ 
public interface ElevatorIO {
    /** Setting up AutoLog */
    @AutoLog
    /** Setting up the Inputs class and assigning default values */
    static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMeters = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    public double ELEVATOR_MAX_HEIGHT = 36.0;
    public double ELEVATOR_MIN_HEIGHT = 0.0;

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
    default void setP(double p) {}
    
    default void setI(double i) {}

    default void setD(double d) {}

    default void setFF(double ff) {}
    /** Returns the PID arguments used in setPIDConstants */
    default double getP() { return 0.0; }

    default double getI() { return 0.0; }

    default double getD() { return 0.0; }

    default double getFF() { return 0.0; }

}