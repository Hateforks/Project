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

// spin off of the IO class
public class ElevatorIOSim implements ElevatorIO {
    private final ProfiledPIDController m_controller =
      // creqation of new objects to allow the simulation to run
        new ProfiledPIDController(
            kElevatorKp,
            kElevatorKi,
            kElevatorKd,
            new TrapezoidProfile.Constraints(2.45, 2.45)
        );
        ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(
            kElevatorkS,
            kElevatorkG,
            kElevatorkV,
            kElevatorkA
        );
         private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

//         main sensor function that could be utilized in the future for automonous movement as well as precise control
        private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
         private final ElevatorSim sim =
            new ElevatorSim(
                m_elevatorGearbox,
                kElevatorGearing,
                kCarriageMass,
                kElevatorDrumRadius,
                kMinElevatorHeightMeters,
                kMaxElevatorHeightMeters,
                false,
                VecBuilder.fill(0.01)
        );

private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    
    public ElevatorIOSim() {
        m_encoder.setDistancePerPulse(kElevatorEncoderDistPerPulse);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        /* the changing of aspects of the class that will allow for the 
        robot and the controller to have updated 
        and current information about its position and  power usage*/
        sim.update(0.02);
        m_encoderSim.setDistance(sim.getPositionMeters());
        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMeters = sim.getVelocityMetersPerSecond();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
        // inputs.voltage = new double[] {sim.getBusVoltageV()};
    } 

    @Override
    public void setVoltage(double motorVolts) {
        sim.setInputVoltage(motorVolts);
    }


    @Override
    public void goToSetpoint(double setpoint) {
      // defines the point of which the computer is expected to arrive at
        m_controller.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(m_encoder.getDistance());
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

        sim.setInputVoltage(feedforwardOutput + pidOutput);
    }

// functions used to return pertinent values
 @Override
    public double getDistance() {
        return sim.getPositionMeters();
    }

    @Override
    public boolean atSetpoint() {
        return m_controller.atGoal();
    }
// sets up the controller for objects of the LoggedDashboardNumber class
    @Override
    public void setP(double p) {
        m_controller.setP(p);
    }

    @Override
    public void setI(double i) {
        m_controller.setI(i);
    }

    @Override
    public void setD(double d) {
        m_controller.setD(d);
    }
// makes object of the ElevatorFeedforward class
    @Override
    public void setFF(double ff) {
        m_feedforward = new ElevatorFeedforward(
            kElevatorkS,
            kElevatorkG,
            ff,
            kElevatorkA
        );
    }
//  lots of getters
    @Override
    public double getP() {
        return m_controller.getP();
    }

    @Override
    public double getI() {
        return m_controller.getI();
    }

    @Override
    public double getD() {
        return m_controller.getD();
    }

    @Override
    public double getFF() {
        // allows the robot to see the velocity needed to end up at the desired point.
        return m_feedforward.calculate(m_controller.getSetpoint().velocity);
    }
    
}

