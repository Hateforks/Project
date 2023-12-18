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

public class ElevatorIOSim implements ElevatorIO {
    private final ProfiledPIDController m_controller =
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

 @Override
    public double getDistance() {
        return sim.getPositionMeters();
    }

    @Override
    public boolean atSetpoint() {
        return m_controller.atGoal();
    }

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

    @Override
    public void setFF(double ff) {
        m_feedforward = new ElevatorFeedforward(
            kElevatorkS,
            kElevatorkG,
            ff,
            kElevatorkA
        );
    }

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
        return m_feedforward.calculate(m_controller.getSetpoint().velocity);
    }
    
}

