package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Elevator.ElevatorPhysicalConstants.ELEVATOR_PID;

// extension of the SubsystemBase Class, allows us to not have to repeat code
public class Elevator extends SubsystemBase {
    //  new object being made with the ElevatorIOAutoLogged class
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
/* new objects being made with the LoggedDashboardNumber class with a string value as well as a 
 a value rom a list storing PIDs */
    private LoggedDashboardNumber p = new LoggedDashboardNumber("Elevator/P", ELEVATOR_PID[0]);
    private LoggedDashboardNumber i = new LoggedDashboardNumber("Elevator/I", ELEVATOR_PID[1]);
    private LoggedDashboardNumber d = new LoggedDashboardNumber("Elevator/D", ELEVATOR_PID[2]);
    private LoggedDashboardNumber ff = new LoggedDashboardNumber("Elevator/FF", ELEVATOR_PID[3]);

    private double setpoint = 0;

    private final ElevatorIO io;

      // Create a Mechanism2d visualization of the elevator
    private MechanismLigament2d ElevatorMechanism;
// a setter that sets io, retrieves a name, and put the io and name in the dashboard 
    public Elevator(ElevatorIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }
// a getter that allows the elevator to read max height.
    public double highSetpoint() {
        return io.ELEVATOR_MAX_HEIGHT;
    }
    @Override
    public void periodic() {
        /* uses the variable inputs with the update inputs method in order to make sure io has 
         up to date info */
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);
        //allows the mechanism to know  how far to go
        ElevatorMechanism.setLength(io.getDistance());

        // Update the PID constants if they have changed
        if (p.get() != io.getP()) 
            io.setP(p.get());
        
        if (i.get() != io.getI())
            io.setI(i.get());
        
        if (d.get() != io.getD())
            io.setD(d.get());
        
        if (ff.get() != io.getFF())
            io.setFF(ff.get());
        
        // Log Inputs
        Logger.getInstance().processInputs("Elevator", inputs);
    }
    public void setVoltage(double motorVolts) {
        // limit the elevator if its past its limit
        if (io.getDistance() > io.ELEVATOR_MAX_HEIGHT && motorVolts > 0) {
            motorVolts = 0;
        } else if (io.getDistance() < io.ELEVATOR_MIN_HEIGHT && motorVolts < 0) {
            motorVolts = 0;
        }
         
        io.setVoltage(motorVolts);
    }
    // gives machine needed amount of energy to run by using a mathematical problem 
    public void move(double speed) {
        setVoltage(speed * 12);
    }

    public void runPID() {
        io.goToSetpoint(setpoint);
    }
// sets PID
    public void setPID(double setpoint) {
        this.setpoint = setpoint;
    }

/* gets the distance the elevator is from the setpoint and sees if it has the accuracy
 needed by comparing that value to the margin of error allowed */
    public boolean atSetpoint() {
        return Math.abs(io.getDistance() - setpoint) < ELEVATOR_TOLERANCE;
    }
// functions dealing with Elevator mechanism ligaments
    public void setMechanism(MechanismLigament2d mechanism) {
        ElevatorMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return ElevatorMechanism.append(mechanism);
    }

    public MechanismLigament2d getElevatorMechanism() {
        return new MechanismLigament2d("Elevator", 5, 36, 5, new Color8Bit(Color.kOrange));
    }
// gives elevaator the task of moving to the desired setpoint using the PID values.
    public Command PIDCommand(double setpoint) {
        return new FunctionalCommand(
            () -> setPID(setpoint), 
            () -> runPID(), 
            (stop) -> move(0), 
            this::atSetpoint, 
            this
        );
    }
}