package frc.robot.subsystems.drive.elevator;

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

public class Elevator extends SubsystemBase {
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private LoggedDashboardNumber p = new LoggedDashboardNumber("Elevator/P", ELEVATOR_PID[0]);
    private LoggedDashboardNumber i = new LoggedDashboardNumber("Elevator/I", ELEVATOR_PID[1]);
    private LoggedDashboardNumber d = new LoggedDashboardNumber("Elevator/D", ELEVATOR_PID[2]);
    private LoggedDashboardNumber ff = new LoggedDashboardNumber("Elevator/FF", ELEVATOR_PID[3]);

    private double setpoint = 0;

    private final ElevatorIO io;

      // Create a Mechanism2d visualization of the elevator
    private MechanismLigament2d ElevatorMechanism;

    public Elevator(ElevatorIO io) {
        this.io = io;
        SmartDashboard.putData(getName(), this);
    }

    public double highSetpoint() {
        return io.ELEVATOR_MAX_HEIGHT;
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);
    }
}