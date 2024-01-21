package frc.robot;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.MutableMeasure.mutable;

public class SysIdCommands {
    
private static final String kCANBus = "3847";
    public static final TalonFX m_leftLeader = new TalonFX(1, kCANBus);
    public static final TalonFX m_rightLeader = new TalonFX(11, kCANBus);
    public static final TalonFX m_leftFollower = new TalonFX(21, kCANBus);
    public static final TalonFX m_rightFollower = new TalonFX(31, kCANBus);

    private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);
    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    // Creates a SysIdRoutine
    SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        m_leftLeader.setVoltage(volts.in(Volts));
        m_rightLeader.setVoltage(volts.in(Volts));
      }, log -> {
        // Record a frame for the left motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        log.motor("drive-left")
            .voltage(
                m_appliedVoltage.mut_replace(
                    m_leftLeader.get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Meters))
            .linearVelocity(
                m_velocity.mut_replace(m_leftEncoder.getRate(), MetersPerSecond));
        // Record a frame for the right motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        log.motor("drive-right")
            .voltage(
                m_appliedVoltage.mut_replace(
                    m_right.get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
            .linearVelocity(
                m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
      }, this));
    
    public static Command sysIdQuasistaticForward(){
        return routine.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
    }

    public static Command sysIdQuasistaticBackward(){
        return routine.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
    }

    public static Command sysIdDynamicForward(){
        return routine.sysIdDynamic(SysIdRoutine.Direction.kForward);
    }

    public static Command sysIdDynamicBackward(){
        return routine.sysIdDynamic(SysIdRoutine.Direction.kReverse);
    }
}