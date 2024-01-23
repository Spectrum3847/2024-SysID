package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class FlywheelMechanism extends SubsystemBase {
    private final TalonFX front_right = new TalonFX(Constants.front_right_ID, Constants.CANBUS);
    private final TalonFX front_left = new TalonFX(Constants.front_left_ID, Constants.CANBUS);
    private final TalonFX back_right = new TalonFX(Constants.back_right_ID, Constants.CANBUS);
    private final TalonFX back_left = new TalonFX(Constants.back_left_ID, Constants.CANBUS);
    private final DutyCycleOut m_joystickControl = new DutyCycleOut(0);
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    private SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Default ramp rate is acceptable
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                null,          // Default timeout is acceptable
                                       // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    front_right.setControl(m_sysidControl.withOutput(volts.in(Volts)));
                    front_left.setControl(m_sysidControl.withOutput(volts.in(Volts)));
                    back_right.setControl(m_sysidControl.withOutput(volts.in(Volts)));
                    back_left.setControl(m_sysidControl.withOutput(volts.in(Volts)));
                },
                null,
                this));

    public FlywheelMechanism() {
        setName("Flywheel");

        // TalonFXConfiguration cfg = new TalonFXConfiguration();
        // m_motorToTest.getConfigurator().apply(cfg);

        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            front_right.getPosition(),
            front_right.getVelocity(),
            front_right.getMotorVoltage(),
            front_left.getPosition(),
            front_left.getVelocity(),
            front_left.getMotorVoltage(),
            back_right.getPosition(),
            back_right.getVelocity(),
            back_right.getMotorVoltage(),
            back_left.getPosition(),
            back_left.getVelocity(),
            back_left.getMotorVoltage());

        /* Optimize out the  other signals, since they're not particularly helpful for us */
        //m_motorToTest.optimizeBusUtilization();
    }

    // public Command joystickDriveCommand(DoubleSupplier output) {
    //     return run(() -> m_motorToTest.setControl(m_joystickControl.withOutput(output.getAsDouble())));
    // }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }
}