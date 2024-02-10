package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class SwerveMechanism extends SubsystemBase {
    private final TalonFX front_right = new TalonFX(Constants.front_right_ID, Constants.CANBUS);
    private final TalonFX front_left = new TalonFX(Constants.front_left_ID, Constants.CANBUS);
    private final TalonFX back_right = new TalonFX(Constants.back_right_ID, Constants.CANBUS);
    private final TalonFX back_left = new TalonFX(Constants.back_left_ID, Constants.CANBUS);
    //private final VoltageOut m_sysidControl = new VoltageOut(0);
    private TorqueCurrentFOC m_sysidControl = new TorqueCurrentFOC(0);

    private SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds.of(0.5)),         // Default ramp rate is acceptable
                Volts.of(10), // Reduce dynamic voltage to 4 to prevent motor brownout
                Seconds.of(5),          // Default timeout is acceptable
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

    public SwerveMechanism() {
        setName("Swerve");

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        TalonFXConfiguration cfg_inverted = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg_inverted.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg_inverted.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        front_right.getConfigurator().apply(cfg_inverted);
        front_left.getConfigurator().apply(cfg);
        back_right.getConfigurator().apply(cfg_inverted);
        back_left.getConfigurator().apply(cfg);

        /* Speed up signals for better charaterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            front_right.getPosition(),
            front_right.getVelocity(),
            front_right.getMotorVoltage());

        /* Optimize out the  other signals, since they're not particularly helpful for us */
        front_right.optimizeBusUtilization();
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