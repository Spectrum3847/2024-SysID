package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class Robot extends LoggedRobot {
    public static Auton auton;
 
    public final DutyCycleOut m_leftOut = new DutyCycleOut(0);
    public final DutyCycleOut m_rightOut = new DutyCycleOut(0);
 
    private final XboxController m_driverJoy = new XboxController(0);

    @Override
    public void robotInit() {
        Logger.recordMetadata("Sysid", "SysidValues"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
        
        // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
       auton = new Auton();
        System.out.println("Started Auton");

        // start with factory-default configs
       var currentConfigs = new MotorOutputConfigs();
 
       // The left motor is CCW+
       currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
       SysIdCommands.m_leftLeader.getConfigurator().apply(currentConfigs);
 
       // The right motor is CW+
       currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
       SysIdCommands.m_rightLeader.getConfigurator().apply(currentConfigs);
 
       // Ensure our followers are following their respective leader
       SysIdCommands.m_leftFollower.setControl(new Follower(SysIdCommands.m_leftLeader.getDeviceID(), false));
       SysIdCommands.m_rightFollower.setControl(new Follower(SysIdCommands.m_rightLeader.getDeviceID(), false));
    }
 
    @Override
    public void teleopPeriodic() {
       // retrieve joystick inputs
       var fwd = -m_driverJoy.getLeftY();
       var rot = m_driverJoy.getRightX();
 
       // modify control requests
       m_leftOut.Output = fwd + rot;
       m_rightOut.Output = fwd - rot;
 
       // send control requests
       SysIdCommands.m_leftLeader.setControl(m_leftOut);
       SysIdCommands.m_rightLeader.setControl(m_rightOut);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        Command autonCommand = Auton.getAutonomousCommand();
        autonCommand.schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

  
 }