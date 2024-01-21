package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class Auton {
    public static final SendableChooser<Command> autonChooser = new SendableChooser<>();

    // A chooser for autonomous commands
    public static void setupSelectors() {
        //SysId Configuration Autos
        autonChooser.addOption("SysIdQuasistaticForward", SysIdCommands.sysIdQuasistaticForward());
        autonChooser.addOption("SysIdQuasistaticBackward", SysIdCommands.sysIdQuasistaticBackward());
        autonChooser.addOption("SysIdDynamicForward", SysIdCommands.sysIdDynamicForward());
        autonChooser.addOption("SysIdDynamicBackward", SysIdCommands.sysIdDynamicBackward());

        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    // Setup the named commands
    public static void setupNamedCommands() {
        // Register Named Commands
        // NamedCommands.registerCommand("autoBalance", new AutoBalance());
        // NamedCommands.registerCommand("alignToSpeaker", new AutoBalance());
    }

    // Subsystem Documentation:
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
    public Auton() {
        setupSelectors(); // runs the command to start the chooser for auto on shuffleboard
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public static Command getAutonomousCommand() {
        // return new CharacterizeLauncher(Robot.launcher);
        Command auton = autonChooser.getSelected(); // sees what auto is chosen on shuffleboard
        if (auton != null) {
            return auton; // checks to make sure there is an auto and if there is it runs an auto
        } else {
            return new PrintCommand(
                    "*** AUTON COMMAND IS NULL ***"); // runs if there is no auto chosen, which
            // shouldn't happen because of the default
            // auto set to nothing which still runs
            // something
        }
    }
}
