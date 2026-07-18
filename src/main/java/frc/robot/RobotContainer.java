// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.io.motor.MotorIO;
import frc.lib.io.motor.MotorIOSpark;
import frc.lib.io.motor.MotorIOTalonFX;
import frc.lib.io.motor.MotorIOSpark.RevFollower;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.mechanisms.flywheel.FlywheelMechanismReal;
import frc.lib.mechanisms.rotary.RotaryMechanismReal;
import frc.robot.Constants.DoubleMotorConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SingleMotorConstants;
import frc.robot.subsystems.DoubleMotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Motors;
import frc.robot.subsystems.SingleMotor;

@SuppressWarnings("unused")
/**
 * The RobotContainer class is the central hub for defining the robot's subsystems,
 * commands, and button mappings.
 *
 * <p>This class handles the instantiation and configuration of all subsystems,
 * operator interfaces, and autonomous command routines. It also manages
 * scheduling of initial commands when TeleOp begins.
 *
 * <p>Subsystems such as the drivetrain, vision, and control interfaces are created here,
 * and commands are provided for autonomous and teleoperated operation.
 *
 * <p>All button bindings and command scheduling should be configured within this class.
 *
 */
public class RobotContainer {

    //private final Motors m_motor = new Motors();
    // private final SingleMotor m_singleMotor = new SingleMotor(SingleMotorConstants.getSingleMotorIO());
    // private final DoubleMotor m_doubleMotor = new DoubleMotor(DoubleMotorConstants.getDoubleMotorIO());

    // private final Intake intake = new Intake(
    // new FlywheelMechanismReal("Intake_Roller",
    //     new MotorIOSpark(SingleMotorConstants.NAME, SingleMotorConstants.getREVConfig(), Ports.SingleMotor)
    //     ),
    // new RotaryMechanismReal("Intake_Pivot",
    //     new MotorIOSpark(DoubleMotorConstants.NAME, DoubleMotorConstants.getREVConfig(), Ports.DoubleMotorMain),
    //     DoubleMotorConstants.CONSTANTS,
    //     Optional.empty(),
    //     ""
    //     )
    // );

    private final Intake m_intake = new Intake(SingleMotorConstants.getSingleMotorIO(), DoubleMotorConstants.getDoubleMotorIO());
    
    Joystick driverJoystick = new Joystick(0);

    private JoystickButton btn_1 = new JoystickButton(driverJoystick, 1);
    private JoystickButton btn_2 = new JoystickButton(driverJoystick, 2);
    private JoystickButton btn_3 = new JoystickButton(driverJoystick, 3);
    private JoystickButton btn_4 = new JoystickButton(driverJoystick, 4);
    private JoystickButton btn_5 = new JoystickButton(driverJoystick, 5);
    private JoystickButton btn_6 = new JoystickButton(driverJoystick, 6);
    private JoystickButton btn_7 = new JoystickButton(driverJoystick, 7);
    private JoystickButton btn_8 = new JoystickButton(driverJoystick, 8);
    private JoystickButton btn_9 = new JoystickButton(driverJoystick, 9);
    private JoystickButton btn_10 = new JoystickButton(driverJoystick, 10);
    private JoystickButton btn_11 = new JoystickButton(driverJoystick, 11);
    private JoystickButton btn_12 = new JoystickButton(driverJoystick, 12);

    
    /**
     * Constructs a new RobotContainer.
     *
     * <p>Initializes all subsystems, operator interfaces, and telemetry.
     * Configures button bindings for driver and co-driver controls.
     * Also registers telemetry logging for the drivetrain subsystem.
     */
    public RobotContainer() {
        m_intake.setDefaultCommand(m_intake.stow());

        btn_4.whileTrue(m_intake.intake());
        btn_3.whileTrue(m_intake.unjam());
        btn_2.whileTrue(m_intake.turbob());
    }
    
    /**
     * Defines and registers autonomous commands available for use during the autonomous period.
     *
     * <p>This method can be used to map command names to specific autonomous actions,
     * allowing easy selection and scheduling of autonomous routines.
     */
    public void autoCommands() {
        // Map<String, Command> namedCommands = Map.ofEntries(
        // Map.entry("L1", new RunCommand(() -> m_arm.setPositionByIndex(5))),
        // Map.entry("L1Pre", new RunCommand(() -> m_arm.setPositionByIndex(4))),
        // Map.entry("BallDrop", m_thrower.pivotUnhook()),
        // Map.entry("StopDrive", new StopDrive(m_swerveBase)),
        // Map.entry("Eject", new RunCommand(() -> EjectesGamePiece = true)),
        // Map.entry("EjectStop", new RunCommand(() -> EjectesGamePiece = false))
        // );

        // // Register all at once
        // namedCommands.forEach(NamedCommands::registerCommand);
    }

    /**
     * Builds and returns the autonomous command to run during the autonomous period.
     *
     * <p>This method constructs the selected autonomous routine, which will be
     * scheduled when autonomous mode begins.
     *
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }

}
