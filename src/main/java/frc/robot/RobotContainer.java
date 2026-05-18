// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    private final DriveSubsystem m_drive = new DriveSubsystem();

    // Driver controller on port 0
    private final CommandXboxController m_driverController =
        new CommandXboxController(DriverConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Left stick Y (negated — up = positive) → throttle; right stick X → turn
        m_drive.setDefaultCommand(Commands.run(
            () -> m_drive.arcadeDrive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(),  Drivetrain.DEADBAND),
                 MathUtil.applyDeadband(m_driverController.getRightX(), Drivetrain.DEADBAND)
            ),
            m_drive
        ));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
