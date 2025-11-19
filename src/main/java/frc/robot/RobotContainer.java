// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructureModes;
import frc.robot.util.CommandCustomXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // public final Drive drive;
  // private final Vision vision = new Vision();
  private final SuperStructure superStructure = new SuperStructure();

  // Controller
  public final CommandCustomXboxController controller = new CommandCustomXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new BobotState();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(),
        //         new ModuleIOSpark(0),
        //         new ModuleIOSpark(1),
        //         new ModuleIOSpark(2),
        //         new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim());

        break;

      case REPLAY:
      default:
        // Replayed robot, disable IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});
        break;
    }

    configureDefaultControls();
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureDefaultControls() {
    // switch (Constants.driverControl) {
    //   case FREE:
    //   default:
    //     drive.setDefaultCommand(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> -controller.getLeftYSquared(),
    //             () -> -controller.getLeftXSquared(),
    //             () -> -controller.getRightXSquared()));
    //     break;
    // }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Set up the controls for the Turret Here
    controller.b().onTrue(superStructure.setTreeCommand(SuperStructureModes.R180));
    controller.leftBumper().onTrue(superStructure.setModeCommand(SuperStructureModes.L90));
    controller.rightBumper().onTrue(superStructure.setModeCommand(SuperStructureModes.R90));
    // if (controller.isConnected()) {
    //   controller.x().onTrue(superStructure.TreeRotate());
    // } else if (!controller.isConnected()) {
    //   superStructure.TreeRotate();
    // }
    controller.x().onTrue(superStructure.setTreeCommand(SuperStructureModes.L180));
    controller.y().onTrue(superStructure.setModeCommand(SuperStructureModes.TreeRotate));
    // This is all that's needed for a demonstation
    controller
        .leftX()
        .whileTrue(superStructure.turretManualCommand(() -> controller.getLeftXSquared() / 6))
        .onTrue(superStructure.setModeCommand(SuperStructureModes.MANUAL));
    // controller.start().and(superStructure.isTurretAligned()).onTrue(superStructure.shootCommand());
  }
}
