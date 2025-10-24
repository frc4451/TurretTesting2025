package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.field.FieldConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.feedforward_controller.EmptyFeedforwardController;
import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import frc.robot.subsystems.rollers.follow.FollowRollersIOSim;
import frc.robot.subsystems.rollers.follow.FollowRollersIOTalonFX;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
// import frc.robot.subsystems.superstructure.constants.SuperStructureConstants;
// import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.SuperStructureModes;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final String name = "Superstructure";

  private final Turret turret;

  private SuperStructureModes currentMode = SuperStructureModes.MINUPOS;

  private boolean isAtMode = false;

  public SuperStructure() {
    SingleRollerIO pivotIO;

    switch (Constants.currentMode) {
      case REAL:
        pivotIO =
            new SingleRollerIOTalonFX(
                TurretConstants.canId,
                TurretConstants.reduction,
                TurretConstants.currentLimitAmps,
                TurretConstants.invert,
                TurretConstants.isBrakeMode,
                TurretConstants.foc,
                TurretConstants.gains,
                TurretConstants.mmConfig);
        break;

      case SIM:
        pivotIO =
            new SingleRollerIOSim(
                TurretConstants.gearbox,
                TurretConstants.reduction,
                TurretConstants.moi,
                TurretConstants.gains,
                TurretConstants.mmConfig,
                new EmptyFeedforwardController());
        break;

      case REPLAY:
      default:
        pivotIO = new SingleRollerIO() {};
        break;
    }

    turret = new Turret(name + "/Turret", pivotIO);

  }

  @Override
  public void periodic() {
    // if (DriverStation.isDisabled()) {
    //   setCurrentMode(SuperStructureModes.TUCKED);
    //   shooter.setShooterMode(ShooterModes.NONE);
    // }

    boolean isPivotAtMode = turret.isNear(currentMode.turretPos);

    turret.setGoal(currentMode.turretPos);

    Logger.recordOutput(name + "/IsPivotAtMode", isPivotAtMode);

    turret.periodic();

  }

  // public Command elevatorManualCommand(DoubleSupplier supplier) {
  //   return run(() -> elevator.runVolts(supplier.getAsDouble())).finallyDo(() -> elevator.stop());
  // }

  // public Command pivotManualCommand(DoubleSupplier supplier) {
  //   return run(() -> coralPivot.runVolts(supplier.getAsDouble()))
  //       .finallyDo(() -> coralPivot.stop());
  // }

  private void setCurrentMode(SuperStructureModes nextMode) {
    if (currentMode != nextMode) {
      currentMode = nextMode;
    }
  }

  public Command setModeCommand(SuperStructureModes nextMode) {
    return Commands.runOnce(() -> setCurrentMode(nextMode));
  }

  // public void setShooterMode(ShooterModes nextShooterMode) {
  //   shooter.setShooterMode(nextShooterMode);
  // }

  // public Command setShooterModeCommand(ShooterModes nextShooterMode) {
  //   return Commands.runOnce(() -> setShooterMode(nextShooterMode));
  // }

  public Trigger isAtMode() {
    return new Trigger(() -> isAtMode);
  }

  // public Command intake() {
  //   return Commands.sequence(
  //           setShooterModeCommand(ShooterModes.INTAKE)
  //               .unless(isCoralIntaked().or(() -> Constants.currentMode == Constants.Mode.SIM)),
  //           Commands.waitUntil(
  //               isCoralIntaked().or(() -> Constants.currentMode == Constants.Mode.SIM)))
  //       .finallyDo(() -> shooter.setShooterMode(ShooterModes.NONE));
  // }

  // public Command score(SuperStructureModes mode) {
  //   return Commands.sequence(
  //           setModeAndWaitCommand(mode),
  //           shootCoral(),
  //           setModeAndWaitCommand(SuperStructureModes.TUCKED))
  //       // .onlyIf(isCoralIntaked())
  //       .finallyDo(this::resetModes);
  // }

  // public Command bargeShot() {
  //   return Commands.sequence(
  //           setModeCommand(SuperStructureModes.Barge),
  //           Commands.waitUntil(() -> elevator.isNear(SuperStructureConstants.shootNetHeight)),
  //           setShooterModeCommand(ShooterModes.ALGAE_SHOOT),
  //           Commands.waitSeconds(0.05))
  //       .finallyDo(
  //           () -> {
  //             setCurrentMode(SuperStructureModes.TUCKED_L4);
  //             shooter.setShooterMode(ShooterModes.NONE);
  //           });
  // }

  public Command setModeAndWaitCommand(SuperStructureModes mode) {
    return Commands.sequence(setModeCommand(mode), Commands.waitUntil(isAtMode()));
  }

  // public Command shootCoral() {
  //   return Commands.sequence(
  //       setShooterModeCommand(ShooterModes.SHOOT),
  //       Commands.sequence(
  //           Commands.waitUntil(isCoralIntaked().negate()), Commands.waitSeconds(0.125)),
  //       setShooterModeCommand(ShooterModes.NONE));
  // }

  // public void resetModes() {
  //   shooter.setShooterMode(ShooterModes.NONE);
  //   setCurrentMode(SuperStructureModes.TUCKED);
  // }

  // public Trigger isCoralIntaked() {
  //   return new Trigger(() -> shooter.isCoralDetected());
  // }

  // public Trigger isCoralIntaking() {
  //   return new Trigger(() -> shooter.isCoralIntaking());
  // }

  // public Trigger isNotShooting() {
  //   return new Trigger(() -> shooter.isNotShooting());
  // }
}
