package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.rollers.feedforward_controller.EmptyFeedforwardController;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.superstructure.mechanism.SuperStructureMechanism;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.subsystems.superstructure.turret.TurretConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final String name = "Superstructure";

  private final Turret turret;

  private final SuperStructureMechanism goalMechanism =
      new SuperStructureMechanism("Goal", Color.kLightGreen, 10.0);
  private final SuperStructureMechanism measuredMechanism =
      new SuperStructureMechanism("Measured", Color.kDarkGreen, 3.0);

  private SuperStructureModes currentMode = SuperStructureModes.MANUAL;

  private boolean isAtMode = false;

  public SuperStructure() {
    SingleRollerIO turretIO;

    switch (Constants.currentMode) {
      case REAL:
        turretIO =
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
        turretIO =
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
        turretIO = new SingleRollerIO() {};
        break;
    }

    turret = new Turret(name + "/Turret", turretIO);
  }

  @Override
  public void periodic() {
    boolean isAtMode = turret.isNear(currentMode.turretPosition);

    switch (currentMode) {
      case MINIMUM:
      case MAXIMUM:
      case L180:
      case R180:
        turret.setGoal(currentMode.turretPosition);
        break;
      case AUTOAIMTURRET:
        handleTurretRotateToReefWithoutLimits();
        break;
      case MANUAL:
      default:
        break;
    }

    Logger.recordOutput(name + "/IsTurretAtMode", isAtMode);

    Logger.recordOutput(
        name + "/TurretRotation",
        new Pose2d(BobotState.getGlobalPose().getTranslation(), turret.getPosition()));

    Logger.recordOutput(name + "/IsTurretAlignedWithGoal", isTurretAligned());

    turret.periodic();

    measuredMechanism.update(turret.getPosition());
    goalMechanism.update(turret.getGoalPosition());
  }

  public Trigger isAtMode() {
    return new Trigger(() -> isAtMode);
  }

  public Command turretManualCommand(DoubleSupplier supplier) {
    return run(() -> turret.runVolts(supplier.getAsDouble())).finallyDo(() -> turret.stop());
  }

  private void setCurrentMode(SuperStructureModes nextMode) {
    if (currentMode != nextMode) {
      currentMode = nextMode;
    }
  }

  public Command setModeCommand(SuperStructureModes nextMode) {
    return Commands.runOnce(() -> setCurrentMode(nextMode));
  }

  public Command TreeRotate() {
    return Commands.repeatingSequence(
        Commands.deadline(Commands.waitSeconds(95), setModeCommand(SuperStructureModes.R180)),
        Commands.deadline(Commands.waitSeconds(95), setModeCommand(SuperStructureModes.L180)));
  }

  private Rotation2d getTargetRotation() {
    Rotation2d tagRotation = FieldUtils.getClosestReef().tag.pose().getRotation().toRotation2d();
    Rotation2d targetRotation = tagRotation.plus(Rotation2d.kPi);

    return targetRotation;
  }

  public Trigger isTurretAligned() {
    Rotation2d targetRotation = getTargetRotation();
    boolean isAtMode = turret.isNear(targetRotation);
    return new Trigger(() -> isAtMode);
  }

  private void handleTurretRotateToReefWithoutLimits() {
    turret.setGoal(getTargetRotation());
  }

  // You may want to look at this
  // https://github.com/Team254/FRC-2024-Public/blob/main/src/main/java/com/team254/frc2024/subsystems/turret/TurretSubsystem.java
  //
  // Currently this is broken, it still wraps around when it isn't supposed to
  private void handleTurretRotateToReefWithLimits() {
    Rotation2d robotRotation = BobotState.getGlobalPose().getRotation();
    Rotation2d targetRotation = getTargetRotation();

    // Convert target to robot-relative angle
    double targetRad = targetRotation.minus(robotRotation).getRadians();

    // Adjust for wrapping to prevent long rotations
    targetRad = adjustSetpointForWrap(targetRad);

    // Clamp within turret limits
    targetRad =
        Math.max(
            TurretConstants.minimumPosition.getRadians(),
            Math.min(TurretConstants.maximumPosition.getRadians(), targetRad));

    // Optional: don't move if already at target
    if (unwrapped(targetRad)) {
      return;
    }

    // Convert back to field-relative for turret controller
    Rotation2d clampedTarget = Rotation2d.fromRadians(targetRad).plus(robotRotation);
    turret.setGoal(clampedTarget);
  }

  private double adjustSetpointForWrap(double radiansFromCenter) {
    // We have two options the raw radiansFromCenter or +/- 2 * PI.
    double alternative = radiansFromCenter - 2.0 * Math.PI;
    double turretPositionRads = turret.getPosition().getRadians();

    if (radiansFromCenter < 0.0) {
      alternative = radiansFromCenter + 2.0 * Math.PI;
    }
    if (Math.abs(turretPositionRads - alternative)
        < Math.abs(turretPositionRads - radiansFromCenter)) {
      return alternative;
    }
    return radiansFromCenter;
  }

  private boolean unwrapped(double setpoint) {
    // Radians comparison intentional because this is the raw value going into
    // rotor.
    return epsilonEquals(setpoint, turret.getPosition().getRadians(), Math.toRadians(10.0));
  }

  private boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public Command shootCommand() {
    return run(() -> {
          Logger.recordOutput(name + "/IsShooting", true);
        })
        .finallyDo(
            () -> {
              Logger.recordOutput(name + "/IsShooting", false);
            });
  }
}
