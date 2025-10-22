package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.feedforward_controller.EmptyFeedforwardController;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.rollers.single.SingleRollerIOSim;
import frc.robot.subsystems.rollers.single.SingleRollerIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final String name = "Superstructure";

  private final Turret turret;

  private SuperStructureModes currentMode = SuperStructureModes.MINIMUM;

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

    // Disable this if you want to use manual control
    turret.setGoal(currentMode.turretPosition);

    Logger.recordOutput(name + "/IsTurretAtMode", isAtMode);

    turret.periodic();
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
}
