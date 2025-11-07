package frc.robot.subsystems.superstructure.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperStructureMechanism {
  private final LoggedMechanism2d mechanism;

  private final LoggedMechanismLigament2d turret;

  private final String key;

  public SuperStructureMechanism(String key, Color coralColor, double lineWidth) {
    this.key = key;
    mechanism =
        new LoggedMechanism2d(
            MechanismConstants.displayWidth,
            MechanismConstants.displayHeight,
            new Color8Bit(Color.kWhite));

    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            "superstructure",
            MechanismConstants.rootPosition.getX(),
            MechanismConstants.rootPosition.getY());

    turret =
        new LoggedMechanismLigament2d(
            "turret",
            MechanismConstants.turretLength,
            MechanismConstants.turretInitialAngle.getDegrees(),
            lineWidth,
            new Color8Bit(coralColor));
    root.append(turret);
  }

  /** Update arm visualizer with current arm angle */
  public void update(Rotation2d turretAngle) {
    turret.setAngle(MechanismConstants.turretInitialAngle.plus(turretAngle));

    Logger.recordOutput("Superstructure/" + key + "/Mechanism2d", mechanism);
  }
}
