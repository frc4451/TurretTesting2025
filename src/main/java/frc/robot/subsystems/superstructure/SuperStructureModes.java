package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.turret.TurretConstants;

public enum SuperStructureModes {
  AUTOAIMTURRET(Rotation2d.kZero),
  MINIMUM(TurretConstants.minimumPosition),
  MAXIMUM(TurretConstants.maximumPosition),
  MANUAL(Rotation2d.kZero),
  L180(Rotation2d.fromDegrees(180)),
  R180(Rotation2d.fromDegrees(-180));

  public final Rotation2d turretPosition;

  private SuperStructureModes(Rotation2d turretPosition) {
    this.turretPosition = turretPosition;
  }
}
