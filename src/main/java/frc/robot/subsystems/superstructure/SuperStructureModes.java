package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.turret.TurretConstants;

public enum SuperStructureModes {
  MINIMUM(TurretConstants.intialPosition),
  MAXIMUM(TurretConstants.maximumPosition);

  public final Rotation2d turretPosition;

  private SuperStructureModes(Rotation2d turretPosition) {
    this.turretPosition = turretPosition;
  }
}
