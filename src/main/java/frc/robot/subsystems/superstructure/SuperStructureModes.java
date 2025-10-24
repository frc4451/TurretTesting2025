package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SuperStructureModes {
  MINUPOS(Rotation2d.fromDegrees(-90)),
  MAXPOS(Rotation2d.fromDegrees(90));

  public final Rotation2d turretPos;

  private SuperStructureModes(Rotation2d turretPos) {
    this.turretPos = turretPos;
  }
}
