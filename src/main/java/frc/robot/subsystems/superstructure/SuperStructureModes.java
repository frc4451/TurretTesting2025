package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public enum SuperStructureModes {
  MINUPOS(Rotation2d.kCCW_90deg),
  MAXPOS(Rotation2d.kCW_90deg);

  public final Rotation2d turretPos;

  private SuperStructureModes(Rotation2d turretPos) {
    this.turretPos = turretPos;
  }
}
