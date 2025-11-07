package frc.robot.subsystems.superstructure.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class MechanismConstants {
  public static final double displayWidth = 1;
  public static final double displayHeight = 1;

  public static final Translation2d rootPosition = new Translation2d(displayWidth / 2.0, displayHeight / 2.0);

  public static final double turretLength = Units.inchesToMeters(7.0);
  public static final Rotation2d turretInitialAngle = Rotation2d.fromDegrees(90);

}
