package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class TurretConstants {
  public static final int canId = 6;

  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 25 * (216 / 24);
  public static final double moi = 0.000002;

  public static final boolean invert = true;
  public static final double currentLimitAmps = 30;

  public static final boolean isBrakeMode = true;

  // Tune this as needed
  public static final Rotation2d intialPosition = Rotation2d.fromDegrees(0);
  public static final Rotation2d minimumPosition = Rotation2d.fromDegrees(-90);
  public static final Rotation2d maximumPosition = Rotation2d.fromDegrees(90);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html#combined-feedforward-and-feedback-control
  public static final Slot0Configs gains =
      new Slot0Configs()
          // feedforward
          .withKS(0.05)
          .withKV(0.05)
          .withKA(0.0)
          // feedback
          .withKP(1)
          .withKI(0.0)
          .withKD(0.0);

  public static final MotionMagicConfigs mmConfig =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(reduction * 3.0 / 2.0)
          .withMotionMagicAcceleration(4.5 * reduction)
          .withMotionMagicJerk(20 * reduction);

  public static final boolean foc = true;
}
