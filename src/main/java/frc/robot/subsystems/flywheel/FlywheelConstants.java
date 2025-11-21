package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.single.SingleRollerTypes.SingleRollerFXConfig;

public class FlywheelConstants {
  public static final double reduction = 1.0;

  public static final SingleRollerFXConfig flywheelLeft =
      new SingleRollerFXConfig(
          1,
          DCMotor.getKrakenX60Foc(1),
          1,
          0.000002,
          true,
          30,
          true,
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(-90),
          Rotation2d.fromDegrees(90),
          new Slot0Configs()
              // feedforward
              .withKS(0.05)
              .withKV(0.05)
              .withKA(0.0)
              // feedback
              .withKP(1.0)
              .withKI(0.0)
              .withKD(0.0),
          new MotionMagicConfigs()
              .withMotionMagicCruiseVelocity(5.0 * reduction)
              .withMotionMagicAcceleration(4.5 * reduction)
              .withMotionMagicJerk(20.0 * reduction),
          true);
  public static final SingleRollerFXConfig flywheelRight =
      new SingleRollerFXConfig(
          1,
          DCMotor.getKrakenX60Foc(1),
          1,
          0.000002,
          false,
          30,
          true,
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(-90),
          Rotation2d.fromDegrees(90),
          new Slot0Configs()
              // feedforward
              .withKS(0.05)
              .withKV(0.05)
              .withKA(0.0)
              // feedback
              .withKP(1.0)
              .withKI(0.0)
              .withKD(0.0),
          new MotionMagicConfigs()
              .withMotionMagicCruiseVelocity(5.0 * reduction)
              .withMotionMagicAcceleration(4.5 * reduction)
              .withMotionMagicJerk(20.0 * reduction),
          true);

  // public static final int canId = 6;

  // public static final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
  // public static final double reduction = 25.0 * (216.0 / 24.0); // for REAL turret
  // public static final double reduction = 1; // for minion motor
  // public static final double moi = 0.000002;

  // public static final boolean invert = true;
  // public static final double currentLimitAmps = 30;

  // public static final boolean isBrakeMode = true;

  // Tune this as needed
  // public static final Rotation2d intialPosition = Rotation2d.fromDegrees(0);
  // public static final Rotation2d minimumPosition = Rotation2d.fromDegrees(-90);
  // public static final Rotation2d maximumPosition = Rotation2d.fromDegrees(90);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html#combined-feedforward-and-feedback-control
  // public static final Slot0Configs gains =
  //     new Slot0Configs()
  //         // feedforward
  //         .withKS(0.05)
  //         .withKV(0.05)
  //         .withKA(0.0)
  //         // feedback
  //         .withKP(1.0)
  //         .withKI(0.0)
  //         .withKD(0.0);

  // public static final MotionMagicConfigs mmConfig =
  //     new MotionMagicConfigs()
  //         .withMotionMagicCruiseVelocity(5.0 * reduction)
  //         // .withMotionMagicCruiseVelocity(0.01 * reduction)
  //         .withMotionMagicAcceleration(4.5 * reduction)
  //         .withMotionMagicJerk(20.0 * reduction);

  // public static final boolean foc = true;
}
