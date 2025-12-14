package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.robotzgarage.frc2026.util.MotorConfigs.TalonFXConfig;

public class TurretConstants {
  public static final int canId = 6;

  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 25.0 * (216.0 / 24.0); // for REAL turret

  // Tune this as needed
  public static final Rotation2d intialPosition = Rotation2d.fromDegrees(0);
  public static final Rotation2d minimumPosition = Rotation2d.fromDegrees(-90);
  public static final Rotation2d maximumPosition = Rotation2d.fromDegrees(90);

  public static final TalonFXConfig turretTalonFXConfig =
      TalonFXConfig.builder()
          .canId(6)
          .reduction(reduction)
          .currentLimitAmps(30)
          .invert(true)
          .isBrakeMode(true)
          .foc(true)
          .slot0Config(
              new Slot0Configs()
                  // feedforward
                  .withKS(0.05)
                  .withKV(0.05)
                  .withKA(0.0)
                  // feedback
                  .withKP(1.0)
                  .withKI(0.0)
                  .withKD(0.0))
          .mmConfig(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(5.0 * reduction)
                  .withMotionMagicAcceleration(4.5 * reduction)
                  .withMotionMagicJerk(20.0 * reduction))
          .dcMotor(DCMotor.getFalcon500Foc(1))
          .moi(0.00002)
          .build();
}
