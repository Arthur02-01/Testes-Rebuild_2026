package frc.robot.Hardwares;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constantes.ConstantesShooter;

@SuppressWarnings("unused")
public class HardwaresShooter {

    public final SparkMax arlindo;

    public final RelativeEncoder arlindoEncoder;

    public final SparkClosedLoopController arlindopid;

    @SuppressWarnings("removal" )
    public HardwaresShooter() {

        arlindo = new SparkMax(
            ConstantesShooter.Shooter.ShooterArlindo,
            SparkLowLevel.MotorType.kBrushless
        );

        arlindoEncoder = arlindo.getEncoder();


        arlindopid = arlindo.getClosedLoopController();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(ConstantesShooter.CORRENTE_MAXIMA_A);

        cfg.encoder
           .velocityConversionFactor(1.0 / ConstantesShooter.arlindoReducao)
           .positionConversionFactor(1.0 / ConstantesShooter.arlindoReducao);

        cfg.closedLoop
        .p(ConstantesShooter.PID_P)
        .i(ConstantesShooter.PID_I)
        .d(ConstantesShooter.PID_D)
        .iZone(ConstantesShooter.PID_IZONE_RPM)
        .velocityFF(ConstantesShooter.FF_VELOCIDADE)
        .outputRange(-1.0, 1.0);
        cfg.closedLoopRampRate(ConstantesShooter.RAMP_RATE_S);

        arlindo.configure(cfg,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters
        );
    }
}
