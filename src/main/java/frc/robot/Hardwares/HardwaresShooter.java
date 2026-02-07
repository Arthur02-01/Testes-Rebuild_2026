package frc.robot.Hardwares;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constantes.ConstantesShooter;

@SuppressWarnings("unused")
public class HardwaresShooter {

    public final SparkMax arlindo;
    public final SparkMax boquinha;

    public final RelativeEncoder arlindoEncoder;
    public final RelativeEncoder boquinhaEncoder;

    public final SparkClosedLoopController arlindopid;
    public final SparkClosedLoopController boquinhapid;


    @SuppressWarnings("removal" )
    public HardwaresShooter() {

        arlindo = new SparkMax(
            ConstantesShooter.Shooter.ShooterArlindo,
            SparkLowLevel.MotorType.kBrushless
        );

        boquinha = new SparkMax(
            ConstantesShooter.Shooter.ShooterBoquinha,
            SparkLowLevel.MotorType.kBrushless
        );

        arlindoEncoder = arlindo.getEncoder();
        boquinhaEncoder = boquinha.getEncoder();

        arlindopid = arlindo.getClosedLoopController();
        boquinhapid = boquinha.getClosedLoopController();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(ConstantesShooter.CORRENTE_MAXIMA_A);

        cfg.encoder
           .velocityConversionFactor(1.0 / ConstantesShooter.arlindoReducao)
           .positionConversionFactor(1.0 / ConstantesShooter.arlindoReducao);

        cfg.encoder
           .velocityConversionFactor(1.0 / ConstantesShooter.boquinhaReducao)
           .positionConversionFactor(1.0 / ConstantesShooter.boquinhaReducao);

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

        boquinha.configure(cfg,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters
        );
    }
}
