package frc.robot.Hardwares;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constantes.ConstantesIndex;

@SuppressWarnings("removal" )
public class HardwaresIndex {
    
    public final SparkMax boquinha;
    public final SparkMax index;

    public final RelativeEncoder boquinhaEncoder;
    public final RelativeEncoder indexEncoder;

    public final SparkClosedLoopController boquinhapid;
    public final SparkClosedLoopController indexpid;

    public HardwaresIndex(){
        boquinha = new SparkMax(
            ConstantesIndex.Index.ShooterBoquinha,
            SparkLowLevel.MotorType.kBrushless
        );
        index = new SparkMax(
            ConstantesIndex.Index.IndexMotor,
            SparkLowLevel.MotorType.kBrushless
        );
        boquinhaEncoder = boquinha.getEncoder();
        indexEncoder = index.getEncoder();
        
        boquinhapid = boquinha.getClosedLoopController();
        indexpid = index.getClosedLoopController();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(ConstantesIndex.CORRENTE_MAXIMA_A_01);
        
        SparkMaxConfig cfgBoquinha = new SparkMaxConfig();
        cfgBoquinha.encoder
            .velocityConversionFactor(1.0 / ConstantesIndex.boquinhaReducao)
            .positionConversionFactor(1.0 / ConstantesIndex.boquinhaReducao);

        SparkMaxConfig cfgIndex = new SparkMaxConfig();
        cfgIndex.encoder
            .velocityConversionFactor(1.0 / ConstantesIndex.indexReducao)
            .positionConversionFactor(1.0 / ConstantesIndex.indexReducao);


        cfg.closedLoop
        .p(ConstantesIndex.PID_P_01)
        .i(ConstantesIndex.PID_I_01)
        .d(ConstantesIndex.PID_D_01)
        .iZone(ConstantesIndex.PID_IZONE_RPM_01)
        .outputRange(-1.0, 1.0);
        cfg.closedLoopRampRate(ConstantesIndex.RAMP_RATE_S_01);

        boquinha.configure(cfg, 
        SparkBase.ResetMode.kNoResetSafeParameters, 
        SparkBase.PersistMode.kNoPersistParameters
        );
        index.configure(cfg,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
        );
    }
}
