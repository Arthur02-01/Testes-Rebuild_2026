package frc.robot.Hardwares;

import com.ctre.phoenix6.signals.InvertedValue;
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

    // ================= BOQUINHA =================

    SparkMaxConfig cfgBoquinha = new SparkMaxConfig();

    cfgBoquinha
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ConstantesIndex.CORRENTE_MAXIMA_A_01)
        .inverted(false);

    cfgBoquinha.encoder
        .velocityConversionFactor(1.0 / ConstantesIndex.boquinhaReducao)
        .positionConversionFactor(1.0 / ConstantesIndex.boquinhaReducao);

    cfgBoquinha.closedLoop
        .p(ConstantesIndex.PID_P_01)
        .i(ConstantesIndex.PID_I_01)
        .d(ConstantesIndex.PID_D_01)
        .iZone(ConstantesIndex.PID_IZONE_RPM_01)
        .outputRange(-1.0, 1.0);

    cfgBoquinha.closedLoopRampRate(ConstantesIndex.RAMP_RATE_S_01);

    boquinha.configure(
        cfgBoquinha,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
    );


    SparkMaxConfig cfgIndex = new SparkMaxConfig();

    cfgIndex
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ConstantesIndex.CORRENTE_MAXIMA_A_01)
        .inverted(true);
        


    cfgIndex.encoder
        .velocityConversionFactor(1.0 / ConstantesIndex.indexReducao)
        .positionConversionFactor(1.0 / ConstantesIndex.indexReducao);

    cfgIndex.closedLoop
        .p(ConstantesIndex.PID_P_01)
        .i(ConstantesIndex.PID_I_01)
        .d(ConstantesIndex.PID_D_01)
        .iZone(ConstantesIndex.PID_IZONE_RPM_01)
        .outputRange(-1.0, 1.0);

    cfgIndex.closedLoopRampRate(ConstantesIndex.RAMP_RATE_S_01);

    index.configure(
        cfgIndex,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
    );
}
}