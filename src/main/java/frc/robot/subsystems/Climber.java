package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    /* ================== STATE MACHINE ================== */

    public enum Estado {
        IDLE,
        MOVENDO,
        MANUAL,
        HOLD,
        FALHA
    }

    private Estado estado = Estado.IDLE;

    /* ================== HARDWARE ================== */

    private final SparkMax motorClimber;
    private final SparkMax motorFollower;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pid;

    /* ================== CONSTANTES FÍSICAS ================== */

    private static final double DIAMETRO_TAMBOR = 0.30;               // Estão em Metros todas
    private static final double CIRCUNFERENCIA = Math.PI * DIAMETRO_TAMBOR;
    private static final double REDUCAO = 64.0;

    private static final double LIMITE_INFERIOR = 0.0;                
    private static final double LIMITE_SUPERIOR = 1.15;              

    private static final double MARGEM_ERRO = 0.01;                  
    private static final double ERRO_TRAVAMENTO = 0.02;
    


    /* ================== DETECÇÃO DE TRAVAMENTO ================== */

    private static final double DT = 0.02;
    private static final double VELOCIDADE_MIN = 0.002;               
    private static final double TEMPO_MAX_TRAVADO = 0.4;              

    private double posicaoAnterior = 0.0;
    private double tempoSemMovimento = 0.0;
    
    /* ================== CONTROLE ================== */

    private double alvoAltura = Double.NaN;
    private double rotacoesHoldMotor = 0.0;
    private double manualOutput = 0.0;

    /* ================== CONSTRUTOR ================== */

    public Climber() {

        motorClimber = new SparkMax(
            Constants.ClimberConstants.ClimberMotor,
            MotorType.kBrushless
        );

        motorFollower = new SparkMax(
            Constants.ClimberConstants.ClimberMotor2,
            MotorType.kBrushless
        );

        encoder = motorClimber.getEncoder();
        pid = motorClimber.getClosedLoopController();

        SparkMaxConfig masterCfg = new SparkMaxConfig();
        masterCfg
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);

        masterCfg.closedLoop
            .p(0.4)
            .i(0.0)
            .d(0.02)
            .outputRange(-1.0, 1.0);

        SparkMaxConfig followerCfg = new SparkMaxConfig();
        followerCfg
            .follow(motorClimber, false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);

        motorClimber.configure(
            masterCfg,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        motorFollower.configure(
            followerCfg,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    /* ================== CONVERSÕES ================== */

    private double alturaParaRotacoesMotor(double alturaMetros) {
        double rotacoesTambor = alturaMetros / CIRCUNFERENCIA;
        return rotacoesTambor * REDUCAO;
    }

    private double rotacoesMotorParaAltura(double rotacoesMotor) {
        double rotacoesTambor = rotacoesMotor / REDUCAO;
        return rotacoesTambor * CIRCUNFERENCIA;
    }

    /* ================== COMANDOS ================== */

    public void irParaAltura(double alturaMetros) {
        alturaMetros = MathUtil.clamp(
            alturaMetros,
            LIMITE_INFERIOR,
            LIMITE_SUPERIOR
        );

        alvoAltura = alturaMetros;

        pid.setReference(
            alturaParaRotacoesMotor(alturaMetros),
            SparkBase.ControlType.kPosition
        );

        setEstado(Estado.MOVENDO);
    }

    public void controleManual(double output) {
        manualOutput = MathUtil.clamp(output, -0.4, 0.4);
        setEstado(Estado.MANUAL);
    }

    public double getAlturaAtual() {
    return rotacoesMotorParaAltura(encoder.getPosition());
    }
    public void zerarEncoder() {
        encoder.setPosition(0.0);
    }

    public void subirStep(double stepMetros) {
        irParaAltura(getAlturaAtual() + stepMetros);
    }

    public void descerStep(double stepMetros) {
        irParaAltura(getAlturaAtual() - stepMetros);
    }

    /* ================== ESTADOS ================== */

    private void setEstado(Estado novo) {
        if (estado == novo) return;

        if (estado == Estado.MOVENDO || estado == Estado.MANUAL) {
            motorClimber.stopMotor();
        }

        estado = novo;

        if (estado == Estado.HOLD) {
            rotacoesHoldMotor = encoder.getPosition();
        }

        if (estado == Estado.FALHA) {
            motorClimber.stopMotor();
        }
    }

    /* ================== EXECUÇÕES ================== */

    private void executarMovendo() {
        double alturaAtual = rotacoesMotorParaAltura(encoder.getPosition());
        double erro = alvoAltura - alturaAtual;

        if (Math.abs(erro) <= MARGEM_ERRO) {
            setEstado(Estado.HOLD);
            return;
        }

        pid.setReference(
            alturaParaRotacoesMotor(alvoAltura),
            SparkBase.ControlType.kPosition
        );
    }

    private void executarHold() {
        double erro = rotacoesHoldMotor - encoder.getPosition();

        if (Math.abs(erro) < 0.05) {
            motorClimber.stopMotor();
            return;
        }

        pid.setReference(
            rotacoesHoldMotor,
            SparkBase.ControlType.kPosition
        );
    }

    /* ================== PERIODIC ================== */

    @Override
    public void periodic() {

        double posicaoAtual = encoder.getPosition();
        double velocidade = (posicaoAtual - posicaoAnterior) / DT;

        double alturaAtual = rotacoesMotorParaAltura(posicaoAtual);
        double erroAltura = Math.abs(alvoAltura - alturaAtual);

        if (estado == Estado.MOVENDO && erroAltura > ERRO_TRAVAMENTO) {
            if (Math.abs(velocidade) < VELOCIDADE_MIN) {
                tempoSemMovimento += DT;
            } else {
                tempoSemMovimento = 0.0;
            }
        } else {
            tempoSemMovimento = 0.0;
        }

        if (tempoSemMovimento >= TEMPO_MAX_TRAVADO) {
            setEstado(Estado.FALHA);
        }

        posicaoAnterior = posicaoAtual;

        switch (estado) {
            case MOVENDO -> executarMovendo();
            case HOLD    -> executarHold();
            case MANUAL  -> motorClimber.set(manualOutput);
            default      -> {}
        }

        SmartDashboard.putNumber(
            "Climber/Altura (m)",
            alturaAtual
        );

        SmartDashboard.putNumber(
            "Climber/Erro (m)",
            erroAltura
        );

        SmartDashboard.putString(
            "Climber/Estado",
            estado.name()
        );
    }
}