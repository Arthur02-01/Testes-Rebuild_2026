package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Angulador extends SubsystemBase {

    /* ================= STATE MACHINE ================= */

    public enum Estado {
        DESABILITADO,
        MANUAL,
        PERFIL,
        HOLD,
        FALHA
    }

    private Estado estado = Estado.DESABILITADO;

    /* ================= HARDWARE ================= */

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pid;
    private final ArmFeedforward feedforward;

    /* ================= LIMITES ================= */

    public static final double LIMITE_SUPERIOR = 50.0;
    public static final double LIMITE_CENTRAL = 25.0;
    public static final double LIMITE_INFERIOR = 10.0;

    /* margem maior → evita forçar motor */
    private static final double MARGEM_ERRO = 1.65;

    /* ================= CONTROLE ================= */

    private static final double REDUCAO = 5.0;
    private static final double DT = 0.02;
    private double alvoAtual = Double.NaN;

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(90.0, 180.0);

    private TrapezoidProfile.State goal =
            new TrapezoidProfile.State();

    private TrapezoidProfile.State setpoint =
            new TrapezoidProfile.State();

    private double anguloHold = 0.0;
    private double manualOutput = 0.0;

    /* ================= DETECÇÃO DE FALHA ================= */

    private static final double VELOCIDADE_MIN = 0.01;
    private static final double TEMPO_MAX_TRAVADO = 0.8;
    private static final double ERRO_TOLERANCIA = 2.0;

    private double posicaoAnterior = 0.0;
    private double tempoSemMovimento = 0.0;

    /* ================= CONSTRUTOR ================= */

    public Angulador() {

        motor = new SparkMax(
                Constants.Alinhador.AlinhadorMotor,
                SparkLowLevel.MotorType.kBrushless
        );

        encoder = motor.getEncoder();
        pid = motor.getClosedLoopController();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(25);

        cfg.closedLoop
           .p(0.8)
           .i(0.0)
           .d(0.02);

        motor.configure(
                cfg,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );

        feedforward = new ArmFeedforward(
                Constants.FFAlinhador.kS,
                Constants.FFAlinhador.kG,
                Constants.FFAlinhador.kV
        );
    }

    /* ================= UTIL ================= */

    public double getAngulo() {
        return encoder.getPosition() * (360.0 / REDUCAO);
    }

    private double grausParaRotacao(double graus) {
        return (graus / 360.0) * REDUCAO;
    }

    public boolean emHold() {
        return estado == Estado.HOLD;
    }

    private double margemDinamica(double velocidade) {
    double margemMin = 0.5;   // precisão máxima
    double margemMax = 3.0;   // tolerância em alta velocidade

    double velNorm = MathUtil.clamp(Math.abs(velocidade) / 60.0, 0.0, 1.0);

    return MathUtil.interpolate(margemMin, margemMax, velNorm);
    }

    public boolean estaNoAlvo(double alvo) {
        return Math.abs(getAngulo() - alvo) <= MARGEM_ERRO;
    }

    /* ================= ESTADOS ================= */

    private void setEstado(Estado novo) {
        if (estado == novo) return;

        if (estado == Estado.PERFIL || estado == Estado.MANUAL) {
            motor.stopMotor();
        }

        estado = novo;

        if (estado == Estado.HOLD) {
            anguloHold = goal.position;
        }

        if (estado == Estado.FALHA) {
            motor.stopMotor();
        }
    }

    /* ================= COMANDOS ================= */

    public void moverParaPreset(AnguloPreset preset) {
        moverParaAngulo(preset.graus);
    }

    public void moverParaAngulo(double graus) {
    graus = MathUtil.clamp(graus, LIMITE_INFERIOR, LIMITE_SUPERIOR);

    if (estado == Estado.HOLD && estaNoAlvo(graus)) {
        return;
    }

    if (estado == Estado.PERFIL && Math.abs(alvoAtual - graus) <= MARGEM_ERRO) {
        return;
    }

    alvoAtual = graus;
    goal = new TrapezoidProfile.State(graus, 0.0);
    tempoSemMovimento = 0.0;
    setEstado(Estado.PERFIL);
}

    public void controleManual(double output) {
        manualOutput = MathUtil.clamp(output, -0.4, 0.4);
        setEstado(Estado.MANUAL);
    }

    public void resetFalha() {
        if (estado == Estado.FALHA) {
            tempoSemMovimento = 0.0;
            setEstado(Estado.DESABILITADO);
        }
    }

    /* ================= EXECUÇÕES ================= */

    private void executarPerfil() {
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    setpoint = profile.calculate(DT, setpoint, goal);

    double ffVolts = feedforward.calculate(
        Math.toRadians(setpoint.position),
        Math.toRadians(setpoint.velocity)
    );

    // Remove kS quando estiver quase parado
    if (Math.abs(setpoint.velocity) < 0.5) {
        ffVolts -= Math.signum(ffVolts) * Constants.FFAlinhador.kS;
    }

    pid.setSetpoint(
        grausParaRotacao(setpoint.position),
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffVolts
    );

    double margem = margemDinamica(setpoint.velocity);

        if (Math.abs(goal.position - setpoint.position) <= margem &&
            Math.abs(setpoint.velocity) < 1.5) {
            setEstado(Estado.HOLD);
        }
    }

    private void executarHold() {
    double erro = getAngulo() - anguloHold;

    if (Math.abs(erro) < 0.4) {
        motor.stopMotor();
        return;
    }

    double ffVolts = feedforward.calculate(
        Math.toRadians(anguloHold),
        0.0
    );

    pid.setSetpoint(
        grausParaRotacao(anguloHold),
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffVolts
    );
}

    private void executarManual() {
        motor.set(manualOutput);
    }

    /* ================= FALHA ================= */

    private void detectarFalha() {
        if (estado != Estado.PERFIL) return;

        double anguloAtual = getAngulo();
        double velocidade = (anguloAtual - posicaoAnterior) / DT;
        double erro = Math.abs(goal.position - anguloAtual);

        if (erro > ERRO_TOLERANCIA && Math.abs(velocidade) < VELOCIDADE_MIN) {
            tempoSemMovimento += DT;
        } else {
            tempoSemMovimento = 0.0;
        }

        posicaoAnterior = anguloAtual;

        if (tempoSemMovimento >= TEMPO_MAX_TRAVADO) {
            setEstado(Estado.FALHA);
            SmartDashboard.putString("Angulador/Falha", "Travado");
        }
    }

    /* ================= PERIODIC ================= */

    @Override
    public void periodic() {

        detectarFalha();

        switch (estado) {
            case PERFIL -> executarPerfil();
            case HOLD -> executarHold();
            case MANUAL -> executarManual();
            default -> {}
        }

        SmartDashboard.putNumber("Angulador/Angulo", getAngulo());
        SmartDashboard.putString("Angulador/Estado", estado.name());
    }
}