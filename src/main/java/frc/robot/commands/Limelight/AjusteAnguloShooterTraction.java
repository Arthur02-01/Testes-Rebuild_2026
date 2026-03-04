package frc.robot.commands.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constantes.ConstantesAngulador;
import frc.robot.Constantes.ConstantesLimelight.LimelightConstants;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Traction;

public class AjusteAnguloShooterTraction extends Command {

    private final Limelight limelight;
    private final Traction traction;
    private final Shooter shooter;
    private final Angulador angulador;

    private static final double KP_ROT = 0.055;
    private static final double MAX_ROT = 0.60;

    private static final double DIST_MIN = 1.55;
    private static final double DIST_MAX = 9.45;

    private static final double RPM_MIN = 2150.0;
    private static final double RPM_MAX = 3350.0;

    private static final double OFFSET_PROFUNDIDADE_HUB = 0.12;
    private static final double OFFSET_ALTURA_HUB = 1.03;

    private static final double TEMPO_ENTRE_AJUSTES = 0.25;

    private double ultimoAjuste = 0.0;

    public AjusteAnguloShooterTraction(
        Limelight limelight,
        Traction traction,
        Shooter shooter,
        Angulador angulador
    ) {
        this.limelight = limelight;
        this.traction = traction;
        this.shooter = shooter;
        this.angulador = angulador;

        addRequirements(traction, shooter, angulador);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.ligarLED();
        ultimoAjuste = 0.0;
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            traction.stop();
            return;
        }

        double erroX = limelight.getTxShooter();

        if (Math.abs(erroX) > LimelightConstants.DEADZONE_TX_GRAUS) {

            double rot = erroX * KP_ROT;
            rot = MathUtil.clamp(rot, -MAX_ROT, MAX_ROT);

            traction.arcadeMode(0.0, rot);
            return; 
        }

        traction.stop();

        double distancia =
            limelight.getDistanciaFiltrada()
            + OFFSET_PROFUNDIDADE_HUB;

        distancia = MathUtil.clamp(distancia, DIST_MIN, DIST_MAX);

        double rpmCalculado =
            RPM_MIN +
            (distancia - DIST_MIN) *
            (RPM_MAX - RPM_MIN) /
            (DIST_MAX - DIST_MIN);

        double alturaAlvo =
            LimelightConstants.ALTURA_TAG_METROS
            + OFFSET_ALTURA_HUB
            - LimelightConstants.ALTURA_CAMERA_METROS;

        double anguloGraus = Math.toDegrees(
            Math.atan2(alturaAlvo, distancia)
        );

        anguloGraus = MathUtil.clamp(
            anguloGraus,
            ConstantesAngulador.LIMITE_INFERIOR,
            ConstantesAngulador.LIMITE_SUPERIOR
        );

        double agora = Timer.getFPGATimestamp();

        if (agora - ultimoAjuste >= TEMPO_ENTRE_AJUSTES) {
            shooter.setRpmDireto(rpmCalculado);
            angulador.moverParaAngulo(anguloGraus);
            ultimoAjuste = agora;
        }
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
        limelight.desligarLED();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}