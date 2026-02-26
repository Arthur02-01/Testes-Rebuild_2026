package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constantes.ConstantesAngulador;
import frc.robot.Constantes.ConstantesLimelight.LimelightConstants;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoAimShooter extends Command {

    private final Angulador angulador;
    private final Shooter shooter;
    private final Limelight limelight;

    private static final double DIST_MIN = 1.05;  
    private static final double RPM_MIN  = 2150.0;

    private static final double DIST_MAX = 8.45;
    private static final double RPM_MAX  = 2850.0;

    public AutoAimShooter(
        Angulador angulador,
        Shooter shooter,
        Limelight limelight
    ) {
        this.angulador = angulador;
        this.shooter = shooter;
        this.limelight = limelight;

        addRequirements(angulador, shooter);
    }

    @Override
    public void initialize() {
        shooter.parar();
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            shooter.parar();
            return;
        }

        double distancia = limelight.getDistanciaFiltrada();

        /* ================= ANGULADOR ================= */

        double deltaAltura =
              (LimelightConstants.ALTURA_TAG_METROS + 0.40)
            - LimelightConstants.ALTURA_CAMERA_METROS;

        double anguloRad = Math.atan(deltaAltura / distancia);
        double anguloGraus = Math.toDegrees(anguloRad)
                + LimelightConstants.ANGULO_CAMERA_EFETIVO_GRAUS;

        anguloGraus = Math.max(
            ConstantesAngulador.LIMITE_INFERIOR,
            Math.min(ConstantesAngulador.LIMITE_SUPERIOR, anguloGraus)
        );

        angulador.moverParaAngulo(anguloGraus);

        /* ================= SHOOTER ================= */

        distancia = Math.max(DIST_MIN, Math.min(DIST_MAX, distancia));

        // Interpolação linear
        double rpm =
            RPM_MIN +
            (distancia - DIST_MIN) *
            (RPM_MAX - RPM_MIN) /
            (DIST_MAX - DIST_MIN);

        shooter.setRpmDireto(rpm);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.parar();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}