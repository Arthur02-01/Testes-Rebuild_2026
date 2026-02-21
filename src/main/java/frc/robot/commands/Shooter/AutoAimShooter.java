package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constantes.ConstantesAngulador;
import frc.robot.Constantes.ConstantesLimelight.LimelightConstants;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoAimShooter extends Command {

    private final Angulador angulador;
    private final Shooter shooter;
    private final Limelight limelight;

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
              LimelightConstants.ALTURA_TAG_METROS
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

        if (distancia >= 7.30) {
            shooter.setVelocidade(ConstantesShooter.Velocidade.TURBO);
        } else if (distancia >= 5.40) {
            shooter.setVelocidade(ConstantesShooter.Velocidade.ALTA);
        } else {
            shooter.setVelocidade(ConstantesShooter.Velocidade.MEDIA);
        }
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