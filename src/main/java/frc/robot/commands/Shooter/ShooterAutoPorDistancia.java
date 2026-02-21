package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class ShooterAutoPorDistancia extends Command {

    private final Shooter shooter;
    private final Limelight limelight;

    public ShooterAutoPorDistancia(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
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

        if (distancia >= 2.30) {
            shooter.setVelocidade(ConstantesShooter.Velocidade.TURBO);
        } else if (distancia >= 1.45) {
            shooter.setVelocidade(ConstantesShooter.Velocidade.ALTA);
        } else {
            shooter.setVelocidade(ConstantesShooter.Velocidade.MEDIA);
        }

        // garante estado correto sem toggle
        shooter.setAlimentando(false);
        shooter.atirarFrente();
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