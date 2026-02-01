package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.StatesMachines.StateMachineShooter;

public class ShooterAutoPorDistancia extends Command {

    private final Shooter shooter;
    private final Limelight limelight;

    public ShooterAutoPorDistancia(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            shooter.parar();
            return;
        }

        double M = limelight.getDistanciaFiltrada();

        if (M >= 2.30) {
            shooter.setVelocidade(ConstantesShooter.Velocidade.TURBO);
        } else if (M >= 1.45) {
            shooter.setVelocidade(ConstantesShooter.Velocidade.ALTA);
        } else {
            shooter.setVelocidade(ConstantesShooter.Velocidade.MEDIA);
        }

        // garante que está girando para frente
        if (!shooter.estaAtivo()) {
            shooter.atirarFrente();
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
