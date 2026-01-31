package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
//import frc.robot.commands.Shooter.ShooterVelocidade;

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

    double d = limelight.getDistanciaFiltrada();

    if (d >= 2.30) {
        shooter.atirarFrente(ConstantesShooter.Velocidade.TURBO);

    } else if (d >= 1.45) {
        shooter.atirarFrente(ConstantesShooter.Velocidade.ALTA);

    } else {
        shooter.atirarFrente(ConstantesShooter.Velocidade.MEDIA);
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
