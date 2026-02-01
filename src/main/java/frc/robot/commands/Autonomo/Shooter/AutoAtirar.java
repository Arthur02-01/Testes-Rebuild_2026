package frc.robot.commands.Autonomo.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.commands.Shooter.AtivarFrenteShooter;
import frc.robot.commands.Shooter.PararShooter;
import frc.robot.commands.Shooter.ShooterVelocidade;
import frc.robot.subsystems.Shooter;

public class AutoAtirar extends SequentialCommandGroup {

    public AutoAtirar(Shooter shooter) {

        addCommands(
            // garante estado limpo
            new PararShooter(shooter),

            // define velocidade (NÃO gira)
            new ShooterVelocidade(
                shooter,
                ConstantesShooter.Velocidade.TURBO
            ),

            // inicia giro para frente
            new AtivarFrenteShooter(shooter),

            // tempo para ganhar RPM
            new WaitCommand(10),

            // tempo extra para disparo
            new WaitCommand(0.5),

            // para tudo no final
            new PararShooter(shooter)
        );
    }
}
