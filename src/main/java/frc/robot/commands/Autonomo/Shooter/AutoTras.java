package frc.robot.commands.Autonomo.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.commands.Shooter.AtivarAtrasShooter;
import frc.robot.commands.Shooter.PararShooter;
import frc.robot.subsystems.Shooter;

@SuppressWarnings("unused")
public class AutoTras extends SequentialCommandGroup {

    public AutoTras(Shooter shooter) {

        addCommands(
            // Garante estado conhecido
            new PararShooter(shooter),

            // Liga o shooter PARA TRÁS já com velocidade
            new AtivarAtrasShooter(
                shooter
            ),

            // Tempo para pegar giro
            new WaitCommand(3),

            // Tempo para a bola entrar
            new WaitCommand(0.5),

            // Desliga no final
            new PararShooter(shooter)
        );
    }
}
