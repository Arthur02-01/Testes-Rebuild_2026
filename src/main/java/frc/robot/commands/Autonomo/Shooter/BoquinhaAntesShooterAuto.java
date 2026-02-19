package frc.robot.commands.Autonomo.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;

import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Constantes.ConstantesIndex;

public class BoquinhaAntesShooterAuto extends SequentialCommandGroup {

    private static final double ATRASO_SHOOTER = 1.2;
    private static final double TEMPO_TOTAL = 3.0;

    public BoquinhaAntesShooterAuto(Shooter shooter, Index boquinha) {

        addCommands(

            // Liga apenas a boquinha
            new InstantCommand(() -> {
                boquinha.setVelocidade(ConstantesIndex.VelocidadeIndex.NORMAL);
                boquinha.ligarBoquinha(ConstantesIndex.VelocidadeIndex.NORMAL.rpm);
            }, boquinha),

            // Espera antes de ligar shooter
            new WaitCommand(ATRASO_SHOOTER),

            // Liga shooter mantendo boquinha rodando
            new ParallelCommandGroup(

                // Shooter rodando continuamente
                new RunCommand(() -> {
                    shooter.setVelocidade(ConstantesShooter.Velocidade.NORMAL);
                    shooter.atirarTras();
                }, shooter),

                // Mantém boquinha ativa
                new RunCommand(() -> {
                    boquinha.ligarBoquinha(ConstantesIndex.VelocidadeIndex.NORMAL.rpm);
                }, boquinha)

            ).withTimeout(TEMPO_TOTAL - ATRASO_SHOOTER),

            // Para tudo no final
            new InstantCommand(() -> {
                shooter.parar();
                boquinha.desligarBoquinha();
            }, shooter, boquinha)
        );
    }
}