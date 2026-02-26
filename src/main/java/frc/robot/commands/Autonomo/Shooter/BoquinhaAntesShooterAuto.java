package frc.robot.commands.Autonomo.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;

import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Constantes.ConstantesIndex;

public class BoquinhaAntesShooterAuto extends SequentialCommandGroup {

    private static final double ATRASO_BOQUINHA = 0.8;
    private static final double ATRASO_INDEX = 1.2;
    private static final double TEMPO_TOTAL = 12;

    public BoquinhaAntesShooterAuto(
            Shooter shooter,
            Index boquinha,
            Index index) {

        addCommands(

            // Liga shooter
            new InstantCommand(() -> {
                shooter.setVelocidade(ConstantesShooter.Velocidade.AUTO);
                shooter.atirarFrente();
            }, shooter),

            // Espera antes da boquinha
            new WaitCommand(ATRASO_BOQUINHA),

            // Liga boquinha
            new InstantCommand(() -> {
                boquinha.setVelocidade(ConstantesIndex.VelocidadeIndex.ALTA);
                boquinha.ligarBoquinha(
                    ConstantesIndex.VelocidadeIndex.NORMAL.rpm
                );
            }, boquinha),

            // Espera antes do index
            new WaitCommand(ATRASO_INDEX),

            // Liga index
            new InstantCommand(() -> {
                index.entrarModoForcado(0.45);
            }, index),

            //  Mantem tudo rodando pelo tempo restante
            new WaitCommand(TEMPO_TOTAL - ATRASO_BOQUINHA - ATRASO_INDEX),

            // Para tudo
            new InstantCommand(() -> {
                shooter.parar();
                boquinha.desligarBoquinha();
                index.sairModoForcado();
            }, shooter, boquinha, index)
        );
    }
}