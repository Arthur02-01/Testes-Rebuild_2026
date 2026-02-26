package frc.robot.commands.Autonomo.Tracao;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Traction;
import frc.robot.commands.Autonomo.intake.Autobaixointake;
import frc.robot.subsystems.IntakeFloor;

@SuppressWarnings ("unused")
    public class AutoAndarEColetar extends ParallelDeadlineGroup {

    public AutoAndarEColetar(Traction traction, IntakeFloor intake) {

        super(

            new AndarEncoder(traction, -0.7, 0.2),
            new Autobaixointake(intake),


            new StartEndCommand(
                () -> {
                    intake.forcarHold();
                    intake.IntakeReverse();
                },
                intake::PararIntake,
                intake
            )

        );
    }
}