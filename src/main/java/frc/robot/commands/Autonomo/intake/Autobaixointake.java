package frc.robot.commands.Autonomo.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeFloor;

public class Autobaixointake extends SequentialCommandGroup {

    public Autobaixointake(IntakeFloor intake) {

        addCommands(

            new InstantCommand(intake::IntakeOn, intake)
        );
    }
}