package frc.robot.subsystems;

public enum AnguloPreset {

    BAIXO(10.0),
    CENTRAL(25.0),
    ALTO(50.0);

    public final double graus;

    AnguloPreset(double graus) {
        this.graus = graus;
    }
}