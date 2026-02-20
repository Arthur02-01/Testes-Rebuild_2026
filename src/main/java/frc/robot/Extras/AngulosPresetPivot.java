package frc.robot.Extras;

public enum AngulosPresetPivot {

    BAIXO(Math.toRadians(10.0)),
    ALTO(Math.toRadians(198.50));

    public final double rad;

    AngulosPresetPivot(double rad) {
        this.rad = rad;
    }
}
