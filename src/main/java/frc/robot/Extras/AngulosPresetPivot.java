package frc.robot.Extras;

public enum AngulosPresetPivot {

    BAIXO(Math.toRadians(5.0)),
    ALTO(Math.toRadians(150.0));

    public final double rad;

    AngulosPresetPivot(double rad) {
        this.rad = rad;
    }
}
