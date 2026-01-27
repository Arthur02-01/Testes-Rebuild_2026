package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable table;

    private double txFiltrado = 0.0;
    private double distFiltrada = 0.0;

    private static final double ALPHA = 0.2; 

// Alturas em metros
private static final double ALTURA_CAMERA = 0.35;
private static final double ALTURA_TAG = 1.135;

// Ângulo EFETIVO calibrado (não o físico)
private static final double ANGULO_CAMERA_EFETIVO = 20.4; // exemplo

// Distância da câmera até o bumper (em metros)
private static final double OFFSET_CAMERA_BUMPER = 0.25;
 

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean temAlvo() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0.0);
    }
    public double getTxFiltrado() {
        double tx = getTx();
        txFiltrado = (1 - ALPHA) * txFiltrado + ALPHA * tx;
        return txFiltrado;
    }

    public double getDistanciaAprilTag() {
if (!temAlvo()) return -1;


double ty = getTy();
double anguloTotalGraus = ANGULO_CAMERA_EFETIVO + ty;
double anguloTotalRad = Math.toRadians(anguloTotalGraus);


double distanciaCamera =
(ALTURA_TAG - ALTURA_CAMERA) / Math.tan(anguloTotalRad);


double distanciaBumper = distanciaCamera - OFFSET_CAMERA_BUMPER;
return Math.max(distanciaBumper, 0.0);
}


    public double getDistanciaFiltrada() {
        double d = getDistanciaAprilTag();
        distFiltrada = (1 - ALPHA) * distFiltrada + ALPHA * d;
        return distFiltrada;
    }

    public void ligarLED() {
        table.getEntry("ledMode").setNumber(3);
    }

    public void desligarLED() {
        table.getEntry("ledMode").setNumber(1);
    }

    public int getAprilTagID() {
        return (int) table.getEntry("tid").getDouble(-1);
    }
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight/Tem Alvo", temAlvo());
        SmartDashboard.putNumber("Limelight/tx (graus)", getTx());
        SmartDashboard.putNumber("Limelight/ty (graus)", getTy());
        SmartDashboard.putNumber("Limelight/Distancia (m)", getDistanciaAprilTag());
        SmartDashboard.putNumber("limelight/Distancia Filtrada (m)", getDistanciaFiltrada());
    }
}
