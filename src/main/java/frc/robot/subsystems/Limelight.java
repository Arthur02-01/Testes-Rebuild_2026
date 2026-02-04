package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    
    private final NetworkTable table;

    private double txFiltrado = 0.0;
    private double distFiltrada = 0.0;
    private double UltimaDistanciaValida = 0.0;
    private double ultimoDashboard = 0.0;  
    
    private static final double ALPHA = 0.2; 
    private static final double Tan_Angulo_Minimo = 1e-3;
 
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

    public double getTxAlvo () {
        return temAlvo() ? getTx() : Double.NaN;
    }

    public double getTyAlvo () {
        return temAlvo() ? getTy() : Double.NaN;
    }


    public double getTxFiltrado() {
        double tx = getTx();
        txFiltrado = (1 - ALPHA) * txFiltrado + ALPHA * tx;
        return txFiltrado;
    }

    public double getDistanciaAprilTag() {
        if (!temAlvo()) {
            return Double.NaN;
        }

double ty = getTy();
double anguloTotalGraus = Constants.LimelightConstants.ANGULO_CAMERA_EFETIVO_GRAUS + ty;
double anguloTotalRad = Math.toRadians(anguloTotalGraus);

double tangente = Math.tan(anguloTotalRad);
if (Math.abs(tangente) < Tan_Angulo_Minimo) {
    DriverStation.reportWarning(
        "Limelight: tan(angulo) muito pequeno, distancia invalida. Angulo=" + anguloTotalGraus,
        false
    );
    return UltimaDistanciaValida == 0.0 ? Double.NaN : UltimaDistanciaValida;
}


double distanciaCamera =
(Constants.LimelightConstants.ALTURA_TAG_METROS - Constants.LimelightConstants.ALTURA_CAMERA_METROS) / tangente;


double distanciaBumper = distanciaCamera - Constants.LimelightConstants.OFFSET_CAMERA_BUMPER_METROS;
return Math.max(distanciaBumper, 0.0);
}


    public double getDistanciaFiltrada() {
        double d = getDistanciaAprilTag();
        if (Double.isNaN(d)) {
            return UltimaDistanciaValida;
        }
        distFiltrada = (1 - ALPHA) * distFiltrada + ALPHA * d;
        UltimaDistanciaValida = distFiltrada;
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
        double agora = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (agora - ultimoDashboard >= 0.2) {
        ultimoDashboard = agora;
        SmartDashboard.putBoolean("Limelight/Tem Alvo", temAlvo());
        SmartDashboard.putNumber("Limelight/tx (graus)", getTx());
        SmartDashboard.putNumber("Limelight/ty (graus)", getTy());
        SmartDashboard.putNumber("Limelight/Distancia (m)", getDistanciaAprilTag());
        SmartDashboard.putNumber("limelight/Distancia Filtrada (m)", getDistanciaFiltrada());
        }
    }
}
