package org.firstinspires.ftc.teamcode.control;
import org.ejml.simple.SimpleMatrix;
public class KalmanEstimate {
    public final SimpleMatrix est;
    public final SimpleMatrix obs;
    public final SimpleMatrix cov;
    public final long time;
    public KalmanEstimate(SimpleMatrix est, SimpleMatrix obs, SimpleMatrix cov, long time) {
        this.est = est;
        this.obs = obs;
        this.cov = cov;
        this.time = time;
    }
}
