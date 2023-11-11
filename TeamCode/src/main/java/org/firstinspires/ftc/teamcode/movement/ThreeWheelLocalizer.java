package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
public class ThreeWheelLocalizer implements Localizer {
    private final double EPS = 1e-6;
    private Pose pos = new Pose(0, 0, 0);
    private Pose vel = new Pose(0, 0, 0);
    private Encoder enc1;
    private Encoder enc2;
    private Encoder enc3;
    private double last1 = Double.NaN;
    private double last2 = Double.NaN;
    private double last3 = Double.NaN;
    private double lastTime = Double.NaN;
    private double f;
    private DecompositionSolver invKin;
    public ThreeWheelLocalizer(Encoder enc1, Encoder enc2, Encoder enc3, Pose p1, Pose p2, Pose p3, double f) {
        this.enc1 = enc1;
        this.enc2 = enc2;
        this.enc3 = enc3;
        this.f = f;
        invKin = new LUDecomposition(new Array2DRowRealMatrix(new double[][] {
                {cos(p1.h), sin(p1.h), p1.x * sin(p1.h) - p1.y * cos(p1.h)},
                {cos(p2.h), sin(p2.h), p2.x * sin(p2.h) - p2.y * cos(p2.h)},
                {cos(p3.h), sin(p3.h), p3.x * sin(p3.h) - p3.y * cos(p3.h)}})).getSolver();
        if (!invKin.isNonSingular()) {
            throw new IllegalArgumentException("Pose cannot be uniquely determined");
        }
    }
    public ThreeWheelLocalizer(Encoder par1, Encoder par2, Encoder perp, double parDist, double perpDist, double conv) {
        this(par1, par2, perp, new Pose(0, parDist / 2, 0), new Pose(0, -parDist / 2, 0), new Pose(perpDist, 0, PI / 2), conv);
    }
    @Override
    public Pose pos() {
        return pos;
    }
    @Override
    public Pose vel() {
        return vel;
    }
    @Override
    public void setPose(Pose p) {
        pos = p;
        vel = new Pose(0, 0, 0);
    }
    @Override
    public void update(double time) {
        double p1 = f * enc1.getPosition();
        double p2 = f * enc2.getPosition();
        double p3 = f * enc3.getPosition();
        if (p1 == last1 && p2 == last2 && p3 == last3) {
            vel = new Pose(0, 0, 0);
        } else if (!Double.isNaN(last1)) {
            RealMatrix local = invKin.solve(new Array2DRowRealMatrix(new double[][]{{p1 - last1}, {p2 - last2}, {p3 - last3}}));
            RealMatrix rot = new Array2DRowRealMatrix(new double[][] {
                    {cos(pos.h), -sin(pos.h), 0}, {sin(pos.h), cos(pos.h), 0}, {0, 0, 1}});
            double dA = local.getEntry(2, 0);
            RealMatrix integ;
            if (abs(dA) < EPS) {
                integ = new Array2DRowRealMatrix(new double[][] {
                        {1 - dA * dA / 6, -dA / 2, 0}, {dA / 2, 1 - dA * dA / 6, 0}, {0, 0, 1}});
            } else {
                integ = new Array2DRowRealMatrix(new double[][] {
                        {sin(dA) / dA, (cos(dA) - 1) / dA, 0}, {(1 - cos(dA)) / dA, sin(dA) / dA, 0}, {0, 0, 1}});
            }
            RealMatrix dPos = rot.multiply(integ.multiply(local));
            RealMatrix nVel = rot.multiply(local).scalarMultiply(1 / (time - lastTime));
            pos = new Pose(pos.x + dPos.getEntry(0, 0), pos.y + dPos.getEntry(1, 0),
                    pos.h + dPos.getEntry(2, 0));
            vel = new Pose(nVel.getEntry(0, 0), nVel.getEntry(1, 0), nVel.getEntry(2, 0));
        }
        last1 = p1;
        last2 = p2;
        last3 = p3;
        lastTime = time;
    }
}
