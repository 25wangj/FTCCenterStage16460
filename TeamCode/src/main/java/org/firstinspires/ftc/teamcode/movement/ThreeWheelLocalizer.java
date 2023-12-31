package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import org.ejml.simple.SimpleMatrix;
public class ThreeWheelLocalizer implements Localizer {
    private final double EPS = 1e-6;
    private Pose pos = new Pose(0, 0, 0);
    private Pose vel = new Pose(0, 0, 0);
    private Encoder enc1;
    private Encoder enc2;
    private Encoder enc3;
    private double lastTime = Double.NaN;
    private double f;
    private SimpleMatrix invKin;
    private SimpleMatrix integ = new SimpleMatrix(3, 3);
    private SimpleMatrix rot = new SimpleMatrix(3, 3);
    public ThreeWheelLocalizer(Encoder enc1, Encoder enc2, Encoder enc3, Pose p1, Pose p2, Pose p3, double f) {
        this.enc1 = enc1;
        this.enc2 = enc2;
        this.enc3 = enc3;
        this.f = f;
        invKin = new SimpleMatrix(new double[][] {
                {cos(p1.h), sin(p1.h), p1.x * sin(p1.h) - p1.y * cos(p1.h)},
                {cos(p2.h), sin(p2.h), p2.x * sin(p2.h) - p2.y * cos(p2.h)},
                {cos(p3.h), sin(p3.h), p3.x * sin(p3.h) - p3.y * cos(p3.h)}}).invert();
        integ.set(2, 2, 1);
        rot.set(2, 2, 1);
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
        double v1 = f * enc1.getVelocity(time);
        double v2 = f * enc2.getVelocity(time);
        double v3 = f * enc3.getVelocity(time);
        if (v1 == 0 && v2 == 0 && v3 == 0) {
            vel = new Pose(0, 0, 0);
        } else if (!Double.isNaN(lastTime)) {
            double dt = time - lastTime;
            SimpleMatrix local = invKin.mult(new SimpleMatrix(new double[][]{{v1 * dt}, {v2 * dt}, {v3 * dt}}));
            rot.set(0, 0, cos(pos.h));
            rot.set(0, 1, -sin(pos.h));
            rot.set(1, 0, sin(pos.h));
            rot.set(1, 1, cos(pos.h));
            double dA = local.get(2, 0);
            if (abs(dA) < EPS) {
                integ.set(0, 0, 1 - dA * dA / 6);
                integ.set(0, 1, -dA / 2);
                integ.set(1, 0, dA / 2);
                integ.set(1, 1, 1 - dA * dA / 6);
            } else {
                integ.set(0, 0, sin(dA) / dA);
                integ.set(0, 1, (cos(dA) - 1) / dA);
                integ.set(1, 0, (1 - cos(dA)) / dA);
                integ.set(1, 1, sin(dA) / dA);
            }
            SimpleMatrix dPos = rot.mult(integ.mult(local));
            SimpleMatrix nVel = rot.mult(local).scale(1 / dt);
            pos = new Pose(pos.x + dPos.get(0, 0), pos.y + dPos.get(1, 0), pos.h + dPos.get(2, 0));
            vel = new Pose(nVel.get(0, 0), nVel.get(1, 0), nVel.get(2, 0));
        }
        lastTime = time;
    }
}
