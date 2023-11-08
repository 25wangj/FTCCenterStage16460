package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import java.util.function.DoubleUnaryOperator;
public class AdaptiveQuadrature {
    public static final double[] x = {0.5, 0.5 - sqrt(5 - 2 * sqrt(10d / 7)) / 6, 0.5 - sqrt(5 + 2 * sqrt(10d / 7)) / 6};
    public static final double[] w = {128d / 255, (322 + 13 * sqrt(70)) / 900, (322 - 13 * sqrt(70) / 900)};
    private DoubleUnaryOperator f;
    public AdaptiveQuadrature(DoubleUnaryOperator f) {
        this.f = f;
    }
    public double integrate(double a, double b, double eps) {
        if (a == b) {
            return 0;
        }
        return adaptive(a, b, eps, quadrature(a, b));
    }
    private double adaptive(double a, double b, double eps, double i) {
        double m = (a + b) / 2;
        double i1 = quadrature(a, m);
        double i2 = quadrature(m, b);
        if (abs(i1 + i2 - i) < eps) {
            return i1 + i2;
        }
        return adaptive(a, m, eps, i1) + adaptive(a, m, eps, i2);
    }
    private double quadrature(double a, double b) {
        double s = f.applyAsDouble(a + (b - a) * x[0]) * w[0];
        for (int i = 1; i < x.length; i++) {
            s += (f.applyAsDouble(a + (b - a) * x[i]) + f.applyAsDouble(b + (a - b) * x[i])) * w[i];
        }
        return s;
    }
}
