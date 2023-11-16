package org.firstinspires.ftc.teamcode.control;
public class AsymConstraints {
    public final double vm;
    public final double ai;
    public final double af;
    public AsymConstraints(double vm, double ai, double af) {
        this.vm = vm;
        this.ai = ai;
        this.af = af;
    }
    public AsymConstraints scaleX(double f) {
        return new AsymConstraints(vm * f, ai * f, af * f);
    }
    public AsymConstraints scaleT(double f) {
        return new AsymConstraints(vm * f, ai * f * f, af * f * f);
    }
}
