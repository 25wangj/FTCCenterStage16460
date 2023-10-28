package org.firstinspires.ftc.teamcode.control;
import java.util.ArrayList;
public class ProfileChain extends MotionProfile {
    ArrayList<MotionProfile> profiles = new ArrayList<>();
    public ProfileChain(ArrayList<MotionProfile> profiles) {
        for (int i = 0; i < profiles.size(); i++) {
            if (i < profiles.size() - 1 && profiles.get(i).getX(profiles.get(i + 1).ti) != profiles.get(i + 1).xi) {
                throw new IllegalArgumentException("Discontinuous positions");
            } else if (i < profiles.size() - 1 && profiles.get(i).getV(profiles.get(i + 1).ti) != profiles.get(i + 1).vi) {
                throw new IllegalArgumentException("Discontinuous velocities");
            }  else if (i < profiles.size() - 1 && profiles.get(i).tf > profiles.get(i + 1).ti) {
                throw new IllegalArgumentException("Profiles interfere");
            }
            if (profiles.get(i) instanceof ProfileChain) {
                this.profiles.addAll(((ProfileChain) profiles.get(i)).getProfiles());
            } else {
                this.profiles.add(profiles.get(i));
            }
        }
        this.ti = this.profiles.get(0).ti;
        this.xi = this.profiles.get(0).xi;
        this.vi = this.profiles.get(0).vi;
        this.tf = this.profiles.get(this.profiles.size() - 1).tf;
        this.xf = this.profiles.get(this.profiles.size() - 1).xf;
        this.vf = this.profiles.get(this.profiles.size() - 1).vf;
    }
    public ProfileChain(MotionProfile profile) {
        this.profiles.add(profile);
        this.ti = profile.ti;
        this.xi = profile.xi;
        this.vi = profile.vi;
        this.tf = profile.tf;
        this.xf = profile.xf;
        this.vf = profile.vf;
    }
    @Override
    public double getX(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).ti) {
                return profiles.get(i).getX(t);
            }
        }
        return profiles.get(profiles.size() - 1).getX(t);
    }
    @Override
    public double getV(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).ti) {
                return profiles.get(i).getV(t);
            }
        }
        return profiles.get(profiles.size() - 1).getV(t);
    }
    @Override
    public double getA(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).ti) {
                return profiles.get(i).getA(t);
            }
        }
        return profiles.get(profiles.size() - 1).getA(t);
    }
    public ArrayList<MotionProfile> getProfiles() {
        return profiles;
    }
    public void add(MotionProfile newProfile) {
        if (newProfile.tf < tf) {
            throw new IllegalArgumentException("Profiles interfere");
        } else if (newProfile.xi != getX(newProfile.ti)) {
            throw new IllegalArgumentException("Discontinuous positions");
        } else if (newProfile.vi != getV(newProfile.ti)) {
            throw new IllegalArgumentException("Discontinuous velocities");
        }
        profiles.add(newProfile);
    }
}
