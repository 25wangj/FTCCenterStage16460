package org.firstinspires.ftc.teamcode.control;
import java.util.ArrayList;
import java.util.List;
public class ProfileChain extends MotionProfile {
    List<MotionProfile> profiles = new ArrayList<>();
    public ProfileChain(List<MotionProfile> profiles) {
        for (int i = 0; i < profiles.size(); i++) {
            if (i < profiles.size() - 1 && profiles.get(i).pos(profiles.get(i + 1).ti) != profiles.get(i + 1).xi) {
                throw new IllegalArgumentException("Discontinuous positions");
            } else if (i < profiles.size() - 1 && profiles.get(i).vel(profiles.get(i + 1).ti) != profiles.get(i + 1).vi) {
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
    public double pos(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).ti) {
                return profiles.get(i).pos(t);
            }
        }
        return profiles.get(profiles.size() - 1).pos(t);
    }
    @Override
    public double vel(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).ti) {
                return profiles.get(i).vel(t);
            }
        }
        return profiles.get(profiles.size() - 1).vel(t);
    }
    @Override
    public double accel(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).ti) {
                return profiles.get(i).accel(t);
            }
        }
        return profiles.get(profiles.size() - 1).accel(t);
    }
    public List<MotionProfile> getProfiles() {
        return profiles;
    }
    public ProfileChain add(MotionProfile newProfile) {
        if (newProfile.tf < tf) {
            throw new IllegalArgumentException("Profiles interfere");
        } else if (newProfile.xi != pos(newProfile.ti)) {
            throw new IllegalArgumentException("Discontinuous positions");
        } else if (newProfile.vi != vel(newProfile.ti)) {
            throw new IllegalArgumentException("Discontinuous velocities");
        }
        profiles.add(newProfile);
        return this;
    }
}
