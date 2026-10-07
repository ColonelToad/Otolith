#include "otolith/log.hpp"
#include "otolith/estimate_log.hpp"
#include "otolith/fusion.hpp"
#include "otolith/leg_kin.hpp"
#include <iostream>

int main(int argc, char** argv){
    if(argc<3){
        std::cerr<<"usage: fuse_log <in.otlg> <out.estm> [dt_override] [--no-leg-update] [--sigma-leg <v>]\n";
        std::cerr<<"  --no-leg-update: predict-only dead reckoning (IMU, no contact updates)\n";
        std::cerr<<"  --sigma-leg <v>: override cfg.sigma_leg_vel (Rmat = v^2*I).\n";
        std::cerr<<"  --robot <go2|g1>: select the leg descriptor.\n";
        std::cerr<<"  --sigma-leg-from-robot: use the measured value for that robot.\n";
        std::cerr<<"      Exists so the filter's PERFORMANCE can be swept against the\n";
        std::cerr<<"      assumption. The existing --sigma-leg study on fuse_update_study\n";
        std::cerr<<"      measures the fixed-point numerical FLOOR instead (~0.0055), which\n";
        std::cerr<<"      says nothing about how much measurement noise the filter tolerates.\n";
        std::cerr<<"      See docs/V06_SIGMA_LEG.md.\n";
        return 2;
    }
    std::string in=argv[1], out=argv[2];
    bool no_leg_update = false;
    double sigma_leg = -1.0;
    // Default -1 means "leave the config alone". A robot whose measured sigma_leg
    // differs from the Go2's 0.3 must be able to override it, which is the whole
    // point of P4: sigma_leg scales as 1/dt, so 0.3 is not a property of legged
    // robots, it is a property of one robot at one sample rate.
    // -1 means "use the robot's measured sigma_leg, not a constant". See
    // SigmaLegModel in leg_kin.hpp for why it is a formula and not a number.
    bool sigma_leg_from_robot = false;
    const char* robot_name = "go2";
    try{
        auto lf = otolith::read_log(in);
        double dt = lf.header.dt;
        for(int a=3;a<argc;++a){
            std::string arg=argv[a];
            if(arg=="--no-leg-update"){
                no_leg_update = true;
            } else if(arg=="--sigma-leg" && a+1<argc){
                // consumed with its value, so the positional dt parser below
                // never tries to stod the number
                sigma_leg = std::stod(argv[++a]);
            } else if(arg=="--robot" && a+1<argc){
                robot_name = argv[++a];
            } else if(arg=="--sigma-leg-from-robot"){
                sigma_leg_from_robot = 1.0;
            } else {
                dt = std::stod(arg);
            }
        }

        otolith::FusionConfig fcfg;
        if(sigma_leg > 0.0){
            fcfg.sigma_leg_vel = sigma_leg;
            std::cerr << "sigma_leg_vel = " << sigma_leg
                      << " -> Rmat = " << sigma_leg*sigma_leg << "*I\n";
        }
        otolith::RobotSpec spec;
        try {
            spec = otolith::robot_spec(robot_name);
        } catch (const std::exception& e) {
            std::cerr << e.what() << "\n";
            return 2;
        }
        otolith::FusionEKF ekf(fcfg);
        if (spec.n_legs != 4 || spec.dof_per_leg != 3) {
            // An explicit --sigma-leg always wins over the model.
            if (sigma_leg_from_robot && sigma_leg < 0.0)
                fcfg.sigma_leg_vel = spec.sigma_leg.at(dt);
            ekf.set_robot(spec);
            std::cerr << "robot " << spec.name << ": " << spec.n_legs
                      << " legs x " << spec.dof_per_leg << " dof";
            if (sigma_leg_from_robot && sigma_leg < 0.0)
                std::cerr << ", sigma_leg_vel = " << fcfg.sigma_leg_vel
                          << " m/s (sigma_q=" << spec.sigma_leg.sigma_q
                          << " rad, L=" << spec.sigma_leg.lever_nominal
                          << " m, k=" << spec.sigma_leg.k
                          << ", motion=" << spec.sigma_leg.body_motion
                          << ", dt=" << dt << ")";
            std::cerr << "\n";
        }
        // Init from first GT to avoid huge initial transient dominating RMSE
        if(!lf.rows.empty()){
            auto &r0 = lf.rows[0];
            otolith::FusionState s = ekf.state();
            s.q = Eigen::Quaterniond(r0.gt_quat[0], r0.gt_quat[1], r0.gt_quat[2], r0.gt_quat[3]);
            s.q.normalize();
            s.p = Eigen::Vector3d(r0.gt_pos[0], r0.gt_pos[1], r0.gt_pos[2]);
            s.v = Eigen::Vector3d(r0.gt_vel[0], r0.gt_vel[1], r0.gt_vel[2]);
            s.bg.setZero(); s.ba.setZero();
            ekf.set_state(s);
        }

        std::vector<otolith::EstRowV2> est;
        est.reserve(lf.rows.size());
        for(auto &row: lf.rows){
            Eigen::Vector3d gyro(row.gyro[0], row.gyro[1], row.gyro[2]);
            Eigen::Vector3d accel(row.accel[0], row.accel[1], row.accel[2]);
            ekf.predict(dt, gyro, accel);

            if(!no_leg_update){
                Eigen::Matrix<double,12,1> qj;
                for(int i=0;i<12;++i) qj[i]=row.qj[i];
                std::array<uint8_t,4> contacts{row.contacts[0],row.contacts[1],row.contacts[2],row.contacts[3]};
                ekf.update_legs(qj, contacts, gyro, dt);
            }

            auto st = ekf.state();
            otolith::EstRowV2 er{};
            er.base.t = row.t;
            er.base.p[0]=st.p.x(); er.base.p[1]=st.p.y(); er.base.p[2]=st.p.z();
            er.base.quat[0]=st.q.w(); er.base.quat[1]=st.q.x(); er.base.quat[2]=st.q.y(); er.base.quat[3]=st.q.z();
            er.base.v[0]=st.v.x(); er.base.v[1]=st.v.y(); er.base.v[2]=st.v.z();
            er.base.bg[0]=st.bg.x(); er.base.bg[1]=st.bg.y(); er.base.bg[2]=st.bg.z();
            er.base.ba[0]=st.ba.x(); er.base.ba[1]=st.ba.y(); er.base.ba[2]=st.ba.z();
            // P row-major 15x15
            for(int r=0;r<15;++r) for(int c=0;c<15;++c) er.P[r*15+c]=st.P(r,c);
            est.push_back(er);
        }
        otolith::write_estimate_v2(out, dt, est);
        std::cout<<"fuse_log: "<<lf.rows.size()<<" rows -> "<<est.size()<<" estimates, dt="<<dt
                 <<(no_leg_update?" (predict-only, no leg updates)":"")<<"\n";
        return 0;
    }catch(const std::exception& e){
        std::cerr<<"fuse_log error: "<<e.what()<<"\n";
        return 1;
    }
}
