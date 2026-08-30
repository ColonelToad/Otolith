#include "otolith/log.hpp"
#include "otolith/estimate_log.hpp"
#include "otolith/fusion.hpp"
#include <iostream>

int main(int argc, char** argv){
    if(argc<3){
        std::cerr<<"usage: fuse_log <in.otlg> <out.estm> [dt_override]\n";
        return 2;
    }
    std::string in=argv[1], out=argv[2];
    try{
        auto lf = otolith::read_log(in);
        double dt = lf.header.dt;
        if(argc>=4) dt = std::stod(argv[3]);

        otolith::FusionEKF ekf;
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

            Eigen::Matrix<double,12,1> qj;
            for(int i=0;i<12;++i) qj[i]=row.qj[i];
            std::array<uint8_t,4> contacts{row.contacts[0],row.contacts[1],row.contacts[2],row.contacts[3]};
            ekf.update_legs(qj, contacts, gyro, dt);

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
        std::cout<<"fuse_log: "<<lf.rows.size()<<" rows -> "<<est.size()<<" estimates, dt="<<dt<<"\n";
        return 0;
    }catch(const std::exception& e){
        std::cerr<<"fuse_log error: "<<e.what()<<"\n";
        return 1;
    }
}
