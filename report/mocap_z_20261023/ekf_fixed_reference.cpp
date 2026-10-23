// Local-only PX4 1.16 estimator library probe. No network, ROS, FC or actuator IO.
#include <cstdio>
#include <cmath>
#include <memory>
#include "EKF/ekf.h"

using namespace estimator;
static bool run_case(PositionFrame frame, float yaw, bool inject_mag, float z=-.3f)
{
    auto ekf = std::make_unique<Ekf>();
    auto *p = ekf->getParamHandle();
    p->mag_fusion_type = 5;
    p->gnss_ctrl = 0;
    p->ev_ctrl = 11;
    p->ev_delay_ms = 0.f;
    p->baro_ctrl = 1;
    p->height_sensor_ref = static_cast<int32_t>(HeightSensor::EV);
    ekf->init(0);
    ekf->set_in_air_status(false);
    ekf->set_vehicle_at_rest(true);
    for (uint64_t t = 4000; t <= 16000000; t += 4000) {
        imuSample imu{};
        imu.time_us = t;
        imu.delta_ang_dt = .004f;
        imu.delta_vel_dt = .004f;
        imu.delta_vel = Vector3f(0.f, 0.f, -9.80665f * .004f);
        ekf->setIMUData(imu);
        if (t % 20000 == 0) {
            baroSample baro{};
            baro.time_us = t;
            baro.hgt = t < 8000000 ? 0.f : .8f;
            ekf->setBaroData(baro);
            if (inject_mag) {
                magSample mag{};
                mag.time_us = t;
                // Deliberately changes field direction while external yaw remains fixed.
                mag.mag = t < 8000000 ? Vector3f(.2f, .0f, .4f) : Vector3f(-.1f, .2f, .4f);
                ekf->setMagData(mag);
            }
            if (t > 2000000) {
                extVisionSample ev{};
                ev.time_us = t;
                ev.pos = Vector3f(1.2f, -.4f, z);
                ev.vel = Vector3f(NAN, NAN, NAN);
                ev.quat = Quatf(Eulerf(0.f, 0.f, yaw));
                ev.position_var = Vector3f(.01f, .01f, .01f);
                ev.orientation_var = Vector3f(.01f, .01f, .01f);
                ev.velocity_var = Vector3f(NAN, NAN, NAN);
                ev.pos_frame = frame;
                ev.quality = 0;
                ekf->setExtVisionData(ev);
            }
        }
        ekf->update();
    }
    const auto &s = ekf->control_status_flags();
    const auto pos = ekf->getPosition();
    const float estimate = Eulerf(ekf->getQuaternion()).psi();
    const float yaw_error = std::atan2(std::sin(estimate - yaw), std::cos(estimate - yaw));
    const bool expect_ready = frame == PositionFrame::LOCAL_FRAME_NED;
    const bool passed = s.ev_pos && s.ev_yaw && !s.mag_hdg && !s.mag_3D
        && s.ev_hgt && std::fabs(pos(2)-z)<.03f
        && ekf->isLocalHorizontalPositionValid()
        && ekf->isYawFinalAlignComplete() == expect_ready
        && std::fabs(yaw_error) < .02f
        && std::hypot(pos(0)-1.2f,pos(1)+.4f) < .02f;
    std::printf("CASE frame=%d yaw=%.6f inject_mag=%d ev_pos=%d ev_yaw=%d mag_hdg=%d mag3d=%d xy=%d heading_ready=%d yaw_error=%.6f x=%.6f y=%.6f passed=%d\n",
        static_cast<int>(frame),static_cast<double>(yaw),inject_mag,s.ev_pos,s.ev_yaw,
        s.mag_hdg,s.mag_3D,ekf->isLocalHorizontalPositionValid(),ekf->isYawFinalAlignComplete(),
        static_cast<double>(yaw_error),static_cast<double>(pos(0)),static_cast<double>(pos(1)),passed);
    // Stop EV while keeping IMU and barometer; check EV height aiding goes inactive.
    for(uint64_t t=16004000;t<=24000000;t+=4000) {
        imuSample imu{};imu.time_us=t;imu.delta_ang_dt=.004f;imu.delta_vel_dt=.004f;
        imu.delta_vel=Vector3f(0.f,0.f,-9.80665f*.004f);ekf->setIMUData(imu);
        if(t%20000==0){baroSample b{};b.time_us=t;b.hgt=.8f;ekf->setBaroData(b);}
        ekf->update();
    }
    const bool loss_ok=!ekf->control_status_flags().ev_hgt;
    std::printf("HEIGHT z_in=%.6f z_out=%.6f ev_hgt=%d dropout_ev_hgt_off=%d passed=%d\n",
        double(z),double(pos(2)),s.ev_hgt,loss_ok,passed&&loss_ok);
    return passed&&loss_ok;
}
int main()
{
    bool ok = true;
    for (const float yaw : {0.f, .7f, -1.5f}) {
        ok = run_case(PositionFrame::LOCAL_FRAME_NED, yaw, false) && ok;
        ok = run_case(PositionFrame::LOCAL_FRAME_NED, yaw, true) && ok;
    }
    ok = run_case(PositionFrame::LOCAL_FRAME_NED, .7f, true, -1.3f) && ok;
    ok = run_case(PositionFrame::LOCAL_FRAME_NED, -.7f, true, .4f) && ok;
    return ok ? 0 : 2;
}
