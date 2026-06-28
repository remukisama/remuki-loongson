float AttitudeCascade::calc_target_gyro_for_yaw(float a,const Mpu6050Data& d,float dt,float m){
    if(dt<=0.0001f||dt>0.1f)dt=car_config::kOuterLoopFallbackPeriodSec;
    float g=d.gyro_z;
    if(std::fabs(g)>car_config::kGyroRawSpikeRejectDps)g=last_gyro_z_measurement_;
    g=clamp_float(g,-car_config::kGyroMeasurementLimitDps,car_config::kGyroMeasurementLimitDps);
    if(std::fabs(g)<car_config::kYawIntegralDeadBandDps)g=0.0f;
    current_yaw+=g*dt; angle_pid.err=a-current_yaw;
    angle_pid.output=clamp_float(car_config::kYawTrackKp*angle_pid.err,-m,m);
    float e=std::fabs(angle_pid.err);
    if(e<=car_config::kYawTrackToleranceDeg&&std::fabs(d.gyro_z)<=car_config::kYawTrackSettleGyroDps)angle_pid.output=0.0f;
    else if(e>car_config::kYawTrackMinRateStartDeg&&std::fabs(angle_pid.output)<car_config::kYawTrackMinRateDps)angle_pid.output=angle_pid.err>=0.0f?car_config::kYawTrackMinRateDps:-car_config::kYawTrackMinRateDps;
    else if(e>car_config::kYawTrackFineRateStartDeg&&std::fabs(angle_pid.output)<car_config::kYawTrackFineRateDps)angle_pid.output=angle_pid.err>=0.0f?car_config::kYawTrackFineRateDps:-car_config::kYawTrackFineRateDps;
    angle_pid.last_err=angle_pid.err; return angle_pid.output;
}

bool AttitudeCascade::yaw_target_reached(float a,const Mpu6050Data& d)const{
    return std::fabs(a-current_yaw)<=car_config::kYawTrackToleranceDeg&&std::fabs(d.gyro_z)<=car_config::kYawTrackSettleGyroDps;
}
