float AttitudeCascade::calc_diff_speed_bypass_angle(float t,const Mpu6050Data& d,float dt){
    if(dt<=0.0001f||dt>0.1f)dt=0.005f;
    float g=d.gyro_z;
    if(std::fabs(g)>car_config::kGyroRawSpikeRejectDps)g=last_gyro_z_measurement_;
    g=clamp_float(g,-car_config::kGyroMeasurementLimitDps,car_config::kGyroMeasurementLimitDps);
    gyro_z_filtered_=gyro_z_filtered_*(1.0f-car_config::kGyroLpfAlpha)+g*car_config::kGyroLpfAlpha;
    g=std::fabs(gyro_z_filtered_)<car_config::kGyroDeadBandDps?0.0f:gyro_z_filtered_;
    gyro_pid.err=t-g;
    gyro_pid.integral=clamp_float(gyro_pid.integral+gyro_pid.err*dt,-gyro_pid.int_max,gyro_pid.int_max);
    float dterm=-gyro_pid.D*(g-last_gyro_z_measurement_)/dt;
    last_gyro_z_measurement_=g;
    float wanted=t*car_config::kGyroFeedforwardGain+gyro_pid.P*gyro_pid.err+gyro_pid.I*gyro_pid.integral+dterm;
    float step=car_config::kDiffOutputMaxDelta;
    if((t>car_config::kReverseGyroTargetThresholdDps&&g<-car_config::kReverseGyroRealThresholdDps)||(t<-car_config::kReverseGyroTargetThresholdDps&&g>car_config::kReverseGyroRealThresholdDps))step=car_config::kDiffOutputReverseMaxDelta;
    float out=last_gyro_output_+clamp_float(wanted-last_gyro_output_,-step,step);
    out=clamp_float(out,-gyro_pid.out_max,gyro_pid.out_max);
    if(t==0.0f&&std::fabs(g)>car_config::kStraightHighGyroLimitDps)out=clamp_float(out,-car_config::kStraightHighGyroDiffLimit,car_config::kStraightHighGyroDiffLimit);
    gyro_pid.output=out; gyro_pid.last_err=gyro_pid.err; last_gyro_output_=out;
    return out;
}
