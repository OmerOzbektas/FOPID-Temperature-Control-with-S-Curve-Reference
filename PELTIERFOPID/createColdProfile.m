function profile = createColdProfile()

    T_start  = 32;
    T_target = 14;
    T_end    = 30;
    
    t_rise   = 10 * 60;
    t_hold   = 15 * 60;
    t_fall   = 10 * 60;
    
    k_rise   = 6;
    k_fall   = 6;
    
    profile = SCurveProfile(T_start, T_target, T_end, ...
                            t_rise, t_hold, t_fall, ...
                            k_rise, k_fall, 'Cold');
end
