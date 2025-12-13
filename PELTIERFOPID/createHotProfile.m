function profile = createHotProfile()

    T_start  = 32;
    T_target = 42;
    T_end    = 35;
    
    t_rise   = 4 * 60;
    t_hold   = 20 * 60;
    t_fall   = 5 * 60;
    
    k_rise   = 8;
    k_fall   = 8;
    
    profile = SCurveProfile(T_start, T_target, T_end, ...
                            t_rise, t_hold, t_fall, ...
                            k_rise, k_fall, 'Hot');
end
