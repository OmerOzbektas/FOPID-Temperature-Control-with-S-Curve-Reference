function p = PeltierParams()

    p.Tamb_nominal  = 24;
    p.Tskin_nominal = 32;

    p.Kth = 18;
    p.tau = 60;
    p.Td  = 3;

    p.Imax    = 6.0;
    p.Vsupply = 12.0;
    p.Relec   = 2.2;

    p.Kth_uncertainty = 0.2;
    p.tau_uncertainty = 0.2;
    p.Td_uncertainty  = 0.2;
end
