function [t_session, Tref_session, segmentIds] = buildHotColdSession( ...
            hotProfile, coldProfile, nCycles, Ts)

    if nargin < 3 || isempty(nCycles)
        nCycles = 3;
    end
    if nargin < 4 || isempty(Ts)
        Ts = 1.0;
    end

    Thot  = hotProfile.t_rise  + hotProfile.t_hold  + hotProfile.t_fall;
    Tcold = coldProfile.t_rise + coldProfile.t_hold + coldProfile.t_fall;

    t_session    = [];
    Tref_session = [];
    segmentIds   = [];

    t_cursor = 0;

    for k = 1:nCycles
        [t_hot, T_hot] = hotProfile.generate(Thot, Ts);
        t_hot = t_hot + t_cursor;
        t_cursor = t_cursor + Thot;

        t_session    = [t_session;    t_hot(:)];
        Tref_session = [Tref_session; T_hot(:)];
        segmentIds   = [segmentIds;   ones(numel(T_hot),1)*1];

        [t_cold, T_cold] = coldProfile.generate(Tcold, Ts);
        t_cold = t_cold + t_cursor;
        t_cursor = t_cursor + Tcold;

        t_session    = [t_session;    t_cold(:)];
        Tref_session = [Tref_session; T_cold(:)];
        segmentIds   = [segmentIds;   ones(numel(T_cold),1)*2];
    end

    [t_session, uniqueIdx] = unique(t_session, 'stable');
    Tref_session = Tref_session(uniqueIdx);
    segmentIds   = segmentIds(uniqueIdx);
end
