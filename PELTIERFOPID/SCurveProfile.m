classdef SCurveProfile < handle

    properties
        T_start
        T_target
        T_end

        t_rise
        t_hold
        t_fall
        
        k_rise
        k_fall
        
        modeName
    end

    methods
        function obj = SCurveProfile(T_start, T_target, T_end, ...
                                     t_rise, t_hold, t_fall, ...
                                     k_rise, k_fall, modeName)

            if nargin >= 1, obj.T_start  = T_start;  end
            if nargin >= 2, obj.T_target = T_target; end
            if nargin >= 3, obj.T_end    = T_end;    end
            if nargin >= 4, obj.t_rise   = t_rise;   end
            if nargin >= 5, obj.t_hold   = t_hold;   end
            if nargin >= 6, obj.t_fall   = t_fall;   end
            if nargin >= 7, obj.k_rise   = k_rise;   end
            if nargin >= 8, obj.k_fall   = k_fall;   end
            if nargin >= 9, obj.modeName = modeName; end

            if obj.t_rise < 0 || obj.t_hold < 0 || obj.t_fall < 0
                error('SCurveProfile: Time durations must be non-negative.');
            end
        end

        function T = value(obj, t)
            Tprog = obj.t_rise + obj.t_hold + obj.t_fall;

            if t <= 0
                T = obj.T_start;
                return;
            end
            if t >= Tprog
                T = obj.T_end;
                return;
            end

            t1 = obj.t_rise;
            t2 = obj.t_rise + obj.t_hold;

            if t < t1
                if obj.t_rise <= 0
                    T = obj.T_target;
                    return;
                end
                u = t / obj.t_rise;
                s = SCurveProfile.smoothstep5(u);
                T = obj.T_start + (obj.T_target - obj.T_start) * s;
                return;
            end

            if t < t2
                T = obj.T_target;
                return;
            end

            if obj.t_fall <= 0
                T = obj.T_end;
                return;
            end
            u = (t - t2) / obj.t_fall;
            s = SCurveProfile.smoothstep5(u);
            T = obj.T_target + (obj.T_end - obj.T_target) * s;
        end

        function [t_vec, T_vec] = generate(obj, T_total, Ts)

            Tprog = obj.t_rise + obj.t_hold + obj.t_fall;

            if nargin < 2 || isempty(T_total)
                T_total = Tprog;
            end
            if nargin < 3 || isempty(Ts) || Ts <= 0
                Ts = 0.1;
            end

            tr = obj.t_rise;
            th = obj.t_hold;
            tf = obj.t_fall;

            t1 = SCurveProfile.timeGrid(tr, Ts);
            t2 = SCurveProfile.timeGrid(th, Ts);
            t3 = SCurveProfile.timeGrid(tf, Ts);

            if tr <= 0
                T1 = obj.T_target * ones(size(t1));
                T1(1) = obj.T_start;
                T1(end) = obj.T_target;
            else
                u1 = t1 / tr;
                s1 = SCurveProfile.smoothstep5(u1);
                T1 = obj.T_start + (obj.T_target - obj.T_start) .* s1;
                T1(1) = obj.T_start;
                T1(end) = obj.T_target;
            end

            T2 = obj.T_target * ones(size(t2));
            if ~isempty(T2)
                T2(1) = obj.T_target;
                T2(end) = obj.T_target;
            end

            if tf <= 0
                T3 = obj.T_end * ones(size(t3));
                T3(1) = obj.T_target;
                T3(end) = obj.T_end;
            else
                u3 = t3 / tf;
                s3 = SCurveProfile.smoothstep5(u3);
                T3 = obj.T_target + (obj.T_end - obj.T_target) .* s3;
                T3(1) = obj.T_target;
                T3(end) = obj.T_end;
            end

            t_full = t1(:);
            T_full = T1(:);

            if numel(t2) > 1
                t_full = [t_full; (t_full(end) + t2(2:end)).'];
                T_full = [T_full; T2(2:end).'];
            end

            if numel(t3) > 1
                t_full = [t_full; (t_full(end) + t3(2:end)).'];
                T_full = [T_full; T3(2:end).'];
            end

            if T_total <= 0
                t_vec = 0;
                T_vec = obj.T_start;
                return;
            end

            if T_total < t_full(end) - 1e-12
                idx = t_full <= T_total + 1e-12;
                t_vec = t_full(idx);
                T_vec = T_full(idx);
                if t_vec(end) < T_total - 1e-12
                    t_vec = [t_vec; T_total];
                    T_vec = [T_vec; obj.value(T_total)];
                else
                    t_vec(end) = T_total;
                    T_vec(end) = obj.value(T_total);
                end
            else
                t_vec = t_full;
                T_vec = T_full;
                if T_total > t_vec(end) + 1e-12
                    t_vec = [t_vec; T_total];
                    T_vec = [T_vec; obj.value(T_total)];
                end
            end
        end
    end

    methods (Static, Access = private)
        function t = timeGrid(Tseg, Ts)
            if Tseg <= 0
                t = 0;
                return;
            end
            n = floor(Tseg / Ts);
            t = (0:n) * Ts;
            if abs(t(end) - Tseg) > 1e-12
                t = [t, Tseg];
            end
        end

        function s = smoothstep5(u)
            u = max(0, min(1, u));
            s = 10*u.^3 - 15*u.^4 + 6*u.^5;
            if ~isempty(u)
                s(u <= 0) = 0;
                s(u >= 1) = 1;
            end
        end
    end
end
