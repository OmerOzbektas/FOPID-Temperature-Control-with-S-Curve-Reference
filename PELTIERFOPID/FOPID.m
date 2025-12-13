classdef FOPID < handle
    properties
        Kp = 0
        Ki = 0
        Kd = 0
        lambda = []
        mu = []
        wh = []
        wb = []
        N = 0
        Domain = 'Continuous'
        Ts = []
    end

    methods
        function obj = FOPID(Kp, Ki, Kd, lambda, mu, wh, wb, N, Domain, Ts)
            if nargin >= 1, obj.Kp = Kp; end
            if nargin >= 2, obj.Ki = Ki; end
            if nargin >= 3, obj.Kd = Kd; end
            if nargin >= 4, obj.lambda = lambda; end
            if nargin >= 5, obj.mu = mu; end
            if nargin >= 6, obj.wh = wh; end
            if nargin >= 7, obj.wb = wb; end
            if nargin >= 8, obj.N = round(N); end
            if nargin >= 9, obj.Domain = Domain; end
            if strcmp(obj.Domain, 'Discrete')
                if nargin < 10 || isempty(Ts) || Ts <= 0
                    error('For Discrete domain, Ts must be provided and > 0.');
                end
                obj.Ts = Ts;
            else
                obj.Ts = [];
            end
            if ~isempty(obj.lambda) && (obj.lambda < 0 || obj.lambda > 1)
                error('lambda must satisfy 0 <= lambda <= 1');
            end
            if ~isempty(obj.mu) && (obj.mu < 0 || obj.mu > 1)
                error('mu must satisfy 0 <= mu <= 1');
            end
            if isempty(obj.N) || obj.N < 0
                obj.N = 0;
            end
        end

        function y = ORA(obj, gamma, s)
            N = round(obj.N);
            K = obj.wh^gamma;
            prod_val = 1;
            inv1pg = 1 / (1 + gamma);
            inv1mg = 1 / (1 - gamma + 1e-9);
            for k = -N : N
                a  = k + N + 0.5 * inv1pg;
                ap = k + N + 0.5 * inv1mg;
                b  = 2 * N + 1;
                wk  = obj.wb * (obj.wh / obj.wb)^(a / b);
                wkp = obj.wb * (obj.wh / obj.wb)^(ap / b);
                prod_val = prod_val .* ((s + wkp) / (s + wk));
            end
            y = K * prod_val;
        end

        function num_tf = getnumtf(obj)
            s = tf('s');
            s_lambda = obj.ORA(obj.lambda, s);
            s_mu     = obj.ORA(obj.mu, s);
            if strcmp(obj.Domain,'Continuous')
            num_tf = obj.Kp + obj.Kd * s_lambda / (1+5*s) + obj.Ki / s_mu;
            else
                num_tf = c2d(num_tf, obj.Ts, 'tustin'); 
            end
        end

        function reduced = reduce(obj, order)
            con_tf = obj.getnumtf();
            opt = balredOptions('StateElimMethod','MatchDC');
            if isa(con_tf,'zpk') && length(pole(con_tf)) <= order
                reduced_con = con_tf;
            else
                reduced_con = balred(con_tf, order, opt);
            end
                reduced = reduced_con;
        end
    end

    methods (Static)
        function sym_tf = tf2sym(sys_numeric)
            sys_tf_obj = tf(sys_numeric);
            [num, den] = tfdata(sys_tf_obj, 'v');
            if sys_tf_obj.Ts == 0
                var = sym('s');
            else
                var = sym('z');
            end
            sym_num = poly2sym(vpa(num, 8), var);
            sym_den = poly2sym(vpa(den, 8), var);
            sym_tf = simplify(sym_num / sym_den);
        end
    end
end
