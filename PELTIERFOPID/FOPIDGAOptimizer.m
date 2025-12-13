classdef FOPIDGAOptimizer < handle

    properties
        plantParams
        hotProfile
        coldProfile
        programType
        nCycles = 3

        simTs = 1.0

        wb
        wh
        N

        lb
        ub

        popSize = 30
        maxGen  = 40

        T_max = 45
        T_min = 10

        tSplit = 120
        wEarly = 6.0
        wLate  = 1.0
        wU     = 0.01
        wDU    = 0.05
        wKick  = 0.2
        wOS    = 3000
        osTol  = 0.3

        bestX
        bestJ
        bestController
        bestResponse
    end

    methods
        function obj = FOPIDGAOptimizer(plantParams, hotProfile, coldProfile)

            if nargin < 1 || isempty(plantParams)
                plantParams = PeltierParams();
            end
            if nargin < 2 || isempty(hotProfile)
                hotProfile = createHotProfile();
            end
            if nargin < 3 || isempty(coldProfile)
                coldProfile = createColdProfile();
            end

            obj.plantParams = plantParams;
            obj.hotProfile  = hotProfile;
            obj.coldProfile = coldProfile;
            obj.programType = 'Hot';

            tau = plantParams.tau;

            obj.wb = 1/(50 * tau);
            obj.wh = 5 / tau;

            obj.N  = 5;

            obj.lb = [0    0      0      0.4  0.6];
            obj.ub = [6.0  0.8    0.8    0.95 0.95];
        end

        function setProgram(obj, programType)
            obj.programType = programType;
        end

        function setBounds(obj, lb, ub)
            obj.lb = lb(:).';
            obj.ub = ub(:).';
        end

        function setGAOptions(obj, popSize, maxGen)
            obj.popSize = popSize;
            obj.maxGen  = maxGen;
        end

        function [xBest, JBest] = runOptimization(obj)

            fitnessFcn = @(x) obj.evaluateFitness(x);

            opts = optimoptions('ga', ...
                'PopulationSize', obj.popSize, ...
                'MaxGenerations', obj.maxGen, ...
                'Display', 'off', ...
                'UseParallel', false, ...
                'OutputFcn', @(options,state,flag) obj.gaOutputFcn(options,state,flag));

            [xBest, JBest] = ga(fitnessFcn, 5, [], [], [], [], obj.lb, obj.ub, [], opts);

            obj.bestX = xBest;
            obj.bestJ = JBest;

            [C, resp] = obj.simulateWithParams(xBest);
            obj.bestController = C;
            obj.bestResponse   = resp;
        end

        function [C, resp] = simulateWithParams(obj, x)

            G = PeltierPlant(obj.plantParams);

            Kp     = x(1);
            Ki     = x(2);
            Kd     = x(3);
            lambda = x(4);
            mu     = x(5);

            fop = FOPID(Kp, Ki, Kd, lambda, mu, obj.wh, obj.wb, obj.N, 'Continuous', []);

            C = fop.reduce(10);

            CL_y = feedback(C * G, 1);
            CL_u = feedback(C, G);

            [t_ref, T_ref] = obj.buildReference(obj.simTs);

            T0 = T_ref(1);
            r  = T_ref - T0;

            [y, t]  = lsim(CL_y, r, t_ref);
            [u, ~]  = lsim(CL_u, r, t_ref);

            T_out = y + T0;

            resp.t     = t;
            resp.T_ref = T_ref;
            resp.T_out = T_out;
            resp.u     = u;
        end

        function plotBestResponse(obj)
            if isempty(obj.bestResponse)
                error('Optimization has not been run yet.');
            end

            r = obj.bestResponse;
            figure;
            plot(r.t/60, r.T_ref, 'r--', 'LineWidth', 1.2); hold on;
            plot(r.t/60, r.T_out, 'b',  'LineWidth', 1.5);
            grid on;
            xlabel('Time [min]');
            ylabel('Temperature [°C]');
            legend('Reference', 'Output', 'Location', 'best');
            title('Best GA-Tuned FOPID Response');
        end

        function printBestParameters(obj)
            if isempty(obj.bestX)
                disp('No solution available.');
                return;
            end

            x = obj.bestX;
            fprintf('Best FOPID parameters:\n');
            fprintf('Kp = %.6f\n', x(1));
            fprintf('Ki = %.6f\n', x(2));
            fprintf('Kd = %.6f\n', x(3));
            fprintf('lambda = %.6f\n', x(4));
            fprintf('mu = %.6f\n', x(5));
            fprintf('Cost J = %.6g\n', obj.bestJ);
        end
    end

    methods (Access = private)

        function [t_ref, T_ref] = buildReference(obj, Ts)

            switch obj.programType
                case 'Hot'
                    Ttot = obj.hotProfile.t_rise + obj.hotProfile.t_hold + obj.hotProfile.t_fall;
                    [t_ref, T_ref] = obj.hotProfile.generate(Ttot, Ts);

                case 'Cold'
                    Ttot = obj.coldProfile.t_rise + obj.coldProfile.t_hold + obj.coldProfile.t_fall;
                    [t_ref, T_ref] = obj.coldProfile.generate(Ttot, Ts);

                case 'HotCold3'
                    [t_ref, T_ref, ~] = buildHotColdSession(obj.hotProfile, obj.coldProfile, obj.nCycles, Ts);

                otherwise
                    error('Unsupported program type.');
            end
        end

        function J = evaluateFitness(obj, x)

            if any(~isfinite(x))
                J = 1e9; return;
            end

            Kp = x(1); Ki = x(2); Kd = x(3);
            lambda = x(4); mu = x(5);

            if Kp < 0 || Ki < 0 || Kd < 0 || lambda <= 0 || lambda >= 1 || mu <= 0 || mu >= 1
                J = 1e9; return;
            end

            G = PeltierPlant(obj.plantParams);

            try
                fop = FOPID(Kp, Ki, Kd, lambda, mu, obj.wh, obj.wb, obj.N, 'Continuous', []);
                C = fop.reduce(10);
            catch
                J = 1e9; return;
            end

            CL_y = feedback(C * G, 1);
            CL_u = feedback(C, G);

            if ~isstable(CL_y)
                J = 1e9; return;
            end

            [t_ref, T_ref] = obj.buildReference(obj.simTs);

            T0 = T_ref(1);
            r  = T_ref - T0;

            [y, t] = lsim(CL_y, r, t_ref);
            [u, ~] = lsim(CL_u, r, t_ref);

            if any(~isfinite(y)) || any(~isfinite(u))
                J = 1e9; return;
            end

            T_out = y + T0;

            e = T_ref(:) - T_out(:);
            t = t(:);

            idx0 = t <= obj.tSplit;
            idx1 = t >  obj.tSplit;

            IAE_early = sum(abs(e(idx0))) * obj.simTs;
            ITAE_late = sum(t(idx1) .* abs(e(idx1))) * obj.simTs;

            Ju = sum(abs(u(:))) * obj.simTs;

            du = diff(u(:)) / obj.simTs;
            Jdu = sum(abs(du)) * obj.simTs;

            kick = abs(u(1));

            os = max(T_out(:) - T_ref(:));
            Jos = 0;
            if os > obj.osTol
                Jos = obj.wOS * (os - obj.osTol)^2;
            end

            penalty = 0;
            if any(T_out > obj.T_max)
                penalty = penalty + (max(T_out) - obj.T_max) * 1000;
            end
            if any(T_out < obj.T_min)
                penalty = penalty + (obj.T_min - min(T_out)) * 1000;
            end

            J = obj.wEarly*IAE_early + obj.wLate*ITAE_late + obj.wU*Ju + obj.wDU*Jdu + obj.wKick*kick + Jos + penalty;

            if ~isfinite(J)
                J = 1e9;
            end
        end

        function [stateOut, optionsOut, optchanged] = gaOutputFcn(~, options, state, flag)

            stateOut   = state;
            optionsOut = options;
            optchanged = false;

            if strcmp(flag, 'iter') && ~isempty(state.Best)
                fprintf('[Gen %d] Best J = %.3e\n', state.Generation, state.Best(end));
            elseif strcmp(flag, 'done')
                fprintf('GA completed.\n');
            end
        end
    end
end
