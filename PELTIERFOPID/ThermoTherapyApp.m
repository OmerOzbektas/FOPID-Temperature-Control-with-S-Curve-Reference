classdef ThermoTherapyApp < matlab.apps.AppBase

    properties (Access = public)
        UIFigure               matlab.ui.Figure
        UIAxes                 matlab.ui.control.UIAxes
        ProgramDropDownLabel   matlab.ui.control.Label
        ProgramDropDown        matlab.ui.control.DropDown
        StartButton            matlab.ui.control.Button
        StopButton             matlab.ui.control.Button

        KpEditFieldLabel       matlab.ui.control.Label
        KpEditField            matlab.ui.control.NumericEditField
        KiEditFieldLabel       matlab.ui.control.Label
        KiEditField            matlab.ui.control.NumericEditField
        KdEditFieldLabel       matlab.ui.control.Label
        KdEditField            matlab.ui.control.NumericEditField
        LambdaEditLabel        matlab.ui.control.Label
        LambdaEditField        matlab.ui.control.NumericEditField
        MuEditLabel            matlab.ui.control.Label
        MuEditField            matlab.ui.control.NumericEditField

        TuneButton             matlab.ui.control.Button
        StatusLabel            matlab.ui.control.Label
    end

    properties (Access = private)
        Timer          timer
        StepsPerTick   double = 10
        SimIndex       double = 1

        t_session      double
        Tref_session   double
        Tout_session   double

        RefLine        matlab.graphics.chart.primitive.Line
        OutLine        matlab.graphics.chart.primitive.Line

        PlantParams    struct
        HotProfile
        ColdProfile

        TuneFig        matlab.ui.Figure = matlab.ui.Figure.empty
        lbEdit         matlab.ui.control.EditField = matlab.ui.control.EditField.empty
        ubEdit         matlab.ui.control.EditField = matlab.ui.control.EditField.empty
        popEdit        matlab.ui.control.NumericEditField = matlab.ui.control.NumericEditField.empty
        genEdit        matlab.ui.control.NumericEditField = matlab.ui.control.NumericEditField.empty
        runBtn         matlab.ui.control.Button = matlab.ui.control.Button.empty
        closeBtn       matlab.ui.control.Button = matlab.ui.control.Button.empty
        progLbl        matlab.ui.control.Label = matlab.ui.control.Label.empty
        bestLbl        matlab.ui.control.Label = matlab.ui.control.Label.empty

        GArunning      logical = false

        T_max double = 45
        T_min double = 10

        tSplit double = 120
        wEarly double = 6.0
        wLate  double = 1.0
        wU     double = 0.01
        wDU    double = 0.05
        wKick  double = 0.2
        wOS    double = 3000
        osTol  double = 0.3
    end

    methods (Access = private)

        function createComponents(app)
            app.UIFigure = uifigure('Name','ThermoTherapy App');
            app.UIFigure.Position = [100 100 980 560];

            app.UIAxes = uiaxes(app.UIFigure);
            app.UIAxes.Position = [55 190 870 340];
            xlabel(app.UIAxes,'Time [min]');
            ylabel(app.UIAxes,'Temperature [°C]');
            title(app.UIAxes,'Thermotherapy Session');
            grid(app.UIAxes,'on');
            hold(app.UIAxes,'on');

            app.RefLine = plot(app.UIAxes, NaN, NaN, 'r--', 'LineWidth', 1.5);
            app.OutLine = plot(app.UIAxes, NaN, NaN, 'b-',  'LineWidth', 1.5);
            legend(app.UIAxes, {'T_{ref}','T_{out}'}, 'Location','best');

            app.ProgramDropDownLabel = uilabel(app.UIFigure);
            app.ProgramDropDownLabel.Text = 'Program';
            app.ProgramDropDownLabel.Position = [55 150 70 22];

            app.ProgramDropDown = uidropdown(app.UIFigure);
            app.ProgramDropDown.Items = {'Hot','Cold','HotCold3'};
            app.ProgramDropDown.Value = 'Hot';
            app.ProgramDropDown.Position = [125 150 150 22];

            app.StartButton = uibutton(app.UIFigure,'push');
            app.StartButton.Text = 'Start Simulation';
            app.StartButton.Position = [300 145 130 32];
            app.StartButton.ButtonPushedFcn = @(~,~)app.onStartButtonPushed();

            app.StopButton = uibutton(app.UIFigure,'push');
            app.StopButton.Text = 'Stop';
            app.StopButton.Position = [440 145 80 32];
            app.StopButton.Enable = 'off';
            app.StopButton.ButtonPushedFcn = @(~,~)app.onStopButtonPushed();

            x0 = 55; y0 = 110; w = 90;

            app.KpEditFieldLabel = uilabel(app.UIFigure);
            app.KpEditFieldLabel.Text = 'Kp';
            app.KpEditFieldLabel.Position = [x0 y0 25 22];
            app.KpEditField = uieditfield(app.UIFigure,'numeric');
            app.KpEditField.Position = [x0+25 y0 w 22];
            app.KpEditField.Value = 0.5;

            app.KiEditFieldLabel = uilabel(app.UIFigure);
            app.KiEditFieldLabel.Text = 'Ki';
            app.KiEditFieldLabel.Position = [x0+140 y0 25 22];
            app.KiEditField = uieditfield(app.UIFigure,'numeric');
            app.KiEditField.Position = [x0+165 y0 w 22];
            app.KiEditField.Value = 0.01;

            app.KdEditFieldLabel = uilabel(app.UIFigure);
            app.KdEditFieldLabel.Text = 'Kd';
            app.KdEditFieldLabel.Position = [x0+280 y0 25 22];
            app.KdEditField = uieditfield(app.UIFigure,'numeric');
            app.KdEditField.Position = [x0+305 y0 w 22];
            app.KdEditField.Value = 0.05;

            app.LambdaEditLabel = uilabel(app.UIFigure);
            app.LambdaEditLabel.Text = 'λ';
            app.LambdaEditLabel.Position = [x0+420 y0 20 22];
            app.LambdaEditField = uieditfield(app.UIFigure,'numeric');
            app.LambdaEditField.Position = [x0+445 y0 w 22];
            app.LambdaEditField.Value = 0.7;

            app.MuEditLabel = uilabel(app.UIFigure);
            app.MuEditLabel.Text = 'μ';
            app.MuEditLabel.Position = [x0+560 y0 20 22];
            app.MuEditField = uieditfield(app.UIFigure,'numeric');
            app.MuEditField.Position = [x0+585 y0 w 22];
            app.MuEditField.Value = 0.8;

            app.TuneButton = uibutton(app.UIFigure,'push');
            app.TuneButton.Text = 'Tune (GA)';
            app.TuneButton.Position = [x0+720 y0-2 120 28];
            app.TuneButton.ButtonPushedFcn = @(~,~)app.onTuneButtonPushed();

            app.StatusLabel = uilabel(app.UIFigure);
            app.StatusLabel.Text = 'Status: idle';
            app.StatusLabel.Position = [55 70 870 22];
        end

        function startupFcn(app)
            app.PlantParams = PeltierParams();
            app.HotProfile  = createHotProfile();
            app.ColdProfile = createColdProfile();

            app.Timer = timer( ...
                'ExecutionMode','fixedRate', ...
                'Period',0.1, ...
                'TimerFcn',@(~,~)app.onTimerTick());
        end

        function G = getPlant(app)
            p = app.PlantParams;
            try
                G = PeltierPlant(p);
            catch
                G = PeltierPlant();
            end
        end

        function [t_ref, T_ref] = buildReference(app, Ts)
            switch app.ProgramDropDown.Value
                case 'Hot'
                    Ttot = app.HotProfile.t_rise + app.HotProfile.t_hold + app.HotProfile.t_fall;
                    [t_ref, T_ref] = app.HotProfile.generate(Ttot, Ts);
                case 'Cold'
                    Ttot = app.ColdProfile.t_rise + app.ColdProfile.t_hold + app.ColdProfile.t_fall;
                    [t_ref, T_ref] = app.ColdProfile.generate(Ttot, Ts);
                case 'HotCold3'
                    [t_ref, T_ref, ~] = buildHotColdSession(app.HotProfile, app.ColdProfile, 3, Ts);
                otherwise
                    error('Unsupported program');
            end
        end

        function [C, CL_y, CL_u] = buildClosedLoop(app, Kp, Ki, Kd, lambda, mu, G)
            p = app.PlantParams;
            tau = p.tau;
            wb  = 1/(50*tau);
            wh  = 5/tau;
            N   = 5;
            fop = FOPID(Kp, Ki, Kd, lambda, mu, wh, wb, N, 'Continuous', []);
            C   = fop.reduce(10);
            CL_y = feedback(C*G, 1);
            CL_u = feedback(C, G);
        end

        function T_out = simulateSession(app, t_ref, T_ref)
            G = app.getPlant();
            Kp     = app.KpEditField.Value;
            Ki     = app.KiEditField.Value;
            Kd     = app.KdEditField.Value;
            lambda = app.LambdaEditField.Value;
            mu     = app.MuEditField.Value;

            [~, CL_y, ~] = app.buildClosedLoop(Kp, Ki, Kd, lambda, mu, G);

            T0 = T_ref(1);
            r  = T_ref - T0;

            y = lsim(CL_y, r, t_ref);
            T_out = y + T0;
        end

        function onStartButtonPushed(app)
            if ~isempty(app.Timer) && isvalid(app.Timer) && strcmp(app.Timer.Running,'on')
                stop(app.Timer);
            end

            app.StartButton.Enable = 'off';
            app.StopButton.Enable  = 'on';

            TsSim = 1.0;
            [t_ref, T_ref] = app.buildReference(TsSim);

            T_out = app.simulateSession(t_ref, T_ref);

            app.t_session    = t_ref;
            app.Tref_session = T_ref;
            app.Tout_session = T_out;
            app.SimIndex     = 1;

            t_min = t_ref/60.0;
            set(app.RefLine,'XData',t_min,'YData',T_ref);
            set(app.OutLine,'XData',[],'YData',[]);
            xlim(app.UIAxes,[0 max(t_min)]);
            ylim(app.UIAxes,[10 50]);

            app.StatusLabel.Text = 'Status: simulation running';
            drawnow;

            start(app.Timer);
        end

        function onStopButtonPushed(app)
            if ~isempty(app.Timer) && isvalid(app.Timer) && strcmp(app.Timer.Running,'on')
                stop(app.Timer);
            end
            app.StartButton.Enable = 'on';
            app.StopButton.Enable  = 'off';
            app.StatusLabel.Text = 'Status: idle';
        end

        function onTimerTick(app)
            if isempty(app.t_session)
                return;
            end

            n = numel(app.t_session);
            if app.SimIndex > n
                app.onStopButtonPushed();
                return;
            end

            idx_end = min(app.SimIndex + app.StepsPerTick - 1, n);
            idx = 1:idx_end;

            t_min = app.t_session(idx)/60.0;
            T_out = app.Tout_session(idx);

            set(app.OutLine,'XData',t_min,'YData',T_out);
            drawnow limitrate;

            app.SimIndex = idx_end + 1;
        end

        function onTuneButtonPushed(app)
            if ~isempty(app.Timer) && isvalid(app.Timer) && strcmp(app.Timer.Running,'on')
                app.onStopButtonPushed();
            end

            if ~isempty(app.TuneFig) && isvalid(app.TuneFig)
                figure(app.TuneFig);
                return;
            end

            app.GArunning = false;

            app.TuneFig = uifigure('Name','GA Settings','Position',[200 200 560 270]);
            app.TuneFig.CloseRequestFcn = @(~,~)app.onTuneDialogClose();

            uilabel(app.TuneFig,'Text','Lower bounds [Kp Ki Kd λ μ]','Position',[20 220 210 22]);
            app.lbEdit = uieditfield(app.TuneFig,'text','Position',[240 220 300 22],'Value','[0 0 0 0.4 0.6]');

            uilabel(app.TuneFig,'Text','Upper bounds [Kp Ki Kd λ μ]','Position',[20 185 210 22]);
            app.ubEdit = uieditfield(app.TuneFig,'text','Position',[240 185 300 22],'Value','[6 0.8 0.8 0.95 0.95]');

            uilabel(app.TuneFig,'Text','Population size','Position',[20 150 210 22]);
            app.popEdit = uieditfield(app.TuneFig,'numeric','Position',[240 150 120 22],'Value',30);

            uilabel(app.TuneFig,'Text','Max generations','Position',[20 115 210 22]);
            app.genEdit = uieditfield(app.TuneFig,'numeric','Position',[240 115 120 22],'Value',40);

            app.progLbl = uilabel(app.TuneFig,'Text','Gen: -   Best J: -','Position',[20 75 520 22]);
            app.bestLbl = uilabel(app.TuneFig,'Text','Status: idle','Position',[20 50 520 22]);

            app.runBtn = uibutton(app.TuneFig,'push','Text','Run GA','Position',[420 110 120 32], ...
                'ButtonPushedFcn',@(~,~)app.onRunGAInDialog());

            app.closeBtn = uibutton(app.TuneFig,'push','Text','Close','Position',[420 70 120 32], ...
                'ButtonPushedFcn',@(~,~)app.onTuneDialogClose());
        end

        function onRunGAInDialog(app)
            if app.GArunning
                return;
            end

            try
                lb = str2num(app.lbEdit.Value); %#ok<ST2NM>
                ub = str2num(app.ubEdit.Value); %#ok<ST2NM>
            catch
                uialert(app.TuneFig,'Invalid bounds format. Example: [0 0 0 0.4 0.6]','GA Error');
                return;
            end

            if numel(lb) ~= 5 || numel(ub) ~= 5
                uialert(app.TuneFig,'Bounds must be 1x5 vectors.','GA Error');
                return;
            end

            popSize = max(5, round(app.popEdit.Value));
            maxGen  = max(1, round(app.genEdit.Value));

            app.GArunning = true;
            app.runBtn.Enable = 'off';
            app.closeBtn.Enable = 'off';

            app.StatusLabel.Text = 'Status: GA running';
            app.bestLbl.Text = 'Status: running...';
            app.progLbl.Text = 'Gen: 0   Best J: -';
            drawnow;

            simTs = 1.0;
            [t_ref, T_ref] = app.buildReference(simTs);
            G = app.getPlant();

            T0 = T_ref(1);
            r  = T_ref - T0;

            function J = fitnessFcn(x)
                if any(~isfinite(x))
                    J = 1e9; return;
                end
                Kp = x(1); Ki = x(2); Kd = x(3); lambda = x(4); mu = x(5);
                if Kp < 0 || Ki < 0 || Kd < 0 || lambda <= 0 || lambda >= 1 || mu <= 0 || mu >= 1
                    J = 1e9; return;
                end

                try
                    [~, CL_y, CL_u] = app.buildClosedLoop(Kp, Ki, Kd, lambda, mu, G);
                catch
                    J = 1e9; return;
                end

                if ~isstable(CL_y)
                    J = 1e9; return;
                end

                try
                    [y, t] = lsim(CL_y, r, t_ref);
                    [u, ~] = lsim(CL_u, r, t_ref);
                catch
                    J = 1e9; return;
                end

                if any(~isfinite(y)) || any(~isfinite(u))
                    J = 1e9; return;
                end

                T_out = y + T0;

                if any(~isfinite(T_out))
                    J = 1e9; return;
                end

                e = T_ref(:) - T_out(:);
                t = t(:);

                idx0 = t <= app.tSplit;
                idx1 = t >  app.tSplit;

                IAE_early = sum(abs(e(idx0))) * simTs;
                ITAE_late = sum(t(idx1) .* abs(e(idx1))) * simTs;

                Ju = sum(abs(u(:))) * simTs;

                du = diff(u(:)) / simTs;
                Jdu = sum(abs(du)) * simTs;

                kick = abs(u(1));

                os = max(T_out(:) - T_ref(:));
                Jos = 0;
                if os > app.osTol
                    Jos = app.wOS * (os - app.osTol)^2;
                end

                penalty = 0;
                if any(T_out > app.T_max)
                    penalty = penalty + (max(T_out) - app.T_max) * 1000;
                end
                if any(T_out < app.T_min)
                    penalty = penalty + (app.T_min - min(T_out)) * 1000;
                end

                J = app.wEarly*IAE_early + app.wLate*ITAE_late + app.wU*Ju + app.wDU*Jdu + app.wKick*kick + Jos + penalty;

                if ~isfinite(J)
                    J = 1e9;
                end
            end

            function [stateOut, optionsOut, optchanged] = outFcn(options, state, flag)
                stateOut = state;
                optionsOut = options;
                optchanged = false;
                if strcmp(flag,'iter')
                    gen = state.Generation;
                    if ~isempty(state.Best)
                        bestJ = state.Best(end);
                        app.progLbl.Text = sprintf('Gen: %d   Best J: %.3e', gen, bestJ);
                        app.bestLbl.Text = 'Status: running...';
                    else
                        app.progLbl.Text = sprintf('Gen: %d   Best J: -', gen);
                    end
                    drawnow limitrate;
                elseif strcmp(flag,'done')
                    app.bestLbl.Text = 'Status: completed';
                    drawnow;
                end
            end

            opts = optimoptions('ga', ...
                'PopulationSize', popSize, ...
                'MaxGenerations', maxGen, ...
                'Display', 'off', ...
                'UseParallel', false, ...
                'OutputFcn', @outFcn);

            try
                [x_best, j_best] = ga(@fitnessFcn, 5, [], [], [], [], lb(:).', ub(:).', [], opts);
            catch ME
                app.GArunning = false;
                app.runBtn.Enable = 'on';
                app.closeBtn.Enable = 'on';
                app.StatusLabel.Text = 'Status: idle';
                if ~isempty(app.TuneFig) && isvalid(app.TuneFig)
                    uialert(app.TuneFig, getReport(ME,'extended','hyperlinks','off'), 'GA Error');
                end
                return;
            end

            app.KpEditField.Value     = x_best(1);
            app.KiEditField.Value     = x_best(2);
            app.KdEditField.Value     = x_best(3);
            app.LambdaEditField.Value = x_best(4);
            app.MuEditField.Value     = x_best(5);

            app.StatusLabel.Text = sprintf('Status: GA done (Best J = %.3e)', j_best);
            drawnow;

            app.GArunning = false;
            app.onTuneDialogClose();
        end

        function onTuneDialogClose(app)
            if app.GArunning
                return;
            end
            if ~isempty(app.TuneFig) && isvalid(app.TuneFig)
                delete(app.TuneFig);
            end
            app.TuneFig = matlab.ui.Figure.empty;
        end
    end

    methods (Access = public)
        function app = ThermoTherapyApp
            createComponents(app);
            startupFcn(app);
        end

        function delete(app)
            if ~isempty(app.Timer) && isvalid(app.Timer)
                if strcmp(app.Timer.Running,'on')
                    stop(app.Timer);
                end
                delete(app.Timer);
            end
            if ~isempty(app.TuneFig) && isvalid(app.TuneFig)
                delete(app.TuneFig);
            end
            if isvalid(app.UIFigure)
                delete(app.UIFigure);
            end
        end
    end
end
