function SOH_Estimation()
    % Clear command window and workspace
    clc;
    
    try
        % Battery Parameters
        params = struct(...
            'Q_nom', 2.3, ...       % Initial nominal capacity (Ah)
            'R0_nom', 0.1, ...      % Initial internal resistance (Ohm)
            'cycle_life', 1000, ... % Expected full cycles
            'Ts', 1, ...            % Sample time (s)
            'simCycles', 200 ...    % Number of cycles to simulate
        );
        
        % Generate simulated aging data
        [cycles, capacity, resistance] = generateAgingData(params);
        
        % SOH Estimation Methods
        soh_cf = curveFittingSOH(params, cycles, capacity, resistance);
        soh_kf = kalmanFilterSOH(params, cycles, capacity, resistance);
        
        % Plot results
        plotResults(cycles, capacity, resistance, soh_cf, soh_kf);
        
    catch ME
        fprintf('Error in SOH Estimation: %s\n', ME.message);
        return;
    end
end

function [cycles, capacity, resistance] = generateAgingData(params)
    % Generate simulated battery aging data
    try
        cycles = 0:params.simCycles;
        n = length(cycles);
        
        % Capacity fade model (exponential decay with noise)
        capacity = params.Q_nom * (1 - 0.2*(cycles/params.cycle_life).^0.8);
        capacity = capacity + 0.01*randn(1,n);
        capacity = max(0, capacity); % Ensure non-negative
        
        % Resistance growth model (linear increase with noise)
        resistance = params.R0_nom * (1 + 0.5*(cycles/params.cycle_life));
        resistance = resistance + 0.005*randn(1,n);
        resistance = max(params.R0_nom, resistance); % Minimum at initial value
        
    catch ME
        error('Error generating aging data: %s', ME.message);
    end
end

function soh = curveFittingSOH(params, cycles, capacity, resistance)
    % SOH estimation using curve fitting
    try
        % Normalize measurements
        cap_fade = (params.Q_nom - capacity) / params.Q_nom;
        res_growth = (resistance - params.R0_nom) / params.R0_nom;
        
        % Fit exponential decay model for capacity
        cap_model = fit(cycles', cap_fade', 'exp1');
        cap_soh = 1 - cap_model(cycles);
        
        % Fit linear growth model for resistance
        res_model = fit(cycles', res_growth', 'poly1');
        res_soh = 1 - res_model(cycles);
        
        % Combined SOH (weighted average)
        w_cap = 0.7; % Weight for capacity-based SOH
        w_res = 0.3; % Weight for resistance-based SOH
        soh = w_cap * cap_soh + w_res * res_soh;
        soh = max(0, min(1, soh)); % Bound between 0 and 1
        
    catch ME
        error('Error in curve fitting SOH: %s', ME.message);
    end
end

function soh = kalmanFilterSOH(params, cycles, capacity, resistance)
    % SOH estimation using Kalman Filter
    try
        n = length(cycles);
        soh = zeros(1,n);
        soh(1) = 1; % Initial SOH = 100%
        
        % Kalman Filter Parameters
        Q = 1e-6;     % Process noise covariance
        R = 1e-4;     % Measurement noise covariance
        P = 0.01;     % Initial error covariance
        
        for k = 2:n
            % Process model: SOH decreases with cycles
            soh_pred = soh(k-1) - (1/params.cycle_life); % Simple linear degrade
            P_pred = P + Q;
            
            % Measurement model: combine capacity and resistance
            cap_soh = capacity(k)/params.Q_nom;
            res_soh = params.R0_nom/resistance(k);
            z = 0.7*cap_soh + 0.3*res_soh; % Weighted measurement
            
            % Measurement update
            H = 1; % Observation matrix (direct measurement)
            K = P_pred*H/(H*P_pred*H + R); % Kalman Gain
            soh(k) = soh_pred + K*(z - soh_pred);
            P = (1 - K*H)*P_pred;
            
            soh(k) = max(0, min(1, soh(k)));
        end
        
    catch ME
        error('Error in Kalman Filter SOH: %s', ME.message);
    end
end

function plotResults(cycles, capacity, resistance, soh_cf, soh_kf)
    % Plot estimation results
    try
        figure('Name', 'SOH Estimation Results', 'NumberTitle', 'off');
        
        % Capacity Plot
        subplot(3,1,1);
        plot(cycles, capacity, 'b-', 'LineWidth', 1.5);
        xlabel('Cycles');
        ylabel('Capacity (Ah)');
        title('Battery Capacity Fade');
        grid on;
        
        % Resistance Plot
        subplot(3,1,2);
        plot(cycles, resistance, 'r-', 'LineWidth', 1.5);
        xlabel('Cycles');
        ylabel('Resistance (Ohm)');
        title('Internal Resistance Growth');
        grid on;
        
        % SOH Plot
        subplot(3,1,3);
        plot(cycles, soh_cf*100, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Curve Fitting');
        hold on;
        plot(cycles, soh_kf*100, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Kalman Filter');
        xlabel('Cycles');
        ylabel('SOH (%)');
        title('State of Health Estimation');
        legend('Location', 'best');
        grid on;
        
    catch ME
        warning('Error in plotting: %s', ME.message);
    end
end