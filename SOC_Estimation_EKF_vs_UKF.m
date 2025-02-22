function SOC_Estimation_Robust()
    % Clear command window and workspace
    clc;
    clear;
    
    % Main try-catch block for error handling
    try
        % Battery Parameters
        params = struct(...
            'Q_nom', 2.3, ...      % Nominal capacity (Ah)
            'R0', 0.1, ...         % Internal resistance (Ohm)
            'SOC_init', 0.8, ...   % Initial SOC (0-1)
            'V_ocv', @(soc) 3.2 + 0.7*soc + 0.1*soc.^2, ... % Nonlinear OCV function
            'Ts', 1, ...           % Sample time (s)
            'simTime', 3600 ...    % Simulation time (s)
        );
        
        % Generate simulated battery data
        [time, current, voltage] = generateBatteryData(params);
        
        % SOC Estimation Methods
        [soc_ekf, P_ekf] = extendedKalmanFilter(params, time, current, voltage);
        [soc_ukf] = unscentedKalmanFilter(params, time, current, voltage);
        
        % Plotting results
        plotResults(time, soc_ekf, soc_ukf, current, voltage);
        
    catch ME
        fprintf('Error in SOC Estimation: %s\n', ME.message);
        return;
    end
end

function [time, current, voltage] = generateBatteryData(params)
    % Generate simulated battery measurement data
    try
        time = 0:params.Ts:params.simTime;
        n = length(time);
        
        % Generate realistic current profile
        current = -0.1 + 0.05*randn(1,n); % Base discharge with noise
        pulse_idx = randi([1 n], 1, 15);  % Random pulse locations
        current(pulse_idx) = -1.5;        % Discharge pulses
        
        % Calculate voltage using battery model
        soc = params.SOC_init;
        voltage = zeros(1,n);
        
        for k = 1:n
            voltage(k) = params.V_ocv(soc) - current(k)*params.R0 + 0.01*randn;
            soc = soc - (current(k)*params.Ts)/(params.Q_nom*3600);
            soc = max(0, min(1, soc));
        end
        
    catch ME
        error('Error generating battery data: %s', ME.message);
    end
end

function [soc, P] = extendedKalmanFilter(params, time, current, voltage)
    % Extended Kalman Filter SOC estimation
    try
        n = length(time);
        soc = zeros(1,n);
        soc(1) = params.SOC_init;
        
        % EKF Parameters
        Q = 1e-5;          % Process noise covariance
        R = 0.01;          % Measurement noise covariance
        P = 0.1;           % Initial error covariance
        
        for k = 2:n
            delta_t = time(k) - time(k-1);
            soc_pred = soc(k-1) - (current(k-1)*delta_t)/(params.Q_nom*3600);
            P_pred = P + Q;
            
            % Linearize measurement model
            H = 0.7 + 0.2*soc_pred;
            V_pred = params.V_ocv(soc_pred) - current(k-1)*params.R0;
           
            K = P_pred*H/(H*P_pred*H + R); % Kalman Gain
            soc(k) = soc_pred + K*(voltage(k) - V_pred);
            P = (1 - K*H)*P_pred;
            
            soc(k) = max(0, min(1, soc(k)));
        end
        
    catch ME
        error('Error in EKF: %s', ME.message);
    end
end

function soc = unscentedKalmanFilter(params, time, current, voltage)
    % Unscented Kalman Filter SOC estimation
    try
        n = length(time);
        soc = zeros(1,n);
        soc(1) = params.SOC_init;
        
        % UKF Parameters
        Q = 1e-5;          
        R = 0.01;          
        P = 0.1;           
        alpha = 1e-3;      
        kappa = 0;         
        beta = 2;          
        
        L = 1;             
        lambda = alpha^2*(L + kappa) - L;
        Wm = [lambda/(L+lambda) 0.5/(L+lambda)*ones(1,2*L)]; 
        Wc = Wm;
        Wc(1) = Wc(1) + (1-alpha^2+beta); 
        
        for k = 2:n
            delta_t = time(k) - time(k-1);
            
            sigma = sqrt((L+lambda)*P);
            X = [soc(k-1) soc(k-1)-sigma soc(k-1)+sigma];
            
            X_pred = X - (current(k-1)*delta_t)/(params.Q_nom*3600);
            soc_pred = sum(Wm .* X_pred);
            P_pred = Q;
            for i = 1:(2*L+1)
                P_pred = P_pred + Wc(i)*(X_pred(i)-soc_pred)^2;
            end
            
            Z_pred = params.V_ocv(X_pred) - current(k-1)*params.R0;
            z_mean = sum(Wm .* Z_pred);
            
            Pzz = R;
            Pxz = 0;
            for i = 1:(2*L+1)
                Pzz = Pzz + Wc(i)*(Z_pred(i)-z_mean)^2;
                Pxz = Pxz + Wc(i)*(X_pred(i)-soc_pred)*(Z_pred(i)-z_mean);
            end
            
            K = Pxz/Pzz;
            soc(k) = soc_pred + K*(voltage(k) - z_mean);
            P = P_pred - K*Pzz*K';
            
            soc(k) = max(0, min(1, soc(k)));
        end
        
    catch ME
        error('Error in UKF: %s', ME.message);
    end
end

function plotResults(time, soc_ekf, soc_ukf, current, voltage)
    try
        figure('Name', 'SOC Estimation Results', 'NumberTitle', 'off');
        
        % SOC Plot
        subplot(3,1,1);
        plot(time/60, soc_ekf*100, 'b-', 'LineWidth', 1.5, 'DisplayName', 'EKF');
        hold on;
        plot(time/60, soc_ukf*100, 'r--', 'LineWidth', 1.5, 'DisplayName', 'UKF');
        xlabel('Time (min)');
        ylabel('SOC (%)');
        title('State of Charge Estimation');
        legend('Location', 'best');
        grid on;
        
        % Current Plot
        subplot(3,1,2);
        plot(time/60, current, 'k-', 'LineWidth', 1.5);
        xlabel('Time (min)');
        ylabel('Current (A)');
        title('Battery Current');
        grid on;
        
        % Voltage Plot
        subplot(3,1,3);
        plot(time/60, voltage, 'm-', 'LineWidth', 1.5);
        xlabel('Time (min)');
        ylabel('Voltage (V)');
        title('Battery Voltage');
        grid on;
        
    catch ME
        warning('Error in plotting: %s', ME.message);
    end
end