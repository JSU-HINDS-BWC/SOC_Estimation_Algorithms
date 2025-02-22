classdef SOC_Estimation_RobustTest < matlab.unittest.TestCase
    methods (Test)
        function testEKFOutput(testCase)
            % Define test parameters
            params = struct(...
                'Q_nom', 2.3, ...
                'R0', 0.1, ...
                'SOC_init', 0.8, ...
                'V_ocv', @(soc) 3.2 + 0.7*soc + 0.1*soc.^2, ...
                'Ts', 1, ...
                'simTime', 60 ...
            );
            
            % Generate test data
            [time, current, voltage] = generateBatteryData(params);
            
            % Run EKF
            [soc_ekf, ~] = extendedKalmanFilter(params, time, current, voltage);
            
            % Check that SOC estimates remain within valid range (0 to 1)
            testCase.verifyGreaterThanOrEqual(soc_ekf, 0, 'SOC should not be negative');
            testCase.verifyLessThanOrEqual(soc_ekf, 1, 'SOC should not exceed 1');
        end
    end
end
