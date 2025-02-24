classdef TestSOHEstimation < matlab.unittest.TestCase
    properties
        params
    end
    
    methods(TestClassSetup)
        function setupParamsAndPath(testCase)
            % Setup parameters and add path to tested functions
            testCase.params = struct(...
                'Q_nom', 2.3, ...       % Initial nominal capacity (Ah)
                'R0_nom', 0.1, ...      % Initial internal resistance (Ohm)
                'cycle_life', 1000, ... % Expected full cycles
                'Ts', 1, ...            % Sample time (s)
                'simCycles', 200 ...    % Number of cycles to simulate
            );
            
            % Add path to the folder containing the tested functions
            % Adjust this path based on your project structure
            testedFunctionsPath = fullfile(pwd, '..', 'src'); % Example: '../src'
            addpath(testedFunctionsPath);
            
            % Verify the functions are now accessible
            testCase.assertTrue(exist('generateAgingData', 'file') == 2, ...
                'generateAgingData function not found in path');
        end
    end
    
    methods(TestClassTeardown)
        function removePath(testCase)
            % Optional: Clean up by removing the added path
            testedFunctionsPath = fullfile(pwd, '..', 'src');
            rmpath(testedFunctionsPath);
        end
    end
    
    methods(Test)
        function testGenerateAgingData(testCase)
            [cycles, capacity, resistance] = generateAgingData(testCase.params);
            
            testCase.verifyEqual(length(cycles), testCase.params.simCycles + 1, ...
                'Cycle array length mismatch');
            testCase.verifyEqual(length(capacity), length(cycles), ...
                'Capacity array length mismatch');
            testCase.verifyEqual(length(resistance), length(cycles), ...
                'Resistance array length mismatch');
            
            testCase.verifyTrue(all(capacity >= 0), ...
                'Capacity should be non-negative');
            testCase.verifyTrue(all(resistance >= testCase.params.R0_nom), ...
                'Resistance should not be below initial value');
            
            testCase.verifyGreaterThan(capacity(1), 0.9*testCase.params.Q_nom, ...
                'Initial capacity too low');
            testCase.verifyLessThan(resistance(1), 1.1*testCase.params.R0_nom, ...
                'Initial resistance too high');
        end
        
        function testCurveFittingSOH(testCase)
            [cycles, capacity, resistance] = generateAgingData(testCase.params);
            soh = curveFittingSOH(testCase.params, cycles, capacity, resistance);
            
            testCase.verifyEqual(length(soh), length(cycles), ...
                'SOH array length mismatch');
            testCase.verifyTrue(all(soh >= 0 & soh <= 1), ...
                'SOH values out of bounds [0,1]');
            
            diffs = diff(soh);
            testCase.verifyTrue(all(diffs <= 0.05), ...
                'SOH should generally decrease');
            
            testCase.verifyGreaterThan(soh(1), 0.95, ...
                'Initial SOH should be near 1');
            testCase.verifyLessThan(soh(end), 0.95, ...
                'Final SOH should decrease significantly');
        end
        
        function testKalmanFilterSOH(testCase)
            [cycles, capacity, resistance] = generateAgingData(testCase.params);
            soh = kalmanFilterSOH(testCase.params, cycles, capacity, resistance);
            
            testCase.verifyEqual(length(soh), length(cycles), ...
                'SOH array length mismatch');
            testCase.verifyTrue(all(soh >= 0 & soh <= 1), ...
                'SOH values out of bounds [0,1]');
            
            testCase.verifyEqual(soh(1), 1, ...
                'Initial SOH should be 1');
            
            testCase.verifyLessThan(soh(end), 0.95, ...
                'Final SOH should decrease significantly');
        end
        
        function testExtremeInputs(testCase)
            cycles = 0:10;
            capacity = zeros(1,11);
            resistance = ones(1,11)*testCase.params.R0_nom;
            
            soh_cf = curveFittingSOH(testCase.params, cycles, capacity, resistance);
            soh_kf = kalmanFilterSOH(testCase.params, cycles, capacity, resistance);
            
            testCase.verifyTrue(all(soh_cf >= 0 & soh_cf <= 1), ...
                'Curve Fitting SOH out of bounds with zero capacity');
            testCase.verifyTrue(all(soh_kf >= 0 & soh_kf <= 1), ...
                'Kalman Filter SOH out of bounds with zero capacity');
        end
        
        function testHighResistance(testCase)
            cycles = 0:10;
            capacity = ones(1,11)*testCase.params.Q_nom;
            resistance = 10*ones(1,11);
            
            soh_cf = curveFittingSOH(testCase.params, cycles, capacity, resistance);
            soh_kf = kalmanFilterSOH(testCase.params, cycles, capacity, resistance);
            
            testCase.verifyTrue(all(soh_cf >= 0 & soh_cf <= 1), ...
                'Curve Fitting SOH out of bounds with high resistance');
            testCase.verifyTrue(all(soh_kf >= 0 & soh_kf <= 1), ...
                'Kalman Filter SOH out of bounds with high resistance');
        end
    end
end