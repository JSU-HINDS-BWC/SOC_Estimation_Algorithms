classdef soh_tests < matlab.unittest.TestCase
    properties
        params
    end
    
    methods(TestClassSetup)
        function setupParams(testCase)
            % Setup common parameters for all tests
            testCase.params = struct(...
                'Q_nom', 2.3, ...       % Initial nominal capacity (Ah)
                'R0_nom', 0.1, ...      % Initial internal resistance (Ohm)
                'cycle_life', 1000, ... % Expected full cycles
                'Ts', 1, ...            % Sample time (s)
                'simCycles', 200 ...    % Number of cycles to simulate
            );
        end
    end
    
    methods(Test)
        function testGenerateAgingData(testCase)
            % Test the aging data generation function
            [cycles, capacity, resistance] = generateAgingData(testCase.params);
            
            % Verify output sizes
            testCase.verifyEqual(length(cycles), testCase.params.simCycles + 1, ...
                'Cycle array length mismatch');
            testCase.verifyEqual(length(capacity), length(cycles), ...
                'Capacity array length mismatch');
            testCase.verifyEqual(length(resistance), length(cycles), ...
                'Resistance array length mismatch');
            
            % Verify physical bounds
            testCase.verifyTrue(all(capacity >= 0), ...
                'Capacity should be non-negative');
            testCase.verifyTrue(all(resistance >= testCase.params.R0_nom), ...
                'Resistance should not be below initial value');
            
            % Verify initial conditions
            testCase.verifyGreaterThan(capacity(1), 0.9*testCase.params.Q_nom, ...
                'Initial capacity too low');
            testCase.verifyLessThan(resistance(1), 1.1*testCase.params.R0_nom, ...
                'Initial resistance too high');
        end
        
        function testCurveFittingSOH(testCase)
            % Test the curve fitting SOH estimation
            [cycles, capacity, resistance] = generateAgingData(testCase.params);
            soh = curveFittingSOH(testCase.params, cycles, capacity, resistance);
            
            % Verify output size and bounds
            testCase.verifyEqual(length(soh), length(cycles), ...
                'SOH array length mismatch');
            testCase.verifyTrue(all(soh >= 0 & soh <= 1), ...
                'SOH values out of bounds [0,1]');
            
            % Verify monotonicity (with some tolerance for noise)
            diffs = diff(soh);
            testCase.verifyTrue(all(diffs <= 0.05), ...
                'SOH should generally decrease');
            
            % Verify initial and final values
            testCase.verifyGreaterThan(soh(1), 0.95, ...
                'Initial SOH should be near 1');
            testCase.verifyLessThan(soh(end), 0.95, ...
                'Final SOH should decrease significantly');
        end
        
        function testKalmanFilterSOH(testCase)
            % Test the Kalman Filter SOH estimation
            [cycles, capacity, resistance] = generateAgingData(testCase.params);
            soh = kalmanFilterSOH(testCase.params, cycles, capacity, resistance);
            
            % Verify output size and bounds
            testCase.verifyEqual(length(soh), length(cycles), ...
                'SOH array length mismatch');
            testCase.verifyTrue(all(soh >= 0 & soh <= 1), ...
                'SOH values out of bounds [0,1]');
            
            % Verify initial condition
            testCase.verifyEqual(soh(1), 1, ...
                'Initial SOH should be 1');
            
            % Verify reasonable final value
            testCase.verifyLessThan(soh(end), 0.95, ...
                'Final SOH should decrease significantly');
        end
        
        function testExtremeInputs(testCase)
            % Test both SOH methods with extreme inputs
            cycles = 0:10;
            capacity = zeros(1,11); % Extreme case: no capacity
            resistance = ones(1,11)*testCase.params.R0_nom;
            
            soh_cf = curveFittingSOH(testCase.params, cycles, capacity, resistance);
            soh_kf = kalmanFilterSOH(testCase.params, cycles, capacity, resistance);
            
            % Verify bounds hold even with extreme inputs
            testCase.verifyTrue(all(soh_cf >= 0 & soh_cf <= 1), ...
                'Curve Fitting SOH out of bounds with zero capacity');
            testCase.verifyTrue(all(soh_kf >= 0 & soh_kf <= 1), ...
                'Kalman Filter SOH out of bounds with zero capacity');
        end
        
        function testHighResistance(testCase)
            % Test with extremely high resistance
            cycles = 0:10;
            capacity = ones(1,11)*testCase.params.Q_nom;
            resistance = 10*ones(1,11); % Very high resistance
            
            soh_cf = curveFittingSOH(testCase.params, cycles, capacity, resistance);
            soh_kf = kalmanFilterSOH(testCase.params, cycles, capacity, resistance);
            
            testCase.verifyTrue(all(soh_cf >= 0 & soh_cf <= 1), ...
                'Curve Fitting SOH out of bounds with high resistance');
            testCase.verifyTrue(all(soh_kf >= 0 & soh_kf <= 1), ...
                'Kalman Filter SOH out of bounds with high resistance');
        end
    end
end