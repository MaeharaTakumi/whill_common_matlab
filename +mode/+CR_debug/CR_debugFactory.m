classdef CR_debugFactory < mode.VehicleFactory
    methods
        function modes = getSupportedModes(obj)
            % Define supported mode in this vehicle
            modes = [0,1,2,3]; 
            % 0: Numerical Sim.
            % 1: Offline
            % 2: Gazebo
            % 3: Real Exp.
        end
        function modeObj = buildNumericalSM(obj, cfg)
            modeObj = mode.CR_debug.NumericalSMMode(cfg);
        end
        function modeObj = buildOffline(obj, cfg)
            modeObj = mode.CR_debug.OfflineMode(cfg);
        end
        function modeObj = buildGazebo(obj, cfg)
            modeObj = mode.CR_debug.GazeboMode(cfg);
        end
        function modeObj = buildReal(obj, cfg)
            modeObj = mode.CR_debug.RealMode(cfg);
        end
    end
end