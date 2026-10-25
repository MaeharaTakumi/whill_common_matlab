classdef RealMode < mode.ModeStrategy
    properties
        Comm
        manual
    end
    methods
        function obj = RealMode(cfg)
            disp("Setting up ROS2 configuration for experiment mode")
            setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
            setenv("FASTDDS_BUILTIN_TRANSPORTS","UDPv4") % Avoid SHM communication
            setenv("ROS_LOCALHOST_ONLY","0")
            obj.Comm = bridge.CR_debug.ROS2CommManager(cfg,3);
            obj.manual = cfg.vehicleInfo.manualCon;
        end
        function setup(obj)
            % obj.Comm.genSensorSubs();
            % obj.Comm.genWhillSubs();
            % obj.Comm.genWhillPubs();
            obj.Comm.generateSensorSubscribers();
            obj.Comm.generateVehicleSubscribers();
            obj.Comm.generateVehiclePublishers();
            
        end
        function [data,Plant] = receiveData(obj)
            [data,Plant] = obj.Comm.getSensorData();
        end
        function exeProcess(~,~), end
        function sendData(obj, cmd)
            if ~obj.manual, obj.Comm.sendVehicleCommand(cmd); end
        end
        function shutdown(obj)
            Spr = utils.SpinnerStatus('Shutting down the ROS2 node. This process will take 5 seconds...');
            close(findall(groot, 'Type', 'figure'));
            rosshutdown
            pause(5)
            Spr.done('Finished.')
        end
    end
end
