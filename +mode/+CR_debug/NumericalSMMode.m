classdef NumericalSMMode < mode.ModeStrategy
    properties
        plantmodel
        Uc
        init = 1
        flag
    end
    methods
        function obj = NumericalSMMode(cfg)
            disp("Setting up numerical simulation mode")
            obj.plantmodel = cfg.plant;
            obj.Uc.V = [0;0];
        end
        function setup(obj), end
        function [data,Plant] = receiveData(obj)
            data = [];
            obj.flag = 0;
            if obj.init == 0
                obj.Uc.V = obj.plantmodel.U;
            end
            [~,Plant] = obj.plantmodel.main(obj.Uc,obj.flag);
            obj.init = 0;
        end
        function exeProcess(~,~), end
        function sendData(obj, cmd)
            obj.flag = 1;
            obj.plantmodel = obj.plantmodel.main(cmd,obj.flag);
            obj.plantmodel.U = cmd.V;
        end
        function shutdown(obj)
            close(findall(groot, 'Type', 'figure'));
            disp("Finished.")
        end
    end
end


