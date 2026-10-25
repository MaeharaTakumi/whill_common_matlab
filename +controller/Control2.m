classdef Control2 < handle
    %% Controller
    %% Method
    % Control2: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% 制御結果の保存
    % "result"を構造体とし，保存したい値を格納．
    properties(Constant)
        
    end

    properties
        omega = 1.0;
        v = 1.0
        
        waite_time = 1.0
        times = 1.0
    end

    methods
        function obj = Control2()
            
        end

        function [result] = main(obj,Est,T)
            t = 2*pi/obj.omega;

            time1 = obj.waite_time;
            time2 = time1+obj.times*t;
            time3 = time2+obj.times*t;
            time4 = time3 +4;

            % result.V = [0;0];
            if T>time1 && T<=time2 
                result.V = [0;obj.omega];
            elseif T>time2 && T<=time3
                result.V = [0;-obj.omega];
            elseif T>time3 && T<=time4
                result.V = [obj.v;0];
            else
                result.V = [0;0];
            end
            
            % result.V = [0;1];

        end
    end
end