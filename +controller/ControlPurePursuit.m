classdef ControlPurePursuit < handle
    %% Controller
    %% Method
    % Control2: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% 制御結果の保存
    % 結果を.matとして保存し，plotResultで使用可能．
    % "result"を構造体とし，保存したい値を格納．
    properties(Constant)
        % For EXP.
        wp = [
                        0.101075205452564	-0.986210909687654;...
                        % 0.531040059492582	-1.86015557283729;...
                        1.29274072992395	-2.48013273739252;...
                        % 2.20748123337163	-2.85932721313020;...
                        3.17911500436239	-3.08388583932722;...
                        % 4.17729526300892	-3.14418651302440;...
                        5.17547552165546	-3.17939448395816;...
                        % 6.17365578030200	-3.11909381026106;...
                        7.16386923797351	-3.00949808732428;...
                        % 8.13022807680476	-2.75230050838404;...
                        8.99966659884077	-2.26603746660426;...
                        % 9.65192225870215	-1.52554519461122;...
                        9.96779093872548	-0.191940835339861;...
                        % 9.80229761072381	0.794270074347785;...
                        9.28884067236386	1.63277295265714;...
                        % 8.47595198939407	2.19638456159256;...
                        7.54002350871072	2.52550250051517;...
                        % 6.56219650882638	2.71174015895879;...
                        5.56401625017984	2.77204083265595;...
                        % 4.56583599153330	2.78377682296719;...
                        3.56765573288676	2.72347614927007;...
                        % 2.58363550410884	2.57555945857987;...
                        1.62651904618980	2.29651805647231;...
                        % 0.769026120477256	1.78202230104478;...
                        0.167958473154285	0.985164473431930;...
                        0	0
                ] + [0 1];

        % For Gazebo
        % wp = [
        %     5 0
        %     10 0
        %     15 0
        %     20 0
        %     25 0
        %     30 0
        %     35 0
        %     37 0
        %     38 1
        %     38 3.5
        %     38 6
        %     37 7
        %     35 7
        %     30 7
        %     25 7
        %     20 7
        %     15 7
        %     10 7
        %     5 7
        %     0 7
        %     -1 6.5
        %     -2 5
        %     -2 1
        %     -1 0.5
        %     0 0
        %     ];        

        % wp = [5,0;
        %       10,0;
        %       15,0;
        %       20,0];
    end

    properties
        vehicleType
        isMultiPC
        controller
        To = 0;
        % --- 車両パラメータ
        v = 0.2;                            % 車両の速度 [m/s] (一定とする)
        omega_max = 0.3;
        REACH_THRESHOLD = 1.0;              % ウェイポイントへの到達判定距離 [m]
        % --- 制御およびログ用の変数初期化
        current_wp_index = 1; % 現在の目標ウェイポイントのインデックス
        integral_error = 0;   % 積分誤差
        previous_error = 0;   % １ステップ前の誤差
        % --- PID制御器のゲイン設定
        Kp = 1.0;   % 比例ゲイン
        Ki = 0.0;   % 積分ゲイン
        Kd = 0.0;   % 微分ゲイン
    end

    methods
        function obj = ControlPurePursuit()
            % obj.controller = controllerPurePursuit(DesiredLinearVelocity=0.5,MaxAngularVelocity=0.2,LookaheadDistance=0.5,Waypoints=obj.wp);
            obj.controller = controllerPurePursuit(DesiredLinearVelocity=0.5,MaxAngularVelocity=obj.omega_max,LookaheadDistance=1,Waypoints=round(obj.wp(1:2,:),4));
        end

        function result = main(obj,Est,T)
            % [U,Omega,ld] = obj.controller([Est.pose(1),Est.pose(2),Est.pose(3)]); % [x y theta] 
            dt = T-obj.To;         % 時間ステップ [s]
            obj.To = T;
            %% -------------------- メインシミュレーションループ --------------------
            % 現在の車両状態を履歴に保存
            % vehicle_state = Est.pose';          % 車両の初期状態 [x(m), y(m), yaw(rad)]'
            % x = vehicle_state(1);
            % y = vehicle_state(2);
            % yaw = vehicle_state(6);

            x=0;
            y=0;
            yaw=0;
        
            % --- ウェイポイントの管理 ---
            % 全てのウェイポイントを通過した場合
            if obj.current_wp_index > size(obj.wp, 1)
                fprintf('全てのウェイポイントを通過しました。\n');
                obj.current_wp_index = 1;
            end
        
            % 現在の目標ウェイポイントを取得
            target_wp = obj.wp(obj.current_wp_index, :);
        
            % 目標までの距離と角度を計算
            dx = target_wp(1) - x;
            dy = target_wp(2) - y;
            dist_to_wp = sqrt(dx^2 + dy^2);

            % --- ウェイポイント到達判定 ---
            if dist_to_wp < obj.REACH_THRESHOLD
                fprintf('ウェイポイント %d に到達しました。\n', obj.current_wp_index);
                obj.current_wp_index = obj.current_wp_index + 1;
                % % 次のループで新しい目標が設定されるため、このステップの処理はスキップ
                % % if obj.current_wp_index > size(obj.wp, 1)
                % %     return;
                % % end
                if obj.current_wp_index > size(obj.wp, 1)-1
                    obj.controller.Waypoints = obj.wp([size(obj.wp,1),1],:);
                else
                    obj.controller.Waypoints = obj.wp(obj.current_wp_index:obj.current_wp_index+1,:);
                end
            end
            % 
            % % --- PID制御による操作量の計算 ---
            % % 1. 誤差（目標方位と現在方位の差）の計算
            % target_yaw = atan2(dy, dx);
            % error_yaw = target_yaw - yaw;
            % % 角度誤差を -pi から pi の範囲に正規化
            % error_yaw = atan2(sin(error_yaw), cos(error_yaw));
            % 
            % % 2. 積分項の計算
            % obj.integral_error = obj.integral_error + error_yaw * dt;
            % 
            % % 3. 微分項の計算
            % derivative_error = (error_yaw - obj.previous_error) / dt;
            % 
            % % 4. PID制御則に基づき操作量（角速度 omega）を計算
            % omega = obj.Kp * error_yaw + obj.Ki * obj.integral_error + obj.Kd * derivative_error;
            % 
            % % 5. 次のステップのために誤差を更新
            % obj.previous_error = error_yaw;

            %%% PurePursuit %%%
            
            [obj.v, omega] = obj.controller([x,y,yaw]);

            % if abs(omega) > obj.omega_max
            %     omega = omega/abs(omega)*obj.omega_max;
            %     warning('Max omega reached')
            % end
            % if Est.pose(1) > 16
            %     obj.v = 0;
            %     omega = 0;
            % end
            result.V = [obj.v;omega]; % wheelchair cmd_vel
            % result.V = [0;0];
            
            
        end
    end
end