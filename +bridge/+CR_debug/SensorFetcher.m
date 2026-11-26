classdef SensorFetcher < handle
    % 各センサデータのメッセージをsubscriberから受け取り，使いやすい形に整理するクラス

    properties
        mode
        sensorIdx
        baseSensor
        % sensor(5) の TF の引き方（cfg.getNewTf）．後輪間中心（Plant）はどちらも最新の TF
        %   false：LiDAR の姿勢は点群の stamp の時刻の TF（引けなければ最新）
        %   true ：LiDAR の姿勢も最新の TF
        getNewTf = false;

    end
    properties (Access=private)
        prevPlantPose   % 前の周期の Plant の姿勢（角度は調整後．角度を連続にするのに使う．まだ無ければ []）
        prevLidarPose   % 前の周期の LiDAR の姿勢（同上）
        timeout = 0.05; % [s] base_sensor のメッセージを待つ上限
        tftree          % ros2tf（/tf・/tf_static を溜める．ROS2CommManager が setTfTree で渡す）
        frames          % TF のフレーム名（struct: map, base, lidar）

    end
    properties (Constant)
        t_cr = 0.495; % Tread width of CR
    end
    properties (Constant, Access=private)
        angleNames = ["Roll","Pitch","Yaw"]; % 前の周期から連続にする角度のフィールド
    end

    methods
        function obj = SensorFetcher(cfg)
            % 設定 cfg（modeNumber，vehicleInfo.sensor・base_sensor，省略できる getNewTf）から作る．
            % 使えないセンサの組み合わせはエラーにする
            obj.mode = cfg.modeNumber;
            obj.sensorIdx = cfg.vehicleInfo.sensor;
            if isfield(cfg,'getNewTf')
                obj.getNewTf = cfg.getNewTf;
            end
            obj.baseSensor = cfg.vehicleInfo.base_sensor;
            if ~obj.sensorIdx(1) && (obj.sensorIdx(4)||obj.sensorIdx(5)) && obj.mode >= 2
                error("Localizer need to LiDAR. Please confirm LiDAR's sensor-index is true.")
            end
            if obj.sensorIdx(4) && obj.mode == 3
                error("CR_debug does not subscribe to a SLAM pose on the real vehicle. Please use Matching (sensor 5).")
            end
        end

        function setTfTree(obj, tftree, frames)
            % sensor(5) の後輪間中心と LiDAR の姿勢を引く TF の木とフレーム名
            obj.tftree = tftree;
            obj.frames = frames;
            % 点群の stamp の TF が引けず最新の TF を使ったときの警告（getNewTf が false のとき）
            warning('on', 'CR_debug:LidarTfFallback');
        end

        function [data,Plant] = getSensorData(obj,sensorSubs,whillSubs)
            % subscriberからセンサ値を取得
            %   sensorSubs：センサの subscriber の cell（購読していないものは []），whillSubs：車両の subscriber の cell
            %   data：センサの値と LiDAR の姿勢，Plant：車両の姿勢（後輪間中心）と車輪の速度
            doProcessing = 0;
            % LIDAR_pose（sensor(5)）：LiDAR の姿勢（getNewTf が false なら点群の stamp の時刻，true なら最新）．
            % X, Y, Z, Roll, Pitch, Yaw，Stamp（使った TF の stamp [s]），
            % tform（取付 base → LiDAR．rigidtform3d で，LiDAR の座標の点を base の座標に直す．引けなければ []），
            % NDT_info（/localization_debug の最新．無ければ []）．map → LiDAR が引けない周期は []
            data = struct("LIDAR",[],"GNSS",[],"CAMERA",[],"IMU",[],"LIDAR_pose",[]);
            % Plant.Stamp：Plant に使った TF の stamp [s]（sensor(5)．最新の TF の stamp．引けなければ []）
            % Plant と LIDAR_pose の Roll, Pitch, Yaw は前の周期から連続にする（±π に収まらない）
            Plant = struct("X",[.0],"Y",[.0],"Z",[.0], "Roll",[.0],"Pitch",[.0],"Yaw",[.0],"odom",[0, 0, 0, 0],"Stamp",[]);
            if obj.mode == 1
                return;
            end
            tStart = tic;
            ret = cell(size(sensorSubs));
            while toc(tStart) < obj.timeout %true
                whillret = whillSubs{1}.LatestMessage;
                for i = 1:numel(sensorSubs)
                    if ~isempty(sensorSubs{i})
                        ret{i} = sensorSubs{i}.LatestMessage;
                    end
                end
                % 基準のセンサ（base_sensor．0 は基準なし）のメッセージがあれば抜ける
                if obj.baseSensor == 0 || ~isempty(ret{obj.baseSensor})
                    break;
                end
            end
            if ~isempty(whillret)
                switch obj.mode
                    case 2
                        Plant.odom(1) = whillret.twist.twist.linear.x; % Linear velocity V (m/s)
                        Plant.odom(2) = whillret.twist.twist.angular.z; % Angular velocity Omega (rad/s)
                        Plant.odom(3) = Plant.odom(1) - (obj.t_cr/2) * Plant.odom(2); % LeftMSpeed
                        Plant.odom(4) = Plant.odom(1) + (obj.t_cr/2) * Plant.odom(2); % RightMSpeed
                    case 3
                        Plant.odom(1) = (-whillret.x + whillret.y) * 1000/3600 / 2; % V (m/s)
                        Plant.odom(2) = -(whillret.x + whillret.y) * 1000/3600 / obj.t_cr; % Omega (rad/s)
                        Plant.odom(3) = whillret.x * 1000/3600; % LeftMSpeed
                        Plant.odom(4) = whillret.y * 1000/3600; % RightMSpeed
                    otherwise
                        error(strcat(strcat('Invalid sensor index in mode',obj.mode)))

                end
            end

            
            % ---- 受け取ったものをそのまま入れる ----
            data.LIDAR = ret{1};
            if obj.sensorIdx(2) && ~isempty(ret{2})
                data.GNSS = ret{2};
            end
            if obj.sensorIdx(3) && ~isempty(ret{3}) && numel(ret{3}.detections) ~= 0
                data.CAMERA = ret{3};
                doProcessing = 1;
            end

            % ---- LiDAR の姿勢（sensor 5）：新旧によらず，点群があれば毎周期 /tf から引く ----
            if obj.sensorIdx(5) && ~isempty(data.LIDAR)
                data.LIDAR_pose = obj.lidarPose(data.LIDAR, ret{5});
            end

            % ---- Plant：Gazebo の真値（sensor 4．/wheelchair/pose）か，最新の map → base（sensor 5） ----
            pose = [];
            if obj.sensorIdx(4)
                % 真値がまだ届いていなければ Plant は 0 のまま
                if ~isempty(ret{4})
                    pose = obj.poseOf(ret{4}.position, ret{4}.orientation, []);
                end
            elseif obj.sensorIdx(5)
                tf = obj.lookup(obj.frames.map, obj.frames.base);
                if ~isempty(tf)
                    pose = obj.poseOf(tf.transform.translation, tf.transform.rotation, obj.stampSec(tf));
                end
            end
            if ~isempty(pose)
                pose = obj.continueAngles(pose, "prevPlantPose");
                for name = string(fieldnames(pose))'
                    Plant.(name) = pose.(name);
                end
            end
            if doProcessing
                data.CAMERA.processed_masks = camera_postProcess(data.CAMERA);
            end

            function [masks] = camera_postProcess(msg)
                %% YoloDetectionArray
                % std_msgs/Header header
                % YoloDetection[] detections
                % # Overlay of all detections' mask
                % std_msgs/UInt8MultiArray masks
                % uint16 rows
                % uint16 cols
                %% YoloDetection
                % # Store detections infomation of each object
                % uint8 label # COCO Labels ID
                % uint16[4] bboxparam # [xmin,ymin,xmax,ymax]
                % uint32[180] hue # Hue histogram
                % float32 score
                % # BoolArray mask Obsolete (too heavy processing)
                detNum = numel(msg.detections);
                if detNum == 0
                    return
                end                
                masks = encode_png(msg);
            
            end
            
            function mask = encode_png(msg)
                % msg : yolo_msgs/YoloDetectionArray から受け取ったメッセージ
                % msg.masks.data に PNG 圧縮バイト列が uint8 のベクトルで格納されている前提

                % 1) MATLAB の uint8 を Java の byte[] (signed) に変換
                pngData = uint8(msg.masks.data)';                  % 列ベクトル化
                javaBytes = typecast(pngData, 'int8');             % Java byte[] は符号付き
            
                % 2) ByteArrayInputStream を生成
                import java.io.ByteArrayInputStream
                bais = ByteArrayInputStream(javaBytes);
            
                % 3) Java の ImageIO で PNG をデコード
                import javax.imageio.ImageIO
                bufImg = ImageIO.read(bais);
                if isempty(bufImg)
                    error('Failed to process ImageIO.read (Possibly byte array is NOT PNG format)');
                end
            
                % 4) グレースケール（mask）は DataBufferByte で取得
                raster = bufImg.getRaster;                         % WritableRaster
                db     = raster.getDataBuffer;                     % DataBuffer
                byteArr= db.getData();                             % int8 の配列
            
                % 5) MATLAB の uint8 配列に戻す
                byteArr = typecast(int8(byteArr), 'uint8');        % Java int8 -> MATLAB uint8
            
                % 6) 画像サイズを取得して reshape
                w = bufImg.getWidth;
                h = bufImg.getHeight;
                mask = reshape(byteArr, [w, h])';                  % 転置して (h×w) に
            
            end

        end
    end
    methods (Access=private)
        function pose = lidarPose(obj, cloud, ndtMsg)
            % LiDAR の姿勢（data.LIDAR_pose）を作る
            %   cloud：点群（PointCloud2），ndtMsg：/localization_debug の最新のメッセージ（無ければ []）
            %   pose ：poseOf の姿勢（角度は前の周期から連続）に tform と NDT_info を足したもの．map → LiDAR が引けなければ []
            % getNewTf が true なら最新の map → LiDAR，false なら点群の stamp の時刻（引けなければ最新．警告）
            if obj.getNewTf
                tf = obj.lookup(obj.frames.map, obj.frames.lidar);
            else
                tf = obj.lookup(obj.frames.map, obj.frames.lidar, cloud.header.stamp);
                if isempty(tf)
                    warning('CR_debug:LidarTfFallback', ...
                        'LiDAR: no TF at the point cloud stamp %.3f; using the latest TF if any', obj.stampSec(cloud));
                    tf = obj.lookup(obj.frames.map, obj.frames.lidar);
                end
            end
            pose = [];
            if isempty(tf)
                return;
            end
            pose = obj.poseOf(tf.transform.translation, tf.transform.rotation, obj.stampSec(tf));
            pose = obj.continueAngles(pose, "prevLidarPose");
            % 取付 base → LiDAR（/tf_static）
            pose.tform = obj.tformOf(obj.lookup(obj.frames.base, obj.frames.lidar));
            % NDT の情報（stamp が無いので，この点群の結果とは限らない）
            pose.NDT_info = obj.ndtInfoOf(ndtMsg);
        end

        function pose = continueAngles(obj, pose, prevName)
            % pose の Roll, Pitch, Yaw を前の周期の値（調整後）に最も近い角度にし，次の周期のために覚える
            %   pose    ：poseOf の姿勢（角度は ±π の範囲）
            %   prevName：前の周期の姿勢を持つプロパティの名前（"prevPlantPose" か "prevLidarPose"）
            %   出力    ：角度を連続にした pose（前の値が無い最初の周期は生の値のまま）
            prev = obj.(prevName);
            if ~isempty(prev)
                for name = obj.angleNames
                    pose.(name) = prev.(name) + mod(pose.(name) - prev.(name) + pi, 2*pi) - pi;
                end
            end
            obj.(prevName) = pose;
        end

        function tf = lookup(obj, parent, child, varargin)
            % parent → child の TF（TransformStamped）を引く（待たない）
            %   varargin：stamp（builtin_interfaces/Time）を渡せばその時刻，渡さなければ最新
            %   tf      ：引けなければ []（getTransform の例外はそのまま呼び出し元へ出す）
            tf = [];
            if canTransform(obj.tftree, parent, child, varargin{:})
                tf = getTransform(obj.tftree, parent, child, varargin{:});
            end
        end
    end
    methods (Static, Access=private)
        function t = stampSec(msg)
            % header.stamp を [s] にする
            t = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
        end

        function pose = poseOf(t, q, stamp)
            % 位置 t（x, y, z）と四元数 q（w, x, y, z）を X, Y, Z, Roll, Pitch, Yaw にする．Stamp [s]（無ければ []）
            eul = quat2eul([q.w, q.x, q.y, q.z],"ZYX");
            pose = struct("X",t.x,"Y",t.y,"Z",t.z,"Roll",eul(3),"Pitch",eul(2),"Yaw",eul(1),"Stamp",stamp);
        end

        function T = tformOf(tf)
            % TransformStamped（parent → child）を rigidtform3d にする（child の座標の点を parent の座標に直す）．tf が [] なら []
            T = [];
            if isempty(tf)
                return;
            end
            t = tf.transform.translation;
            q = tf.transform.rotation;
            T = rigidtform3d(quat2rotm([q.w, q.x, q.y, q.z]), [t.x, t.y, t.z]);
        end

        function info = ndtInfoOf(msg)
            % /localization_debug のメッセージ（Float32MultiArray）を NDT の情報の struct にする．msg が [] なら []
            info = [];
            if isempty(msg)
                return;
            end
            d = msg.data;
            info = struct("downsampled_points",d(1),"matching_time",d(2), ...
                "convergence",d(3),"score",d(4),"angle_diff",d(5));
        end
    end
end
