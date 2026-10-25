%% Global configurations
clc; close all; clear global; clear variables; warning('off','all');
conf.pc = [ismac; isunix; ispc];
conf.mk = [":"; ":"; ";"];
conf.def= "AppData/Local/Temp/Editor";
conf.usr= pwd;
conf.fpath = split(path, conf.mk(conf.pc));
conf.fcheck= and(~contains(conf.fpath, matlabroot), ~contains(conf.fpath, conf.def));
rmpath(strjoin(conf.fpath(conf.fcheck), conf.mk(conf.pc)));
%% Control target
vehicleInfo.type = "CR_debug"; % "CR", "CR2", "Go2", "CR-SIP", etc.
%% Sensor configurations (Vehicle-specific options)
vehicleInfo.color = 'red'; % CR series EXP Only 'red'\'white'\'blue'\'green'
vehicleInfo.ROBOT_SPORT_API_ID_MOVE = int64(1008); % Go2 Only
vehicleInfo.sensor(1) = true; % LiDAR
vehicleInfo.sensor(2) = false; % GNSS
vehicleInfo.sensor(3) = false; % Camera
vehicleInfo.sensor(4) = false; % SLAM
vehicleInfo.sensor(5) = true; % Matching (EXP Only. CR_debug: base pose at the rear axle center)
vehicleInfo.sensor(6) = false; % IMU (CR2 Only)
vehicleInfo.sensor(7) = true; % Localization Debug
vehicleInfo.sensor(8) = true; % LiDAR pose at the LiDAR stamp (CR_debug Only. Paired with LiDAR)
vehicleInfo.base_sensor = 1; % Standard sensor you use mainly. No standard:0, LiDAR:1, GNSS:2, Camera:3
vehicleInfo.manualCon = false; % Enable Manual Control using Joystick (CR Only)
%% Timer configurations
tspan = 0.05; %(sec) Sensor frequency which is corresponded to standard sensor
overrunAct = 'slip'; % 'slip'(recommend) or 'drop': Action when control cycle delays occur
% See: https://jp.mathworks.com/help/robotics/ref/ratecontrol.html
%% Mode configurations
mode = 2; % 0:Numerical simulation, 1:Offline, 2:Gazebo simulation, 3:Real exp.
% Offline: Path to MAT file
% The number of time series data points required for execution is automatically detected.
offlinePath = "/path/to/your/userLocal.mat"; 
tend = 20; % Offline: If NO MAT file is loaded, set any time value (sec).
%% ROS2 configurations
RID = 11;
%% Module configurations
% Save file path
mySavePath = './data';
mySaveFileName = string(datetime("now","Format","yyyyMMdd_HHmmss")); 
Datadir = strcat(mySavePath,filesep,string(datetime("now","Format","yyyyMMdd")),filesep,mySaveFileName);
if ~exist(Datadir,"dir"), mkdir(Datadir); end

% Point cloud video after the run (plotter.myplotPointCloudVideo). Empty mapFile: scans only
makeVideo = true;
mapFile = "/home/student/Documents/Research/Docker/localization/docker/data/map/SC10_4_0.05.pcd"; % Gazebo map

% LiDAR Camera Caliblation File for Fusion
calibparamPath = "./cameracalibparam/prefer.mat";
cameraparamPath = "./cameracalibparam/internal_param_fix.mat";

% You can supply your own class instead of default if you need.
% addpath(genpath("./MyEstimate"))
plantcontainer = plant.PlantWH();
estimator = estimator.Estimate2(mode,offlinePath);
addpath(genpath("./JPDAF_tracker"))
% estimator = estimator.EstimateJPDAF(mode,offlinePath);
% Camera LiDAR Fusion package
% addpath(genpath("./LiDARCamera"))
% estimator = estimator.EstimateLC(mode,offlinePath,calibparamPath,cameraparamPath);
controller = controller.Control2();
% controller = controller.ControlPurePursuit();
logger = logger.DataLogger(Datadir,'tmp');

% Activation paralell worker
% if isempty(gcp('nocreate')), parpool; end

cfg = struct( ...
    "modeNumber"  , mode, ...
    "isParallel"  , false, ...
    "isMultiPC"   , false, ...
    "RID"         , RID, ...
    "vehicleInfo" , vehicleInfo, ...
    "tspan"       , tspan, ...
    "offlinePath" , offlinePath, ...
    "tend"        , tend, ...
    "rosNamespace", "matlab", ...
    "plant"       , plantcontainer, ...
    "estimator"   , estimator, ...
    "controller"  , controller, ...
    "logger"      , logger, ...
    "overrunAct"  , overrunAct);

sys = core.SystemFactory.build(cfg);
sys.run();

%% Plot
plotter.DataPlotter(Datadir,cfg)
plotter.myplotResult(Datadir,cfg)
if makeVideo, plotter.myplotPointCloudVideo(Datadir,mapFile); end
