function  myplotResult(DATAdir,cfg)
    %% Settings
    % warning('off','all');
    % 読み込み
    if nargin < 2
        clear variables; close all; clc;
        tmp     = matlab.desktop.editor.getActive;
        DATAdir = cd(fileparts(tmp.Filename));

        % File  = strcat(DATAdir, '/info.mat');
        % load(File, 'cfg');
    end
    
    File  = strcat(DATAdir, '/userLocal_tmp.mat');
    load(File, 'tmp');
    
    File  = strcat(DATAdir, '/userLocal.mat');
    load(File, 'userLocal');

    %% make folda
    Outputdir = strcat(DATAdir,'/result');
    mkdir(Outputdir,'jpeg');
    mkdir(Outputdir,'eps');
    mkdir(Outputdir,'fig');
    mkdir(Outputdir,'png');
    mkdir(Outputdir,'pdf');
    
    %% データ処理
    tmp_cat=cat(1,tmp{:,1});
    tmp_Estimator=cat(1,tmp_cat.Estimator);
    tmp_RawData=cat(1,tmp_Estimator.RawData);
    tmp_NDT_info=cat(1,tmp_RawData.NDT_info);
    
    ndt.downsampled_points = cat(1,tmp_NDT_info.downsampled_points);
    ndt.matching_time = cat(1,tmp_NDT_info.matching_time);
    ndt.convergence = cat(1,tmp_NDT_info.convergence);
    ndt.score = cat(1,tmp_NDT_info.score);
    ndt.angle_diff = cat(1,tmp_NDT_info.angle_diff);
    
    state = [userLocal.X,userLocal.Y,userLocal.Z,...
            userLocal.Roll,userLocal.Pitch,userLocal.Yaw];
    state = state -[state(1,1:2),0,0,0,0];
    
    T = seconds(userLocal.Time)-seconds(userLocal.Time(1));
    dt = T-[0;T(1:end-1)];
    
    %% プロット
    figure
    box on;hold on;
    plot(T,ndt.matching_time)
    plot_average(T,ndt.matching_time);
    xlabel('Time[s]')
    ylabel('Matching Time[s]')
    xlim(T([1,end],:)')
    saveFigure('ndt_matching_time', Outputdir)
    
    figure
    box on;hold on;
    plot(T,ndt.angle_diff)
    [~,av] =plot_average(T,ndt.angle_diff);
    xlabel('Time [s]')
    ylabel('Angle diff')
    xlim(T([1,end],:)')
    saveFigure('ndt_angle_diff', Outputdir)
    
    figure
    box on; hold on;
    yyaxis left
    plot(T, state(:,1));
    plot(T, state(:,2));
    ylabel('Position (X, Y) [m]')
    yyaxis right
    plot(T, state(:,3));
    ylabel('Position (Z) [m]')
    xlabel('Time [s]')
    xlim(T([1,end],:)')
    legend('X','Y','Z')
    saveFigure('Position', Outputdir)

    figure
    box on; hold on;
    yyaxis left
    plot(T, state(:,4));
    plot(T, state(:,5));
    ylabel('Angle (Roll, Pitch) [rad]')
    yyaxis right
    plot(T, state(:,6));
    ylabel('Angle (Yaw) [rad]')
    xlabel('Time [s]')
    xlim(T([1,end],:)')
    legend('Roll','Pitch','Yaw')
    saveFigure('Angle', Outputdir)
end


function [p,average] = plot_average(T,data)
    average = sum(data)/length(data);
    p = plot(T([1,end],:),[average,average]);
end

function saveFigure(Name, Outputdir)
	hold off;
	saveas(gcf,strcat(Outputdir,strcat('/eps/',Name)),'epsc');
    saveas(gcf,strcat(Outputdir,strcat('/jpeg/',Name)),'jpeg');
	savefig(gcf,strcat(Outputdir,strcat('/fig/',Name)));
end