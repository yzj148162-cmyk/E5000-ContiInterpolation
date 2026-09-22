function plot_replay_summary(result)
time = result.sample.timeS;
desired = result.sample.desiredPose;
actual = result.actualPoseMmRad;

fig = figure('Visible','off','Color','w','Name','CDPR轨迹总览');
tiledlayout(fig,2,1,'TileSpacing','compact');
nexttile;
plot(time,desired(:,1:3),'--','LineWidth',1.1); hold on;
plot(time,actual(:,1:3),'-','LineWidth',1.0); grid on;
xlabel('时间 / s'); ylabel('位置 / mm');
legend({'x期望','y期望','z期望','x实际','y实际','z实际'}, ...
    'Location','bestoutside');
title('Newmark期望位置与八轴Trace正运动学位置');
nexttile;
translation = vecnorm(actual(:,1:3)-desired(:,1:3),2,2);
angle = atan2(sin(actual(:,4:6)-desired(:,4:6)), ...
              cos(actual(:,4:6)-desired(:,4:6)));
orientation = rad2deg(vecnorm(angle,2,2));
yyaxis left; plot(time,translation,'LineWidth',1.1); ylabel('平移误差 / mm');
yyaxis right; plot(time,orientation,'LineWidth',1.1); ylabel('姿态误差 / deg');
grid on; xlabel('时间 / s'); title('末端轨迹误差');
exportgraphics(fig,fullfile(result.outputFolder,'trajectory_overview.png'), ...
    'Resolution',150);
close(fig);
end
