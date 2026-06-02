clear all;close all;clc;
filename='RRT_Data.xlsx';
delete (filename);
delete ('RRT_Data_test_t.xlsx');
ss = stateSpaceSE2;
map=Coppelia_map();map = occupancyMap(map,1);
sv = validatorOccupancyMap(ss);
sv.Map = map;sv.ValidationDistance = 0.001;sv.Map.OccupiedThreshold=0.2;
sv.Map.FreeThreshold=0.01;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
planner = plannerRRT(ss,sv);
map.show; hold on;
start = [4.2030,23.2024,pi/2];
goal = [10,12,pi/2];
rac_start = DrawRectangle(start(1), start(2),start(3));
patch( rac_start(1,:),rac_start(2,:),'g')
rac = DrawRectangle(goal(1), goal(2),goal(3));
patch( rac(1,:),rac(2,:),'r')
rng(100,'twister');
[pthObj,solnInfo] = planner.plan(start,goal);
options = optimizePathOptions;
options.MinTurningRadius = 2;
options.MaxPathStates = size(pthObj.States,1) * 3;
options.ObstacleSafetyMargin = 0.75;
optpath = optimizePath(pthObj.States,map,options);
pose_data=zeros(2,4,max(size(pthObj.States)));
record_rac = zeros(2,5,max(size(pthObj.States)));
%% Visualize the results.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
for i=1:max(size(optpath))
    rac_pose = DrawRectangle(optpath(i,1),optpath(i,2),optpath(i,3));
    pose_data(1,:,i) = rac_pose(1,1:4);
    pose_data(2,:,i) = rac_pose(2,1:4);
    patch(rac_pose(1,1:4),rac_pose(2,1:4),'g');hold on
    if (i+1)==max(size(optpath))
        break
    end
    record_rac(:,:,i) = rac_pose;
    drawnow
end
plot(pthObj.States(:,1), pthObj.States(:,2),'r','LineWidth',2) % draw path  
types={'leg_1_x';'leg_1_y';'leg_2_x';'leg_2_y';'leg_3_x';'leg_3_y';'leg_4_x';'leg_4_y'};
leg_1_x=squeeze(pose_data(1,1,:));leg_1_y=squeeze(pose_data(2,1,:));
leg_2_x=squeeze(pose_data(1,2,:));leg_2_y=squeeze(pose_data(2,2,:));
leg_3_x=squeeze(pose_data(1,3,:));leg_3_y=squeeze(pose_data(2,3,:));
leg_4_x=squeeze(pose_data(1,4,:));leg_4_y=squeeze(pose_data(2,4,:));
leg_1_x=nonzeros(leg_1_x);leg_1_y=nonzeros(leg_1_y);
leg_2_x=nonzeros(leg_2_x);leg_2_y=nonzeros(leg_2_y);
leg_3_x=nonzeros(leg_3_x);leg_3_y=nonzeros(leg_3_y);
leg_4_x=nonzeros(leg_4_x);leg_4_y=nonzeros(leg_4_y);
T=table(leg_1_x,leg_1_y,leg_2_x,leg_2_y,leg_3_x,leg_3_y,leg_4_x,leg_4_y);
leg_1_x_diff = -1*flip(diff(flip(leg_1_x)));leg_1_y_diff = -1*flip(diff(flip(leg_1_y)));
leg_2_x_diff = -1*flip(diff(flip(leg_2_x)));leg_2_y_diff = -1*flip(diff(flip(leg_2_y)));
leg_3_x_diff = -1*flip(diff(flip(leg_3_x)));leg_3_y_diff = -1*flip(diff(flip(leg_3_y)));
leg_4_x_diff = -1*flip(diff(flip(leg_4_x)));leg_4_y_diff = -1*flip(diff(flip(leg_4_y)));
path_angle = 90;
path_angle   = [path_angle (optpath(2:end-1,3)*180/pi)'];
path_angle   = (path_angle-90)*pi/180;path_angle = path_angle';
t = path_angle(1);
path_angle   = -1*flip(diff(flip(path_angle)));
path_angle   = [t; path_angle(2:end)];
final_T_Diff=table(leg_1_x_diff,leg_1_y_diff,...
              leg_2_x_diff,leg_2_y_diff,...
              leg_3_x_diff,leg_3_y_diff,...
              leg_4_x_diff,leg_4_y_diff,...
              path_angle);
final_matrix=makeparab(T);
types_new={'leg_1_x';'leg_1_y';'leg_1_z';'leg_2_x';'leg_2_y';'leg_2_z';...
           'leg_3_x';'leg_3_y';'leg_3_z';'leg_4_x';'leg_4_y';'leg_4_z'};
leg_1_x_n=final_matrix(:,1);leg_1_y_n=final_matrix(:,2);leg_1_z_n=final_matrix(:,3);
leg_2_x_n=final_matrix(:,4);leg_2_y_n=final_matrix(:,5);leg_2_z_n=final_matrix(:,6);
leg_3_x_n=final_matrix(:,7);leg_3_y_n=final_matrix(:,8);leg_3_z_n=final_matrix(:,9);
leg_4_x_n=final_matrix(:,10);leg_4_y_n=final_matrix(:,11);leg_4_z_n=final_matrix(:,12);
final_T=table(leg_1_x_n,leg_1_y_n,leg_1_z_n,...
              leg_2_x_n,leg_2_y_n,leg_2_z_n,...
              leg_3_x_n,leg_3_y_n,leg_3_z_n,...
              leg_4_x_n,leg_4_y_n,leg_4_z_n);
writetable(final_T_Diff,filename);
leg_1_x_t_t=diff(leg_1_x);leg_1_y_t_t=diff(25-leg_1_y);
leg_2_x_t_t=diff(leg_2_x);leg_2_y_t_t=diff(25-leg_2_y);
leg_3_x_t_t=diff(leg_3_x);leg_3_y_t_t=diff(25-leg_3_y);
leg_4_x_t_t=diff(leg_4_x);leg_4_y_t_t=diff(25-leg_4_y);
FL = [4.4922 1.9581];
FR = [4.5209 1.6238];
BL = [3.8967 1.9275];
BR = [3.9137 1.5931];
FL_X = leg_4_x_t_t;
FL_Y = leg_4_y_t_t;
FR_X = leg_1_x_t_t;
FR_Y = leg_1_y_t_t;
BL_X = leg_3_x_t_t;
BL_Y = leg_3_y_t_t;
BR_X = leg_2_x_t_t;
BR_Y = leg_2_y_t_t;
leg_x_FL = zeros(size(leg_4_x_t_t));
leg_y_FL = zeros(size(leg_4_x_t_t));
leg_x_FR = zeros(size(leg_4_x_t_t));
leg_y_FR = zeros(size(leg_4_x_t_t));
leg_x_BL = zeros(size(leg_4_x_t_t));
leg_y_BL = zeros(size(leg_4_x_t_t));
leg_x_BR = zeros(size(leg_4_x_t_t));
leg_y_BR = zeros(size(leg_4_x_t_t));
for i = 1:length(FL_X)
    FL(1,1) = FL(1,1) +  FL_Y(i);
    FL(1,2) = FL(1,2) +  FL_X(i);
    leg_x_FL(i) = FL(1,1);
    leg_y_FL(i) = FL(1,2);
    FR(1,1) = FR(1,1) +  FR_Y(i);
    FR(1,2) = FR(1,2) +  FR_X(i);
    leg_x_FR(i) = FR(1,1);
    leg_y_FR(i) = FR(1,2);
    BL(1,1) = BL(1,1) +  BL_Y(i);
    BL(1,2) = BL(1,2) +  BL_X(i);
    leg_x_BL(i) = BL(1,1);
    leg_y_BL(i) = BL(1,2);
    BR(1,1) = BR(1,1) +  BR_Y(i);
    BR(1,2) = BR(1,2) +  BR_X(i);
    leg_x_BR(i) = BR(1,1);
    leg_y_BR(i) = BR(1,2);
end
final_T_Diff_t=table(leg_1_x_t_t,leg_1_y_t_t,...
                         leg_2_x_t_t,leg_2_y_t_t,...
                         leg_3_x_t_t,leg_3_y_t_t,...
                         leg_4_x_t_t,leg_4_y_t_t,...
                         path_angle,leg_y_FL,leg_x_FL,...
                         leg_y_FR,leg_x_FR,leg_y_BL,...
                         leg_x_BL,leg_y_BR,leg_x_BR);
filename_t='RRT_Data_test_t.xlsx';
writetable(final_T_Diff_t,filename_t);


