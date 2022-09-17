function [report] = TrackDetect(Track, UAV)
%TRACKDETECT 判断航迹是否符合条件(一个agent的)

%% 无人机航迹检测

% 威胁检测
dim = UAV.PointDim;                                         % 仿真环境维度
M = [UAV.Menace.radar; UAV.Menace.other]; % 威胁区

Threat = cell(UAV.num, 1);                                 % 威胁结果树
Angle = cell(UAV.num, 1);                                  % 转角检测结果树
MiniTraj = cell(UAV.num, 1);                              % 最小航迹片段检测结果树
ProbPoint = cell(UAV.num, 1);                           % 问题点  

L = cell(UAV.num, 1);                                          % 航迹片段树（累加结构）
Time = cell(UAV.num, 1);                                    % 到达各个点时间树                                          
      
L_mt = 0;                                                               % 所有无人机航迹之和
totalTime = zeros(UAV.num, 1);                         % 所用时间
totalL = zeros(UAV.num, 1);                                % 每个无人机飞行距离

for i = 1 : UAV.num
      PointNum = UAV.PointNum(i);   
      Judge = zeros(1, PointNum+1);
      L_i = zeros(1, PointNum+1);
      Time_i = zeros(1, PointNum+1);
      Angle_i = zeros(1, PointNum+1);
      Traj_i = zeros(1, PointNum+1);
      ProbPoint_i = zeros(1, PointNum+1);
      % 进行检测
      l = 0;
      V = Track.V(i);
      for k = 1 : PointNum
            % 前向检测
            P2 = Track.P{i}(:, k)' ;   % 转置成 1*dim
            if k == 1
                P1 = UAV.S(i, :);
                ZZ = P2 - P1;
                phi0 = atan2(ZZ(2), ZZ(1));   % 偏角检测
                phi1 = phi0;
                d_phi = phi1 - phi0;
                if dim > 2                                % 倾角检测
                    theta0 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                    theta1 = theta0;
                    d_theta = theta1 - theta0;
                else
                    d_theta = 0;
                end
            else
                P1 = Track.P{i}(:, k-1)' ;  % 转置成 1*dim
                ZZ = P2 - P1;
                phi1 = atan2(ZZ(2), ZZ(1));
                d_phi = phi1 - phi0;
                phi0 = phi1;
                if dim > 2                                   
                    theta1 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                    d_theta = theta1 - theta0;
                    theta0 = theta1;
                else
                    d_theta = 0;
                end
            end
         
            [across, ~] = CheckThreat(P1, P2, M); % 威胁检测
            Judge(k) = across;
            
            dl = norm(P1 - P2);
            l = l + dl; % 累加距离
            t = l / V;  % 时间点
            L_i(k) = l;
            Time_i(k) = t;

            if abs(d_phi) > UAV.limt.phi(i, 2)  ||  abs(d_theta) > UAV.limt.theta(i, 2)
                Angle_i(k) = true;
            else
                Angle_i(k) = false;
            end

            if dl < UAV.limt.L(i, 1)
                Traj_i(k) = true;
            else
                Traj_i(k) = false;
            end

            ProbPoint_i(k) = Angle_i(k) | Traj_i(k) | Judge(k); % 问题点

            % 最后一段检测
            if k == PointNum
                P1 = UAV.G(i, :);
                [across, ~] = CheckThreat(P2, P1, M);
                Judge(k+1) = across;
                
                dl = norm(P1 - P2);
                l = l + dl;
                t = l / V;
                L_i(k+1) = l;
                Time_i(k+1) = t;

                ZZ = P1-P2;
                phi1 = atan2(ZZ(2), ZZ(1));
                d_phi = phi1 - phi0;
                if dim>2      
                    theta1 = atan(ZZ(3) / sqrt(ZZ(1)^2 + ZZ(2)^2));
                    d_theta = theta1 - theta0;
                else
                    d_theta = 0;
                end

                if abs(d_phi) > UAV.limt.phi(i, 2)  ||  abs(d_theta) > UAV.limt.theta(i, 2)
                    Angle_i(k+1) = true;
                else
                    Angle_i(k+1) = false;
                end

                if dl < UAV.limt.L(i, 1)
                    Traj_i(k+1) = true;
                else
                    Traj_i(k+1) = false;
                end

                ProbPoint_i(k+1) = Angle_i(k+1) | Traj_i(k+1) | Judge(k+1);

            end
      end   
      Threat(i) = {Judge};                % 检测结果（长度比点的数目多一）
      Angle(i) = {Angle_i};               % 转角
      MiniTraj(i) = {Traj_i};              % 航迹片段
      ProbPoint(i) = {ProbPoint_i}; % 问题点

      L(i) = {L_i};                 % 路径长度（累加）
      Time(i) = {Time_i};    % 时间（累加）
      L_mt = L_mt + l;           % 总长度
      totalTime(i) = t;            % 时间
      totalL(i) = l;                   % 长度
end

% 多无人机碰撞检测
d_safe = UAV.ds; % 安全距离
CollideTimes = 0; % 碰撞次数
for i = 2 : UAV.num
    PointNum_i = UAV.PointNum(i);
    for k = 1 : PointNum_i
        P1 = Track.P{i}(:, k)';  % K时刻 i 无人机位置
        t_i = Time{i, 1}(k);                  % K时刻 i 无人机时间
        for j = 1 : i-1
            PointNum_j = UAV.PointNum(j);
            flag = false; % 
            % 搜索同一时刻的点
            for kj = 1 : PointNum_j
                if kj ==1
                    t_j_l = 0;
                    P_l =  UAV.S(j, :); 
                else
                    t_j_l = Time{j}(kj-1);
                    P_l =  Track.P{j}(:, kj-1)'; 
                end
                t_j_r = Time{j}(kj);
                P_r =  Track.P{j}(:, kj)'; 

                if t_i <= t_j_r  &&  t_i >= t_j_l
                    flag = true;
                    P2 =  P_l + (t_i - t_j_l) / (t_j_r - t_j_l) * (P_r - P_l);% K时刻 J 无人机位置
                    % K时刻 j 无人机位置
                end
            end
            % ———————

            if flag  % 查找到P2位置，进行碰撞检测
                collide = CheckCollide(P1, P2, d_safe);
            else     % 没有P2位置，不会碰
                collide = false;
            end
            if collide
                CollideTimes = CollideTimes + 1;
            end
        end
    end
end

% 生成检测报告
report.L_mt = L_mt;                       %总行程之和
report.Threat = Threat;                 %受威胁的航迹点位置
report.AngleProb = Angle;            % 转角不满足的点
report.TrajProb = MiniTraj;           % 不满足最小航迹间隔的点
report.ProbPoint = ProbPoint;     % 有问题的点
report.L = totalL;                            %飞行距离
report.time = totalTime;                %飞行时间
report.col_times = CollideTimes;  %碰撞次数
end



%% 穿越威胁区检测
function [across, across_num] = CheckThreat(P1, P2, M)
    % 威胁区（球或圆区域，不适合圆柱区域）
    O = M(:,1:end-1);  % 圆心
    R = M(:, end);        % 半径 
    
    % 检测线段是否穿过某个障碍区
    total = 0;
    for i = 1 : size(O, 1)
         a = norm(P1 - P2);
         b = norm(P2 - O(i, :));
         c = norm(P1 - O(i, :));
         % P1点是否在圆内
         if c < R(i)         
             isHit = true;   
         % P2点是否在圆内
         elseif b < R(i)  
             isHit = true;   
         % P1 P2都不在圆内
         else               
             dim = size(O, 2);  % P1： 1*dim 维
             if dim < 3
                 % 平面情况   
                 A = P1(2) - P2(2);
                 B = P2(1) - P1(1);
                 C = P1(1)*P2(2) - P2(1)*P1(2);
                 x = O(i, 1);
                 y = O(i, 2);
                 d = abs(A*x + B*y + C) / sqrt(A^2 + B^2); 
             else
                 % 空间情况
                 PP = P2 - P1;
                 PO = O(i, :) - P1;
                 d = norm(cross(PP, PO)) / a;
             end
             % 距离判据
             if d >= R(i)
                 isHit = false;  % 不相交
             % 角度判据(距离满足条件时两个角都为锐角时相交)
             elseif d > 0
                 cosP1 = a^2 + c^2 - b^2 / (2*c*a);
                 cosP2 = a^2 + b^2 - c^2 / (2*b*a);
                 if cosP1 > 0  &&  cosP2 > 0
                     isHit = true;
                 else
                     isHit = false;
                 end
             % 两点在外，距离为0情况（共线情况）
             else
                 if a > b  &&  a > c
                    isHit = true;
                 else
                    isHit = false;
                 end
             end
         end

         if isHit
            total = total + 1; %总共碰撞几次
         end
    end

    if total > 0
        across = true;
    else
        across = false;          % 是否穿越禁区
    end
    across_num = total;     % 穿过禁区的个数
end



%% 碰撞检测
function [collide] = CheckCollide(P1, P2, d_safe)
    if norm(P1 - P2) >= d_safe
        collide = false;
    else
        collide = true;
    end
end
