function [WolfPops, Tracks] = BoundAdjust(WolfPops, ProbPoints, UAV)
%BOUNDADJUST 约束处理
Tracks = Pops2Tracks(WolfPops, UAV); 

% ①  不满足约束："删除"航迹点，即取前后平均值
dim = UAV.PointDim;                                    % 维度
for agent = 1 : size(WolfPops.Pos, 1)
    Track = Tracks{agent};                             % 轨迹 (struct结构)
    ProbPoint = ProbPoints{agent};                     % 问题点
    Position = [];                                     % 新狼群编码
    for i =1:UAV.num
          PointNum = UAV.PointNum(i);
          % 删除一条航迹上的问题点
          for k = 1 : PointNum
                flag = ProbPoint{i}(k);
                if flag == 1
                    if k == 1
                        P1 = UAV.S(i, :)' ;
                    else
                        P1 = Track.P{i}(:, k-1);
                    end
                    if k == PointNum
                        P2 = UAV.G(i, :)' ;
                    else
                        P2 = Track.P{i}(:, k+1);
                    end
                    Track.P{i}(:, k) = (P1+P2) / 2;  % 删除航迹点
                end
          end
    
          % 转换为狼群编码形式
          p = Track.P{i} ;
          p = reshape(p, 1, dim*PointNum);
          Position = [Position, p];
    end
    V = Track.V';
    Position = [Position, V];
    
    % 新的位置信息
    WolfPops.Pos(agent, :) = Position;
end

% ②  边界处理：越界取边界值
WolfPops.Pos = BoundClamp(WolfPops.Pos, WolfPops.lb, WolfPops.ub);

% 生成新航迹
Tracks = Pops2Tracks(WolfPops, UAV); 

end



% 边界裁剪
function x = BoundClamp(x, lb, ub)
    Flag4ub = x > ub;
    Flag4lb = x < lb;
    x = x .* ( ~(Flag4ub + Flag4lb) ) + ub .* Flag4ub + lb .* Flag4lb;
end
