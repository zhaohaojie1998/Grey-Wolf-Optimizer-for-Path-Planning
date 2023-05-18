function Tracks = Pops2Tracks(WolfPops, UAV)
%POPS2TRACKS 将狼群转换为航迹

SearchAgents = size(WolfPops.Pos, 1);    % 种群数量
UAVnum = UAV.num;                        % 无人机个数
dim = UAV.PointDim;                      % 仿真维度
v = WolfPops.Pos(:, end-UAVnum+1:end);   % 协同无人机速度
P = WolfPops.Pos(:, 1:end-UAVnum);       % 协同无人机航迹 xy

Tracks = cell(SearchAgents, 1);
for agent = 1 : SearchAgents
    a.V = v(agent, :)';
    P_a = P(agent, :);
    a.P = cell(UAVnum, 1);
    for i =1:UAVnum
        PointNum = UAV.PointNum(i);   
        % 三维仿真这里得改 PointNum*dim
        P_ai = P_a(1 : PointNum*dim);
        P_ai = reshape(P_ai, dim, PointNum);
        P_a = P_a(PointNum*dim+1 : end);
        a.P(i) =  {P_ai};
    end
    Tracks(agent) = {a};
end

end

