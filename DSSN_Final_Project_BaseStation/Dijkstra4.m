function [path, hops, path_length, Paths] = Dijkstra4(Nodes, NT, Dest)
    max_Iterations = 150;
    Src = 8;
    
    Known_Nodes = [];
    Distances = inf*ones(length(NT),1);
    Distances(Src) = 0;
    Total_Hops = zeros(size(Distances));
    
    Paths = cell(length(Nodes),1);
    Paths{Src} = [Src];
    Remaining_Nodes = setxor(Nodes,Known_Nodes);
    i = 0;
    while i < max_Iterations && 0 < length(Remaining_Nodes)
        i = i+1;
        
        [~,index] = min(Distances(Remaining_Nodes)); % minimum of unknown nodes
        display((Remaining_Nodes))
        display(Distances(Remaining_Nodes))
        display(Remaining_Nodes(index))
        display(Paths)
        Working_Node = Remaining_Nodes(index); 
        Known_Nodes = [Known_Nodes;Working_Node];
        Remaining_Nodes = setxor(Nodes,Known_Nodes);
        
        % Re-lable distances for nodes in range
        for j = Nodes
            Dist = NT(Working_Node,j);
            if ~ismember(j,Known_Nodes)
                if Dist < inf 
                   if Distances(j)> Distances(Working_Node) + Dist
                       Distances(j) = Distances(Working_Node) + Dist;
                       Total_Hops(j) = Total_Hops(Working_Node) + 1;
                       Paths{j} = [Paths{Working_Node};j];
                   end
                end
            end
        end
    end
    hops = Total_Hops(Dest);
    path_length = Distances(Dest);
    path = Paths{Dest};
end