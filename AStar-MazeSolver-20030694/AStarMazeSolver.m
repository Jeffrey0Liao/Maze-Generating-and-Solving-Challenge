%%% 
% AstarMazeSolver
% Read in data structure 'maze'
% Read in starting point, end point, obstacles
% A* searching
% Display
%%%

%Invoke the problem
%Initiallization
function [] = AStarMazeSolver(maze)

clear size;
length = size(maze, 1);
MAX_X = length;
MAX_Y = length;

%% add the starting node as the first node (root node) in QUEUE
% QUEUE: [0/1, X val, Y val, Parent X val, Parent Y val, g(n),h(n), f(n)]
for i=1:length %Get the start point location from 'maze' through loop
    if (maze(length,i)==3)
        yStart = i;
        xStart = length;
        yNode = yStart;
        xNode = xStart;
    end;
end;
for i=1:length %Get the destination point location from 'maze' through loop
    if (maze(1,i)==4)
        yTarget = i;
        xTarget = 1;
    end;
end;
% OBSTACLE: [X val, Y val]
% Obtain obstacles from the 'maze'
OBSTACLE = [];
k = 1;
for i = 1 : length
    for j = 1 : length
        if(maze(i, j) == 0 || maze(i, j)==8)
            OBSTACLE(k, 1) = i;
            OBSTACLE(k, 2) = j;
            k = k + 1;
        end
    end
end
OBST_COUNT = size(OBSTACLE, 1);
OBST_COUNT = OBST_COUNT + 1;
OBSTACLE(OBST_COUNT, :) = [xStart, yStart];

QUEUE = [];
QUEUE_COUNT = 1;
NoPath = 1; % assume there exists a path
path_cost = 0; % cost g(n): start node to the current node n
goal_distance = distance(xNode, yNode, xTarget, yTarget); % cost h(n): heuristic cost of n
QUEUE(QUEUE_COUNT, :) = insert(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);


%% Start the search
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
    
    % expand the current node to obtain child nodes
    exp = expand(xNode, yNode, path_cost, xTarget, yTarget, OBSTACLE, MAX_X, MAX_Y, maze);
    exp_count  = size(exp, 1);
    % Update QUEUE with child nodes; exp: [X val, Y val, g(n), h(n), f(n)]
    for i = 1 : exp_count
        flag = 0;
        for j = 1 : QUEUE_COUNT
            if(exp(i, 1) == QUEUE(j, 2) && exp(i, 2) == QUEUE(j, 3))
                QUEUE(j, 8) = min(QUEUE(j, 8), exp(i, 5));
                if QUEUE(j, 8) == exp(i, 5)
                    % update parents, g(n) and h(n)
                    QUEUE(j, 4) = xNode;
                    QUEUE(j, 5) = yNode;
                    QUEUE(j, 6) = exp(i, 3);
                    QUEUE(j, 7) = exp(i, 4);
                end; % end of minimum f(n) check
                flag = 1;
            end;
        end;
        if flag == 0
            QUEUE_COUNT = QUEUE_COUNT + 1;
            QUEUE(QUEUE_COUNT, :) = insert(exp(i, 1), exp(i, 2), xNode, yNode, exp(i, 3), exp(i, 4), exp(i, 5));
        end; % end of insert new element into QUEUE
    end;
 
    % A*: find the node in QUEUE with the smallest f(n), returned by min_fn
    index_min_node = min_fn(QUEUE, QUEUE_COUNT);
    if (index_min_node ~= -1)
        % set current node (xNode, yNode) to the node with minimum f(n)
        xNode = QUEUE(index_min_node, 2);
        yNode = QUEUE(index_min_node, 3);
        path_cost = QUEUE(index_min_node, 6); % cost g(n)
        % move the node to OBSTACLE
        OBST_COUNT = OBST_COUNT + 1;
        OBSTACLE(OBST_COUNT, 1) = xNode;
        OBSTACLE(OBST_COUNT, 2) = yNode;
        QUEUE(index_min_node, 1) = 0;
        
        % Update all the nodes searched to be 6
        % Dynamically printed to be red (1, 0, 0)
        pos = point(xNode, yNode);
        maze = setMazePosition(maze, pos, 6);
    else
        NoPath = 0; % there is no path!
    end;
end;
result();
save A QUEUE;