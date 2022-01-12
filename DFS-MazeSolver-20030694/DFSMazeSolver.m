%%%
% DFSMazeSolver
% Invoke the problem maze
% Read in starting point, end point, obstacles
% Depth first search implementation
% display
%%%

%Invoke the problem
%Initiallization
function [] = DFSMazeSolver(maze)

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
    end
end
for i=1:length %Get the destination point location from 'maze' through loop
    if (maze(1,i)==4)
        yTarget = i;
        xTarget = 1;
    end
end
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
    
    % If no possible direction to go at the current point
    % then trace back
    if (exp_count == 0)
        P = point(xNode,yNode);
        maze = setMazePosition(maze, P, 6);
        for i=1:QUEUE_COUNT
            if (QUEUE(i, 2)==xNode && QUEUE(i, 3)==yNode)
                xNode = QUEUE(i, 4);
                yNode = QUEUE(i, 5);
                % adjust the current path cost
                path_cost = QUEUE(i, 6) - 1;
                % move the node to OBSTACLE
                OBST_COUNT = OBST_COUNT + 1;
                OBSTACLE(OBST_COUNT, 1) = xNode;
                OBSTACLE(OBST_COUNT, 2) = yNode;
                % QUEUE(index_min_node, 1) = 0;
                break;
            end
        end       
    else

          % Update QUEUE with child nodes; exp: [X val, Y val, g(n), h(n), f(n)]
          P = point(xNode,yNode);
          maze = setMazePosition(maze, P, 6);
          % QUEUE_COUNT = QUEUE_COUNT + 1;
          % QUEUE(QUEUE_COUNT, :) = insert(exp(1, 1), exp(1, 2), xNode, yNode, exp(1, 3), exp(1, 4), exp(1, 5));
          for i=1:exp_count
                  QUEUE_COUNT = QUEUE_COUNT + 1;
                  QUEUE(QUEUE_COUNT, :) = insert(exp(i, 1), exp(i, 2), xNode, yNode, exp(i, 3), exp(i, 4), exp(i, 5));
          end
          QUEUE(QUEUE_COUNT, 1) = 0;
          xNode = exp(exp_count, 1);
          yNode = exp(exp_count, 2);
          path_cost = QUEUE(QUEUE_COUNT, 6); % cost g(n)
          % move the node to OBSTACLE
          OBST_COUNT = OBST_COUNT + 1;
          OBSTACLE(OBST_COUNT, 1) = xNode;
          OBSTACLE(OBST_COUNT, 2) = yNode;
    end        
end

% update the expanded node (identifying by the flag)
% 1 for not expanded, 0 for expanded
 for i=1:QUEUE_COUNT
     if (maze(QUEUE(i, 2), QUEUE(i, 3)) == 6)
         QUEUE(i, 1) = 0;
     end
 end
result();
save D QUEUE;