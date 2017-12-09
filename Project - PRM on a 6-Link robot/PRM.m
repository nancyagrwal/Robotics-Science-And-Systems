function roadmap = PRM (RandomSPaceConfiguration, CalculateDistance, LocalPlanner, sample_count, k)
% PRM - ProbablisticRoadMap : Algorithm for Sampling:
%1. nodes â†? sample N nodes random configuration
%2. for all nodes
%3. find knearest nearest neighbors
%4. if collision check and Î³ â‰¤ Î³max then roadmap â†? edge
%5. end
%
% Inputs Functions and outputs:
%
%   RandomSPaceConfiguration : returns a random sample in configuration space 
%   CalculateDistance : A function that computes the distance between 2 random configurations obtained.
%   LocalPlanner :  returns True if there exists a collsion free line between two points.

% Output :roadmap 
pts = zeros(sample_count+2,2);
random_config = RandomSPaceConfiguration();
% is a 6*1 vector.

random_samples = repmat(random_config(:), 1, sample_count);
% random_samples is (6*number of samples) matrix  -- an Array of random samples.

edge_set = zeros(k*sample_count, 2);
% edge_set - each column pair determines if sample i is connected to sample j

costs_edge = zeros(sample_count*k, 1);
% corresponding edge lengths for the above edge pairs.

edge_count = 0;

for i = 2:sample_count
    random_config = RandomSPaceConfiguration();
    random_samples(:,i) = random_config(:);
    distances = CalculateDistance(random_config, random_samples(:,1:(i-1)));
    [sorted_dist, index] = sort(distances,'ascend');

% Resampling: Once a node is selected to be expanded:
% 1. Pick a random motion direction in c-space and move in this direction
% until an obstacle is hit.
% 2. When a collision occurs, choose a new random direction and proceed for
% some distance.
% 3. Add the resulting nodes and edge_set to the tree. Re-run tree connection
% step.

    for j = 1: min(k,i-1)
        if (LocalPlanner(random_config, random_samples(:,index(j))))
            rob = CreatePumaRobot(random_config);
%             for plotting random samples            
%             p = patch(rob);
%             p.FaceColor = 'red';
%             p.EdgeColor = 'black';
            f= rob.vertices;
            x= f(1,1);
            y = f(1,2);
            pt = [x,y];
            pts(i,:) = pt;
            edge_count = edge_count + 1;
            edge_set(edge_count,:) = [i,index(j)];
            costs_edge(edge_count) = sorted_dist(j);
        end
    end
    
    % update edge length and edge count if there is any edge possible 
    % between the k closest samples
    fprintf (1, 'sample_count = %d, edge_count = %d\n', i, edge_count);
   
end
roadmap.pts = pts;
roadmap.random_samples = random_samples;
roadmap.edge_set = edge_set(1:edge_count, :);
roadmap.costs_edge = costs_edge(1:edge_count);