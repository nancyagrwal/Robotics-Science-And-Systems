function out = AddNewNode(random_config, roadmap, CalculateDistance, LocalPlanner, k)
distances = CalculateDistance(random_config, roadmap.random_samples);
[distances_sorted, index] = sort(distances);
len = length(distances);
edge_set = zeros(k,2);
costs_edge = zeros(k,1);
edge_count = 0;

for i = 1:min(k,len)
    j = index(i);
    if (LocalPlanner(random_config, roadmap.random_samples(:,j)))
        edge_count = edge_count + 1;
        edge_set(edge_count,:) = [len+1, j];
        costs_edge(edge_count) = distances_sorted(i);
    end
end

out.random_samples = [roadmap.random_samples, random_config(:)];
out.edge_set = [roadmap.edge_set; edge_set(1:edge_count, :)];
out.costs_edge = [roadmap.costs_edge; costs_edge(1:edge_count)];