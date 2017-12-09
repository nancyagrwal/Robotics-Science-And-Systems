function out = CheckForCollision (rob,obstacle)
face_size1 = size (rob.faces, 1);
face_size2 = size (obstacle.faces, 1);
for i = 1:face_size1
    obj1 = rob.vertices(rob.faces(i,:), :);
    for j = 1:face_size2
        obj2 = obstacle.vertices(obstacle.faces(j,:), :);
        if (intersection_check(obj1,obj2))
            out = true;
            return;
        end
    end
end
out = false;
end

function intersection_flag = intersection_check(obj1, obj2)
% intersection_check : returns true if the faces overlap and false otherwise
    intersection_flag = true;
    for k = 1:2 
	
        bincoeff = nchoosek(1:size(obj1, 1),2); 
        for i = 1:size(bincoeff,1)
            objinter = obj1(setdiff((1:size(obj1,1)), bincoeff(i,:)),:);
            side_one = checkLineIntersection(obj1(bincoeff(i,1),:), obj1(bincoeff(i,2), :), objinter);
            side_two = zeros(size(obj2,1),1);
            for j = 1:size(obj2, 1)
                objintr2 = obj1(bincoeff(i,2),:);
                side_two(j) = checkLineIntersection(obj1(bincoeff(i,1),:),objintr2, obj2(j,:));
            end
            if length(unique(side_two))==1 && sum(side_two ~= side_one) == size(obj2, 1)
                intersection_flag = false;
                return;
            end
        end
        
        obj = obj1;
        obj1 = obj2;
        obj2 = obj;
    end      
end

