function out = rotateStruct(rob,theta,dist)
	out.faces = rob.faces;
	cosine = cosd(theta); sine = sind(theta);
    mat = [cosine sine; -sine cosine];
	out.vertices = bsxfun(@plus, rob.vertices*mat,dist);
end
