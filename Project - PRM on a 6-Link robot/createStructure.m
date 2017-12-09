function rob = createStructure(p1, p2, p3, p4)
	rob.vertices = [p1 p3; p2 p3; p2 p4; p1 p4];
 	rob.faces = [1 2 3; 1 3 4];
end