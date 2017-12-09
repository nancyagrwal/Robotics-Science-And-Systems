function rob3 = appendStructures (struct1,struct2)
	sizeOfStruct = size(struct1.vertices,1);
	rob3.vertices = [struct1.vertices; struct2.vertices];
	rob3.faces = [struct1.faces; struct2.faces + sizeOfStruct];
end
