function rob = CreatePumaRobot(config)
   	link = createStructure(0,5,-0.5,0.5);
	rob = link;
  	for i = 1:5
		rob = appendStructures(link,rotateStruct(rob, config(i),[5 0]));
    end
 	rob = rotateStruct(rob,config(end),[0 0]);
 





	
    
