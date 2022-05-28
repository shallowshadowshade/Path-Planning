function n_index = node_index(OPEN,xval,yval,OPEN_COUNT)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )
        i=i+1;
        if(i>OPEN_COUNT)
            break
        end
    end
    n_index=i;
end