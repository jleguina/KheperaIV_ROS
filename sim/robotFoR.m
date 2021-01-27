function e = robotFoR(input)
    
    theta = input(3);
    A = [cosd(theta), sind(theta), 0;
        -sind(theta), cosd(theta), 0;
        0, 0, 1];
    
    e = A*input;
    ends