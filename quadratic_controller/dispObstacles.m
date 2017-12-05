function [x, y, z] = dispObstacles(obs, r, height, x0, y0, z0)
            
    switch obs
       case 'sphere'
            [x,y,z] = sphere(50);
            x = x*r + x0;
            y = y*r + y0;
            z = z*r + z0;

       case 'cylinder' 
           [x,y,z] = cylinder(r,50);     
           x = x + x0;
           y = y + y0;
           z = z*height + z0;
    end

   hold on
   lightGrey = 0.8*[1 1 1]; % It looks better if the lines are lighter
   surface(x,y,z,'FaceColor', 'none','EdgeColor',lightGrey)
end