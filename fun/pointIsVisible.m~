function isVisible = pointIsVisible( point, Cam, FOV )

% transform to local coordinates
point = [Cam; 0 0 0 1]^-1 * [point; 1];
point = point(1:3);

% compute cos(angle_zx)
cosZX = point(3) / sqrt( point(1)^2 + point(3)^2  );

% compute cos(angle_zy)
cosZY = point(3) / sqrt( point(2)^2 + point(3)^2  );

% check if point is inside the camera FOV
if cosZX > cos(.5*FOV(1)*pi/180) && ...
   cosZY > cos(.5*FOV(2)*pi/180) 
    isVisible = 1;
else
    isVisible = 0;
end    
    
end