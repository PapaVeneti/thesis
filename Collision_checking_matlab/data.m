x = [-0.47,-0.74, -1.24 ];
y = [-1.32, -0.62, 0.21] ;

[r,xc] = fit_circle_through_3_points([x',y'])

a = viscircles(xc',r,Color='b');