
X = [xx(1).Position(1),xx(2).Position(1)];
Y = [xx(1).Position(2),xx(2).Position(2)];
coeff = polyfit(X,Y,1);

disp('Rate is: ')
disp(coeff(1))