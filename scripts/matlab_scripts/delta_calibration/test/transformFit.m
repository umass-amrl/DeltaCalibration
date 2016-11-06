function [X, Y, R, T, Yf, Err] = transformFit(fileX, fileY)


    X = dlmread(fileX, '\t',  1, 3)
    Y = dlmread(fileY, '\t', 1,3)
    X = X(1:37, 1:3)
    Y = Y(12:48, 1:3)
    
    % mean correct

    Xm = mean(X,1); 
    X1 = X - ones(size(X,1),1)*Xm;
    Ym = mean(Y,1); 
    Y1 = Y - ones(size(Y,1),1)*Ym;
    
    XtY = (X1')*Y1;
    [U,S,V] = svd(XtY);
    R = U*(V');

    % solve for the translation vector

    T = Ym - Xm*R;

    % calculate fit points

    Yf = X*R + ones(size(X,1),1)*T;

    % calculate the error
    
    dY = Y - Yf
    Err = norm(dY,'fro'); % must use Frobenius norm