function [L,S,d] = d2poly(point,poly)
% point = current position
% poly = current position of polynomial

d = Inf;
nside = size(poly,1);                                                       % # vertices of obstacle
for i=1:nside                                                               % FOR EACH VERTEX
    x1 = poly(i,1);                                                         % x coord of i-th vertex
    y1 = poly(i,2);                                                         % y coord of i-th vertex
    x2 = poly(mod(i,nside)+1,1);                                            % x coord of next vertex (loops around to 1st vertex when at the last one)
    y2 = poly(mod(i,nside)+1,2);
    
    trid = [];
    trid(1) = norm([x1-x2,y1-y2]);                                          % distance between i-th and i+1-th vertex
    trid(2) = norm([x1-point(1),y1-point(2)]);                              % distance between i-th vertex and current location
    trid(3) = norm([x2-point(1),y2-point(2)]);                              % distance between i+1-th vertex and current location
    
    % default case
    Lr = [y1-y2,x2-x1];                                                     % [-dy, dx]  
    Sr = -x1*y2+x2*y1;
    vd = abs(Lr(1)*point(1)+Lr(2)*point(2)-Sr)/trid(1);                     % |-dy*x0 + dx*y0 - Sr|/trid(1)
                                    
    % case 1
    if trid(2)^2 > trid(1)^2+trid(3)^2                                      % theta > 90deg
        vd = trid(3);                                                       % i+1-th vertex ix closer
        Lr = [point(1)-x2,point(2)-y2];                                     % [x0 - x-coord of i+1-th vertex, y0 - y-coord of i+1-th vertex]
        Sr = [point(1)-x2,point(2)-y2]*[x2,y2]';                            % deltaX * x2 + deltaY * y2
    end
    
    % case 2
    if trid(3)^2 > trid(1)^2+trid(2)^2
        vd = trid(2);
        Lr = [point(1)-x1,point(2)-y1];
        Sr = [point(1)-x1,point(2)-y1]*[x1,y1]';
    end
    
    if vd < d                                                               % if the closer of the 2 vertices is closer than any previous ones
        d = vd;                                                             % new distance = distance to vertex
        L = Lr;                                                             % new x, y distance to vertex
        S = Sr;                                                             
        ii = i;                                                             % index of closest vertex
    end
end

% determine sign on L, S, d

nL = norm(L);
L = L./nL;          % normalized of L
S = S/nL;           % S normalized with L

if L*poly(mod(ii+1,nside)+1,:)'<S    % vertex 2 positions later
    L = -L;         % 
    S = -S;
end

if d == 0
    return
end

area = 0; polyarea = 0;
for i=1:nside
    area = area + triArea(point,poly(i,:),poly(mod(i,nside)+1,:));
end
for i=2:nside-1
    polyarea = polyarea + triArea(poly(1,:),poly(i,:),poly(mod(i,nside)+1,:));
end
if norm(polyarea-area) < 0.01
    d = -d;
end

end

