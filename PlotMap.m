function figH = PlotMap(fig, mapSize, map, gate, mansion, cameras, varargin)
%PLOTMAP Plot a map, the costs for each cell and the control action in
%   each cell.
%   
%	figH = PlotMaze(fig, mapSize, map, gate, mansion, cameras, varargin)
%
%	Input arguments:
%
%       fig:
%       	Figure handle of plot.
%
%       mapSize:
%        	A (2 x 1)-matrix with the first element being the width N of
%           the maze, and the second element being the height M of the
%           maze.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (2 x 1)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 2)-matrix indicating the positions of the cameras.
%
%       varargin (optional):
%       	Input argument list:
%           	1: 	A (K x 2)-matrix 'stateSpace', where each row
%                   represents an element of the state space.
%            	2:  A (K x 1 )-matrix 'J' containing the optimal cost-to-go
%               	for each element of the state space.
%            	3:  A (K x 1 )-matrix containing the index of the optimal
%               	control input for each element of the state space.
%
%	Output arguments:
%
%   	figH:
%         	Figure handle to generated plot.

figH = figure(fig);
clf;

%% plot parameters
% trees
treeColor = [0 0 0]./255;

% water
waterColor = [112 181 255]./255;

% gate
gateColor = [255 230 0]./255;

% mansion
mansionColor = [156 156 156]./255;

% cameras
cameraColor = [242 61 0]./255;

%% clear mansion area in map
row = mansion(:,2);
column = mansion(:,1);
for i = 1:length(row)
    map(row(i), column(i)) = 0;
end

%% plot trees
[row, column] = find(map>0);
for i = 1:length(row)
        [dummyIdx1] = findsubmat(cameras(:,1:2),[column(i), row(i)]);
        if (isempty(dummyIdx1))
            xCorner = [column(i)-1 column(i) column(i) column(i)-1];
            yCorner = [row(i)-1 row(i)-1 row(i) row(i)];
            fill(xCorner, yCorner, treeColor, 'EdgeColor', 'none');
            hold on;
        end
end

%% plot water
[row, column] = find(map<0);
for i = 1:length(row)
        xCorner = [column(i)-1 column(i) column(i) column(i)-1];
        yCorner = [row(i)-1 row(i)-1 row(i) row(i)];
        fill(xCorner, yCorner, waterColor, 'EdgeColor', 'none');
        hold on;
end

%% plot gate
if (nargin < 9)
xCorner = [gate(1)-1 gate(1) gate(1) gate(1)-1];
yCorner = [gate(2)-1 gate(2)-1 gate(2) gate(2)];
fill(xCorner, yCorner, gateColor, 'EdgeColor', 'none');
hold on;
end

%% plot mansion
row = mansion(:,2);
column = mansion(:,1);
for i = 1:length(row)
    xCorner = [column(i)-1 column(i) column(i) column(i)-1];
    yCorner = [row(i)-1 row(i)-1 row(i) row(i)];

    if ((row(i) == min(row) || row(i) == max(row)) && (column(i) == min(column) || column(i) == max(column)))
        fill(xCorner, yCorner, cameraColor, 'EdgeColor', 'none');
        hold on;
    else
        fill(xCorner, yCorner, mansionColor, 'EdgeColor', 'none');
        hold on;
    end
end

%% plot cameras
row = cameras(:,2);
column = cameras(:,1);
for i = 1:length(row)-4
    xCorner = [column(i)-1 column(i) column(i) column(i)-1];
    yCorner = [row(i)-1 row(i)-1 row(i) row(i)];
    fill(xCorner, yCorner, cameraColor, 'EdgeColor', 'none');
    hold on;
end


%% color cells
cMap = jet(101);
cScale = (0:0.01:1)';

if nargin > 8
    x = varargin{ 1 };
    J = varargin{ 2 };
    
    maxJ = max(J);
    
    idxgate = findsubmat(x, gate);
    
    for i = 1 : length( J );
        xCorner = [x(i,1)-1 x(i,1) x(i,1) x(i,1)-1];
        yCorner = [x(i,2)-1 x(i,2)-1 x(i,2) x(i,2)];
        cHSV = rgb2hsv(interp1(cScale, cMap, J(i)/maxJ));
        cHSV(2) = 0.5*cHSV(2);
        if (i == idxgate)
            fill(xCorner, yCorner, hsv2rgb(cHSV), 'EdgeColor', 'k', 'LineWidth', 1);
            plot([x(i,1)-1 x(i,1)],[x(i,2)-1 x(i,2)],'k');
            plot([x(i,1)-1 x(i,1)],[x(i,2) x(i,2)-1],'k');
        else
            fill(xCorner, yCorner, hsv2rgb(cHSV), 'EdgeColor', 'none');
        end
        hold on;
    end
end


%% plot arrows and expected costs
if nargin > 8
    x = varargin{ 1 };
    J = varargin{ 2 };
    u_opt_ind = varargin{ 3 };
    
    for i = 1 : length( J );
        x_i = x( i, : );
        shouldPlotArrow = true;
        shouldPlotCost = true;
        shouldPlotCostRed = false;
        
        if shouldPlotArrow
            center = [ x_i( 1 ) - 0.5, x_i( 2 ) - 0.5 ];
            if (u_opt_ind(i) == 1)
                u_i = [0, 1];
            elseif (u_opt_ind(i) == 2)
                u_i = [-1, 0];
            elseif (u_opt_ind(i) == 3)
                u_i = [0, -1];
            elseif (u_opt_ind(i) == 4)
                u_i = [1, 0];
            else
                u_i = [];
            end
            if (~isempty(u_i))
                startPt = center + 0.2 * u_i;
                endPt = center + 0.4 * u_i;
                arrow( startPt, endPt );
            else
                shouldPlotCostRed = true;
            end
        end
        
        if shouldPlotCost
            if shouldPlotCostRed
                text( x_i( 1 ) - 0.6, x_i( 2 ) - 0.5, num2str( J( i ), 3 ), 'FontSize', 8 , 'Color', [1 0 0]);
            else
                text( x_i( 1 ) - 0.6, x_i( 2 ) - 0.5, num2str( J( i ), 3 ), 'FontSize', 8 );
            end
        end
       
    end
end


%% plot (outer) boundaries
plot([0 mapSize(1)], [0 0], 'k', 'LineWidth', 3);
hold on;
plot([mapSize(1) mapSize(1)], [0 mapSize(2)], 'k', 'LineWidth', 3);
plot([mapSize(1) 0], [mapSize(2) mapSize(2)], 'k', 'LineWidth', 3);
plot([0 0], [mapSize(2) 0], 'k', 'LineWidth', 3);

%% label plot and correct aspect ratio
title(strcat('map (width=', num2str(mapSize(1)), ', height=', num2str(mapSize(2)), ')'));
daspect([1 1 1]);
margin = 2;
axis([-margin mapSize(1)+margin -margin mapSize(2)+margin]);

findfigs;
hold off;

end

%% plot 2D arrow
function arrow( startPt, endPt )
% compute orientation if arrow head
alpha = atan2( endPt( 2 ) - startPt( 2 ), endPt( 1 ) - startPt( 1 ) );
R = [ cos( alpha ), -sin( alpha ); sin( alpha ), cos( alpha ) ];

% define lines that draw the arrow head
arrowHead = [ 0, 0;
              -0.1, 0.1;
              0, 0;
              -0.1, -0.1 ];
for i = 1 : size( arrowHead, 1 );
    arrowHead( i, : ) = ( R * arrowHead( i, : )' )';
    arrowHead( i, : ) = arrowHead( i, : ) + endPt;
end

% define line that draws the arrow
arrowLines = [ startPt( 1 ), startPt( 2 );
               endPt( 1 ), endPt( 2 ) ];

% plot
color = [ 0, 0, 0 ] + 0.5;
plot( arrowLines( :, 1 ), arrowLines( :, 2 ), 'color', color, 'lineWidth', 1.5 );
plot( arrowHead( :, 1 ), arrowHead( :, 2 ), 'color', color, 'lineWidth', 1.5 );
end

%% find submatrix
function [idx,idx2] = findsubmat(A,B)
% FINDSUBMAT find one matrix (a submatrix) inside another.
% IDX = FINDSUBMAT(A,B) looks for and returns the linear index of the
% location of matrix B within matrix A.  The index IDX corresponds to the
% location of the first element of matrix B within matrix A.
% [R,C] = FINDSUBMAT(A,B) returns the row and column instead.
%
% EXAMPLES:
%
%         A = magic(12);
%         B = [81 63;52 94];
%         [r,c]= findsubmat(A,B)
%         % Now to verify:
%         A(r:(r+size(B,1)-1),c:(c+size(B,2)-1))==B
%
%         A = [Inf NaN -Inf 3 4; 2 3 NaN 6 7; 5 6 3 1 6];
%         B = [Inf NaN -Inf;2 3 NaN];
%         findsubmat(single(A),single(B))
%
% Note:  The interested or concerned user who wants to know how NaNs are 
% handled should see the extensive comment block in the code.  
%
% See also find, findstr, strfind
%
% Author:   Matt Fig with improvements by others (See comments)
% Contact:  popkenai@yahoo.com
% Date:     3/31/2009
% Updated:  5/5/2209 to allow for NaNs in B, as per Urs S. suggestion. 
%                    Also to return [] if B is empty.
%           6/12/2009 Introduce another algorithm for larger B matrices.

if nargin < 2
    error('Two inputs are required.  See help.')
end

[rA,cA] = size(A);  % Get the sizes of the inputs. First the larger matrix.
[rB,cB] = size(B);  % The smaller matrix.
tflag = false; % In case we need to transpose.

if ndims(A)~=2 || ndims(B)~=2 || rA<rB || cA<cB
    error('Only 2D arrays are allowed.  B must be smaller than A.')
elseif isempty(B)
    idx = [];  % This should be an obvious situation.
    idx2 = [];
    return;
elseif rA==rB && cA==cB  % User is wasting time, annoy him with a disp().
    disp(['FINDSUBMAT is not recommended for equality determination, ',...
          'instead use:  isequal(A,B)']);
    idx = [];
    idx2 = [];
    
    if all(A(:)==B(:))
        idx = 1;
        idx2 = 1;
    end
    return;
elseif rB==1 && cB==1  % User is wasting time, annoy him with a disp().
    disp(['FINDSUBMAT is not recommended for finding scalars, ',...
          'instead use:  find(A==B)']);
    if nargout==2
        [idx,idx2] = find(A==B);
    else
        idx = find(A==B);
    end
    return;
end

if cB > ceil(1.5*rB)
    A = A.';  % In this case it may be faster to transpose.
    B = B.';  % The 1.5 cutoff is based on several trial runs.
    [rA,cA] = size(A);  % Get the sizes of the inputs transposed.
    [rB,cB] = size(B);
    tflag = true; % For the correct output at the end.
end

nans = isnan(B(:));  % If B has NaNs, user expects to find match in A.

if any(nans)
    % There are at least two strategies for dealing with NaNs here.  One 
    % approach is to pick the largest finite number N between B and A,
    % then replace the NaNs in both matrices with (N + 1).  This has the
    % advantage of certainty when it comes to uniqueness.  Unfortunately,
    % this is slow for large problems.  The other approach is to assign the
    % NaNs in A and B to an 'unlikely' number.  This is much faster, but 
    % also has the risk of duplicating other elements in A or B and thereby
    % giving false results.  The odds against a conflict can be minimized
    % by choosing the unlikely number with some care.  First, the number
    % should not be on [0 1] because this is a very common range, often
    % encountered when working with: images, matrices derived using rand,
    % and normalized data.  Second, the number should not be an integer or
    % an explicit ratio of integers (3/5, 45673/2344), for obvious reasons.
    % Third, in this case we want the number to be a function of rand.  The
    % reason is that for very large problems, the first method above takes
    % up to 20+ times longer than the unlikely number method.  Thus if a
    % user is paranoid about trusting the output when using this method, 
    % even with all of the above precautions and exclusions, it will 
    % still be much faster to run the code twice and compare answers than 
    % to run the code once using the first method, with a few exceptions.
    % I have chosen speed over certainty, but also include the alternate 
    % method for convenience.  To switch to the first approach, uncomment 
    % the first line and comment out the second line below.
%     vct = max([B(isfinite(B(:)))',Atv(isfinite(Atv))]) + 1; % Certainty.
    vct = pi^pi -1/pi + 1/rand; % Unlikely number on [37.14...   9.0...e15]
    B(nans) = vct;  % Set the NaNs in both to vct.
    A(isnan(A)) = vct;
    clear nans  % This could be a large vector, save some memory.
end

if numel(B)<30  % The below method is faster for most small B matrices.
    A = A(:).';  % Make a single row vector for strfind
    vct = strfind(A,B(:,1).');  % Find occurrences of the first col of B.
    % Next eliminate wrap-arounds, this was much improved by Bruno Luong.
    idx = vct(mod(vct-1,rA) <= rA-rB);
    cnt = 2;  % Counter for the while loop.

    while ~isempty(idx) && cnt<=cB
        vct = strfind(A,B(:,cnt).');  % Look for successive columns.
        % The C helper function ismembc needs both args to be sorted.
        % Search the code in ismember for more information.
        idx = idx(ismembc(idx + (cnt-1)*rA,vct)); % Matches with previous?
        cnt = cnt+1;  % Increment counter.
    end
else % The below method is faster for most larger B matrices.
    idx = strfind(A(:).',B(:,1).');  % Occurrences of the first col of B.
    % Next eliminate wrap-arounds, this was much improved by Bruno Luong.
    idx = idx(mod(idx-1,rA) <= rA-rB);
    idx(idx>((cA-cB+1)*rA)) = []; % Too close to right edge.
    cnt = 2;  % Counter for the while loop.
    flag = true(1,length(idx));
    % Siyi Deng noticed that for large B, the previous algorithm was slow.
    % The code below reflects an account for this behavior.
    while cnt<=cB
        TMP = rA*(cnt-1);  % Just to make the cond below more intelligible.
        TMP2 = B(:,cnt)';
        for jj = 1:length(idx)
            if ~isequal(A(idx(jj)+TMP : idx(jj)+rB-1+TMP),TMP2)
                flag(jj) = false;
            end
        end
        cnt = cnt+1;  % Increment counter.
    end

    idx = idx(flag);
end

if tflag  % We must get the index to A from index to A'
    tmp = rem(idx-1,rA) + 1;  % A temporary variable.
    idx = 1 + (idx - tmp)/rA + (tmp - 1)*cA;
    rA = cA;  % Only used if user wants subscripts instead of Linear Index.
end

if nargout==2 % Get subscripts.
    tmp = rem(idx-1,rA) + 1; 
    idx2 = (idx - tmp)/rA + 1;
    idx = tmp;                        
end

end