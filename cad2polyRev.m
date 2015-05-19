
function [P,V] = cad2polyRev(FileName)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main File : cad2poly.m 
% Source Files: cad2mat.m 
% Description : Converts CAD geometry into multiple polygons and plots
%               the resulting geometry
% Input       : filename, filename2 -filenames of the geometry to convert
% Output      : p, V, p2, V2 -The corresponding polygons and vertices of
%               the geometry in filename and filename2, respectively.
% Author      : Dr. L.A. Rodriguez
% Date        : 02/04/2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Converts CAD data to MATLAB using the cad2ply.m file, which is a 
% modified version of the cad2matdemo.m file located on the 
% Mathworks central file exchange.
h = figure(1);
%h = subplot(122); %comment line 18 and 20 to work
clf;
n = length(FileName);

for i = 1:n,
    % F-faces, V-vertices, C-color
    [f,v,c] = cad2mat(FileName{i});
    P(i,1) = patch('faces', f, 'vertices' ,v);% create the polygons
    set(P(i,1), 'facec', 'flat');             % Set the face color flat
    set(P(i,1), 'FaceVertexCData', c);        % Set the color (from file)
    set(P(i,1), 'EdgeColor','none');          % Set the edge color
    % To use homogenous transformation matrices the n by 3 vertices will be 
    % turned to n by 4 vertices by augmenting them with ones, [V;1]
    V{i,1} = [v ones(length(v),1)]'; 
    
end


light                               % add a default light
daspect([1 1 1])                    % Setting the aspect ratio
view(3)                             % Plot isometric view

xlabel('X'),ylabel('Y'),zlabel('Z')
%title({['Imported Solidworks Geometry from ' filename ' and ' filename2] ' Used to animate a 2-link robot'})
title(['Imported Solidworks Geometry from '])
drawnow     
shg

