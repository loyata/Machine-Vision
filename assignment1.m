%%Part 1
points = ones(2000*2000, 3);
RGB = ones(2000*2000, 3);
n = 1;
for y = -999:1:1000
    for x = -999:1:1000
        points(n, :) = [x y 100*sin(x/150)];
        RGB(n, :) = hsv2rgb([0.5*(x/1000+1) 1 0.5*(-y/1000+1)]);
        n = n+1;
    end
end

ptCloud = pointCloud(points, 'Color',RGB);
figure(1);
pcshow(ptCloud);
title('Part 1 Create a virtual scene');
xlabel('X');
ylabel('Y');
zlabel('Z');

%%Part 2
%% IM_Rcam1_RGB_f5.jpg
figure(2)
outputMap = computeProjection(5, 0, points, RGB);
image(outputMap)
title('3D virtual scene in RGB, f = 5');
xlabel('X');
ylabel('Y');


%% IM_Rcam1_HSV_f5.jpg
figure(3)
outputMap_hsv = outputMap;
for i=1:480
    for j=1:640
        outputMap_hsv(i,j,:) = rgb2hsv(outputMap(i,j,:));
    end
end
image(outputMap_hsv)
title('3D virtual scene in HSV');
xlabel('X');
ylabel('Y');

%% IM_Rcam1_RGB_f3.jpg
figure(4)
outputMap = computeProjection(3, 0, points, RGB);
image(outputMap)
title('3D virtual scene in RGB, f = 3');
xlabel('X');
ylabel('Y');

%% IM_Rcam1_RGB_f7.jpg
figure(5)
outputMap = computeProjection(7, 0, points, RGB);
image(outputMap)
title('3D virtual scene in RGB, f = 7');
xlabel('X');
ylabel('Y');


%%Part 3
%% IM_Rcam2_RGB_f5.jpg
figure(6)
outputMap = computeProjection(5, 2, points, RGB);
image(outputMap)
title('Cam2');
xlabel('X');
ylabel('Y');

%% IM_Rcam3_RGB_f5.jpg
figure(7)
outputMap = computeProjection(5, 3, points, RGB);
image(outputMap)
title('Cam3');
xlabel('X');
ylabel('Y');



function [outputMap] = computeProjection(f, cam_shift, points, RGB)
    outputMap = 0.5 * ones(480,640,3);
    P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 1/f 0];
    Q_obj2cam = [1 0 0 0; 0 1 0 0; 0 0 1 1000; 0 0 0 1];

    %% Camera Coordinate -> Pixel Coordinate
    dx = 12.7/640; %%mm/pixel
    dy = 12.7/480;
    u0 = 320;
    v0 = 240;
    Q_cam2pix = [1/dx 0 u0; 0 1/dy v0; 0 0 1];
    
    %% Cam1->Cam2
    Q_cam1cam2 = inv([0.866 0 -0.5 400; 0 1 0 0; 0.5 0 0.866 -600; 0 0 0 1]);
    %% Cam1->Cam3
    Q_cam1cam3 = inv([0.7071 0 0.7071 -1200; 0 1 0 0; -0.7071 0 0.7071 0; 0 0 0 1]);
    for num=1:2000*2000
        if cam_shift == 2
            point_cam = P * Q_cam1cam2 * Q_obj2cam * [points(num, :) 1]';
        end
        if cam_shift == 3
            point_cam = P * Q_cam1cam3 * Q_obj2cam * [points(num, :) 1]';
        end
        if cam_shift == 0
            point_cam = P * Q_obj2cam * [points(num, :) 1]';
        end

        x = point_cam(1,1);
        y = point_cam(2,1);
        scale = point_cam(4,1);
        point_cartesian = [x/scale y/scale 1]';
        point_pixel = Q_cam2pix * point_cartesian;
        w = floor(point_pixel(2,1));
        h = floor(point_pixel(1,1));
        if w > 480 || w < 1 || h > 640 || h < 1
            continue
        end
        outputMap(w,h,:) = RGB(num, :);
    end
end



