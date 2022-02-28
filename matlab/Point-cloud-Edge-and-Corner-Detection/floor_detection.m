function remainPtCloud = floor_detection(ptCloud, showfig)

    maxDistance = 0.02;
    referenceVector = [0,1,0];
    maxAngularDistance = 5;
    [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
            maxDistance,referenceVector,maxAngularDistance);
    plane1 = select(ptCloud,inlierIndices);
    remainPtCloud = select(ptCloud,outlierIndices);
    if showfig
    figure
    pcshow(ptCloud)
    xlabel('X(m)')
    ylabel('Y(m)')
    zlabel('Z(m)')
    title('Original Point Cloud')
    figure
    pcshow(plane1)
    title('First Plane')

    figure
    pcshow(remainPtCloud)
    title('Remaining Point Cloud')
    end
% roi = [-inf,inf;0.4,inf;-inf,inf];
% sampleIndices = findPointsInROI(remainPtCloud,roi);
% [model2,inlierIndices,outlierIndices] = pcfitplane(remainPtCloud,...
%             maxDistance,'SampleIndices',sampleIndices);
% plane2 = select(remainPtCloud,inlierIndices);
% remainPtCloud = select(remainPtCloud,outlierIndices);
% figure
% pcshow(plane2)
% title('Second Plane')
end
