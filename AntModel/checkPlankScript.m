pointArray = contactsTable.("Contact Location")
normArray = contactsTable.("Surface Normal")
nContact = size(pointArray,1);
alignMat = nan([nContact, nContact]);
idx = [];



for n = 1:nContact
    for m = 1:nContact
        if n~= m
            contactPair = pointArray([n, m],:);
            forcePair = genOpposeForces(contactPair);
            normalPair = normArray([n,m],:);

            alignPair = min(tbox.findSurfNormAlign(normalPair, forcePair));
            alignMat(n,m) = alignPair;
            alignMat(m,n) = alignPair;
        end

    end
end


bestAlign = max(alignMat,[], 'all');
[idx(:,1), idx(:,2)] = find(alignMat == bestAlign);

[distanceMat, ~, ~] = findInterPointDistance(pointArray, "none");

function [forces] = genOpposeForces(contacts)
%Generate a pair of forces that point in the direction of
%the other contact point as if pinched in a vice
contactA = contacts(1,:);
contactB = contacts(2,:);
force_a = contactB - contactA;
forceA_norm = force_a/vecnorm(force_a);
forceB_norm = -forceA_norm;
forces = [forceA_norm;forceB_norm];

end

function [distanceMAT, distance, idx] = findInterPointDistance(contact_points, type)


distanceMAT = pdist2(contact_points, contact_points);
distanceMAT(distanceMAT==0) = nan;


switch type
    case "min"
        distance = min(distanceMAT, [], 'all');

    case "max"
        distance = max(distanceMAT, [], 'all');

    otherwise
        distance = nan;
        %Min or Max not selected - no problem
end
[row, col] = find(distanceMAT == distance);
if and(~isempty(row), ~isempty(col))
    idx = [row(1), col(1)];
else
    idx = [];
end
end