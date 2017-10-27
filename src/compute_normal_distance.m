function [Fn, Jthetan, Jbetan, Jrn, Jcenters_fingern, Jcenters_palmn, Jr_palmn, Js_membranen, Jglobal_rotationn, Jglobal_translationn,Jr_foldn,Joffset_foldn ] = compute_normal_distance( camera_normals,model_normals, F, Jtheta, Jbeta, Jr,Jcenters_finger, Jcenters_palm, Jr_palm,Js_membrane, Jglobal_rotation,Jglobal_translation,Jr_fold,Joffset_fold, D)

Fn = zeros(length(camera_normals), 1);
Jthetan = zeros(length(camera_normals), 20);
Jbetan = zeros(length(camera_normals), 16);
Jrn = zeros(length(camera_normals), 21);
Jcenters_palmn = zeros(length(camera_normals),36);
Jr_palmn = zeros(length(camera_normals),12);
Jcenters_fingern = zeros(length(camera_normals),15);
Js_membranen = zeros(length(camera_normals),4);
Jglobal_rotationn = zeros(length(camera_normals),3);
Jglobal_translationn = zeros(length(camera_normals),3);
Jr_foldn = zeros(length(camera_normals),1);
Joffset_foldn = zeros(length(camera_normals),3);

for i = 1:length(camera_normals)
    if isempty(camera_normals{i}) || isempty(model_normals{i}) , continue; end
    c = camera_normals{i};
    n = model_normals{i};
    
    Fn(i) =  c' * (n * n') * F(D * (i - 1) + 1:D * i);
    Jthetan(i, :) =  c' * (n * n') * Jtheta(D * (i - 1) + 1:D * i, :);
    Jbetan(i, :) =  c' * (n * n') * Jbeta(D * (i - 1) + 1:D * i, :);
    Jrn(i, :) =  c' * (n * n') * Jr(D * (i - 1) + 1:D * i, :); 
    Jcenters_palmn(i,:) = c' * (n * n') * Jcenters_palm(D * (i - 1) + 1:D * i, :); 
    Jr_palmn(i,:) = c' * (n * n') * Jr_palm(D * (i - 1) + 1:D * i, :); 
    Jcenters_fingern(i,:) = c' * (n * n') * Jcenters_finger(D * (i - 1) + 1:D * i, :); 
    Js_membranen(i,:) = c' * (n * n') * Js_membrane(D * (i - 1) + 1:D * i, :); 
    Jglobal_rotationn(i,:) = c' * (n * n') * Jglobal_rotation(D * (i - 1) + 1:D * i, :); 
    Jglobal_translationn(i,:) = c' * (n * n') * Jglobal_translation(D * (i - 1) + 1:D * i, :); 
    Jr_foldn(i,:)= c' * (n * n') * Jr_fold(D * (i - 1) + 1:D * i, :); 
    Joffset_foldn(i,:)= c' * (n * n') * Joffset_fold(D * (i - 1) + 1:D * i, :); 
end