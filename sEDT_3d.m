function Q = sEDT_3d(A)

%Compute 3D signed EDT, where A is the binary map of voxels
%Note that no 0 cost boundary voxels are used for simplicity

Ainv = 1 - A;

Q = bwdist(A, 'Euclidean') - bwdist(Ainv, 'Euclidean');

end