% % test


tic
m = 1000;
n = 1000;
v = 200:-1:10;
result1 = reshape(repmat(v,m*n,1),m,n,[]);
toc

tic
x = 1000;
y = 1000;
v = 200:-1:10;
cOut = arrayfun(@(dIn,nInd) ones(x,y) .* dIn, v, 1:length(v), 'UniformOutput',false);
dOut = cat(3,cOut{:});
toc
