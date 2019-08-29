function [e] = gen_color_noise(L,sigma2,c)
nc = length(c) - 1;
xik=zeros(nc,1);%白噪声初值
xi=randn(L,1)*sqrt(sigma2);%产生均值为0，方差为sigma2的高斯白噪声序列
e = zeros(1,L);
for k=1:L
    e(k)=c*[xi(k);xik];%产生有色噪声
    %数据更新
    for i=nc:-1:2
        xik(i)=xik(i-1);
    end
    xik(1)=xi(k);
end

end

