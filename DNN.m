function out = DNN( in, ...
    pw1,pw2,pw3,pw4, ...
    pb1,pb2,pb3,pb4, ...
    Pt_max,Pt_min,p_max,p_min)
%% 神经网络正向传播

norm_in = (in - Pt_min)./(Pt_max-Pt_min);
% 原问题
L1=pw1*norm_in+pb1';
L1=max(L1,0)-0.01*max(-L1,0);
L2=pw2*L1+pb2';
L2=max(L2,0)-0.01*max(-L2,0);
L3=pw3*L2+pb3';
L3=max(L3,0)-0.01*max(-L3,0);
% L4=pw4*L3+pb4';
% L4=max(L4,0)-0.01*max(-L4,0);
% L5=pw5*L4+pb5';
% L5=max(L5,0)-0.01*max(-L5,0);
norm_out = logsig(pw4*L3+pb4');
out = norm_out.*(p_max-p_min) + p_min;

end




function sout = satura(sin)
        sout = sin ;
        if sout > 12
            sout = 12 ;
        elseif sout < 0 
            sout = 0;
        end
end
            


%% 输出
% if norm(p)>u_max
%     p=p/norm(p)*u_max;
% end
% ud=p(1);
% uq=p(2);



