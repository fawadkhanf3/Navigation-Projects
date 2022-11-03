for jj = 1:k
   
    Xhati = interval(Xhat{jj});
    xlow = Xhati.inf;
    xup  = Xhati.sup;
    
    xlowpitch(jj,:) = xlow(2);
    xuppitch(jj,:) = xup(2);
    
end

figure;hold on
plot(true_time(1:jj),true_pitch(1:jj)*180/pi,'r.-');
plot(true_time(1:jj),xhat(2,1:jj)*180/pi,'b.-');

plot(true_time(1:jj),xlowpitch(1:jj,1),'k--');
plot(true_time(1:jj),xuppitch(1:jj,1),'k--');
grid;shg