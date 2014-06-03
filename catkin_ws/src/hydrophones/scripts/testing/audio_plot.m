M = csvread('audio.csv');

for i = 1:4
    subplot(4,1,i);
    beginning = max(0,(i-1)*1024+1)
    ending = 1024*i;
    plot(M(beginning:ending));
end
