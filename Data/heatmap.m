[DATA] = xlsread('150cm.csv');
Channel = DATA(:,1);
for i=1:length(DATA(:,1))
    if Channel(i) < 11 
        Frequency(i) = (2 * Channel(i)) + 2404; %in MHz;
    else
        Frequency(i) = (2 * Channel(i)) + 2406; %in MHz;
    end
end
RSSI = DATA(:,2);
tbl = table(Frequency, RSSI);
heatmap(tbl,'Frequency','RSSI')
title('150cm');
xlabel('Frequency (MHz)');