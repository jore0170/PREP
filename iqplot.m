close all;
clear;
fileID = fopen('test2.txt','r');
Data1 = textscan(fileID, '%f%s');
ChannelArray = Data1{1,1};
Channel = 12;%choose channel here
instance = find(ChannelArray == Channel);
iteration = 1;%choose which channel iteration
fclose(fileID);
fileDD = fopen('test2.txt','r');
Data = textscan(fileDD, '%f%s','Delimiter',',');
IQ = zeros(length(Data1{1,2}),165);
for i = 1:length(Data1{1,2})
     IQ(i,:) = cellfun(@str2double,(strsplit(Data1{1,2}{i,1},',')));  
end
IQ(:,1) = [];%get rid of NaN
x = IQ(instance(iteration),1:2:end);
y = IQ(instance(iteration),2:2:end);
Phase = atan(x./y);
psuedotime = linspace(1,82,82);
figure
plot(psuedotime, Phase);
title("Wrapped Channel " + Channel + " Iteration" + iteration)
for i = 1:length(Phase)-1
    if abs((Phase(i)-Phase(i+1)))>3
        index(i) = i;
    end
end
k = find(index);
if (Phase(k(1))-Phase(k(1)+1)) < 0 %negative slope case
    Phase((k(1)+1):k(2)) = Phase((k(1)+1):k(2)) - pi;
    Phase((k(2)+1):length(Phase)) = Phase((k(2)+1):length(Phase)) - (2*pi);
else % positive slope
    Phase(k(1):k(2)) = Phase(k(1):k(2)) + pi;
    Phase((k(2)+1):length(Phase)) = Phase((k(2)+1):length(Phase)) + (2*pi);

end
figure
plot(psuedotime, Phase)
title("UnWrapped Channel " + Channel + " Iteration" + iteration)
