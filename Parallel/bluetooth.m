close all;
clear;
b1 = ble("588E81A5491F");%change this to either the name or address of the device
b2 = ble("588E81A54A84");%secondary address
%c = characteristic(b,"74A4936E-672D-486F-97FE-95C4BE54B66A","CC0BAAE5-8D87-4C6D-8C1C-20AF6C2E20E8");
c1 = characteristic(b1,"417125c7-6849-422b-bfb0-f4312d00c281","FEC26EC4-6D71-4442-9F81-55BC21D658D6");
%c2 = characteristic(b1,"417125c7-6849-422b-bfb0-f4312d00c281","FEC26EC4-6D71-4442-9F81-55BC21D658D6");
%read/write service and characteristic
%If using same gatt files the service and characteristic ID should be the
%same, if not you can find their IDs by calling blelist in command window
subscribe(c1);
%subscribe(c2);
c1.DataAvailableFcn = @displayCharacteristicData;
%c2.DataAvailableFcn = @displayCharacteristicData;
function displayCharacteristicData(src,evt)
    fileID = fopen('150cmdata.txt','a+');
    data = read(src,'oldest');
    for i=1:length(data)
        if (data(i)>127)
            data(i) = data(i)-256;
        end
    end
    fprintf(fileID,'%d ',data);
    disp(data);
end
