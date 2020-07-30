close all;
clear;
b = ble("588E81A54A84");%change this to either the name or address of the device
%c = characteristic(b,"74A4936E-672D-486F-97FE-95C4BE54B66A","CC0BAAE5-8D87-4C6D-8C1C-20AF6C2E20E8");
c = characteristic(b,"4880C12C-FDCB-4077-8920-A450D7F9B907","FEC26EC4-6D71-4442-9F81-55BC21D658D6");
%read/write service and characteristic
%If using same gatt files the service and characteristic ID should be the
%same, if not you can find their IDs by calling blelist in command window
subscribe(c);

c.DataAvailableFcn = @displayCharacteristicData;
data(244) = 0;
function displayCharacteristicData(src,evt)
    fileID = fopen('data.txt','a+');
    data = read(src,'oldest');
    for i=1:244
        if (data(i)>127)
            data(i) = data(i)-256;
        end
    end
    fprintf(fileID,'%d ',data);
    disp(data);
end