close all;
clear;
b = ble("588E81A5491F");
%c = characteristic(b,"74A4936E-672D-486F-97FE-95C4BE54B66A","CC0BAAE5-8D87-4C6D-8C1C-20AF6C2E20E8");
c = characteristic(b,"4880C12C-FDCB-4077-8920-A450D7F9B907","FEC26EC4-6D71-4442-9F81-55BC21D658D6");

%read/write service and characteristic
%write(c,1); % writing binary 1
subscribe(c);

c.DataAvailableFcn = @displayCharacteristicData;

function displayCharacteristicData(src,evt)
    data = read(src,'oldest');
    disp(char(data));
end