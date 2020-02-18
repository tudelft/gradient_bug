#!/bin/bash

#Assign which crazyflies toflash

CF_id_array=("$@")


#Flash NRF
cd /home/knmcguire/Software/crazyflie/crazyflie2-nrf-firmware
make clean && make BLE=0
echo "Flash NRF"
for i in "${CF_id_array[@]}"
do
#if [ $(($i%2)) -eq 0 ];
#then
channel=$(($i * 10))
#else
#channel=$(($(($i-1)) * 10))
#fi



if [ $i -eq 10 ];
then
URI=radio://0/$channel/2M/E7E7E7E710
else
URI=radio://0/$channel/2M/E7E7E7E70$i
fi

echo "$URI"
python3 -m cfloader -w $URI flash cf2_nrf.bin nrf51-fw
#xterm -title "2: Flash NRF" -e "python3 -m cfloader -w $URI flash cf2_nrf.bin nrf51-fw"  
done

#Flash STM
cd /home/knmcguire/Software/crazyflie/crazyflie-firmware
make clean && make
echo "Flash STM"
for i in "${CF_id_array[@]}"
do
#if [ $(($i%2)) -eq 0 ];
#then
channel=$(($i * 10))
#else
#channel=$(($(($i-1)) * 10))
#fi

if [ $i -eq 10 ];
then
URI=radio://0/$channel/2M/E7E7E7E710
else
URI=radio://0/$channel/2M/E7E7E7E70$i
fi
echo "radio://0/$channel/2M/E7E7E7E70$i"
python3 -m cfloader -w $URI flash  cf2.bin stm32-fw
#xterm -title "2: Flash NRF" -e "python3 -m cfloader -w $URI flash  cf2.bin stm32-fw"  
done


