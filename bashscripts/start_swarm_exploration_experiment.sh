#!/bin/bash

#Assign which crazyflies toflash

CF_id_array=("$@")
#Flash NRF

cd /home/knmcguire/Documents/Onedrive/PhD/system_test/scripts/python
echo "Start FLYING"
radio_id=0
for i in "${CF_id_array[@]}"
do
#if [ $(($i%2)) -eq 0 ];
#then
channel=$(($i * 10))
#else
#channel=$(($(($i-1)) * 10))
#fi

echo "radio://$radio_id/$channel/2M/E7E7E7E70$i"
echo $(($(($i-2))*100))

if [ $i -lt 6 ];
then
Y=0
else
Y=500
fi

X=$(($(($(($i-2))%4))*500))
echo $X
echo $Y
xterm -title "$i: flying and logging" -geometry 71x31+$X+$Y -hold -e "python cf2_gbug_logging.py $radio_id $i"  &\

radio_id=$((radio_id+1))
done

echo "run 'killall -9 xterm' to kill all"

