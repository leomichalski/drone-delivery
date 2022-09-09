
#### Explicação dos arquivos
* **drone-delivery**: pasta do repositório.
  * **flight**: pasta do voo e das simulações de voo.
  * **vision**: pasta do código relacionado a Visão Computacional.
  * **scripts**: pasta com scripts úteis. Alguns deles são rodados quando a raspberry é ligada.

#### Como rodar o código na Raspberry 4 (ou 3B+) conectada na PX4
```
taskset -c 0,1,2 python3 main.py --webstream-video --detect-aruco --using-ros  --frames-per-second 7
# usar a flag "--using-vpn" se for usar a vpn para o video streaming

taskset -c 3 roslaunch mavros px4.launch
taskset -c 3 rosrun aruco_hunter main
```
